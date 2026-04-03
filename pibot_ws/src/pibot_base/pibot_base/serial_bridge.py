import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.time import Time

import serial

from geometry_msgs.msg import TransformStamped, Quaternion, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster

WHEEL_DIAMETER_M  = 0.065
WHEELBASE_M       = 0.159
TICKS_PER_REV     = 2140
DIST_PER_TICK     = math.pi * WHEEL_DIAMETER_M / TICKS_PER_REV

MAX_WHEEL_VEL     = (160.0 / 60.0) * math.pi * WHEEL_DIAMETER_M

# Minimum PWM to overcome motor stiction (0–100 scale)
MIN_SPEED         = 10


def _euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    """Convert roll/pitch/yaw (rad) to a geometry_msgs Quaternion."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


def _vel_to_motor_command(wheel_vel: float):
    """
    Convert a wheel velocity (m/s) to (speed 0-100, direction 1/-1).
    Returns (0, 1) for zero velocity.
    """
    if abs(wheel_vel) < 1e-4:
        return 0, 1

    direction = 1 if wheel_vel > 0 else -1
    # Map absolute velocity to MIN_SPEED–100 range
    speed = abs(wheel_vel) / MAX_WHEEL_VEL * 100.0
    speed = max(MIN_SPEED, min(100, int(speed)))
    return speed, direction


class SerialBridge(Node):
    """
    Reads Pico sensor data, publishes /odom and /imu/data_raw,
    and subscribes to /cmd_vel to drive the motors.
    """

    def __init__(self):
        super().__init__('serial_bridge')

        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('timeout', 1.0)

        port    = self.get_parameter('serial_port').get_parameter_value().string_value
        baud    = self.get_parameter('baud_rate').get_parameter_value().integer_value
        timeout = self.get_parameter('timeout').get_parameter_value().double_value

        self._odom_pub       = self.create_publisher(Odometry, '/odom', 10)
        self._imu_pub        = self.create_publisher(Imu, '/imu/data_raw', 10)
        self._tf_broadcaster = TransformBroadcaster(self)

        self._cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self._cmd_vel_callback,
            10,
        )

        self._x           = 0.0
        self._y           = 0.0
        self._theta       = 0.0
        self._prev_left   = None
        self._prev_right  = None
        self._prev_stamp  = None

        try:
            self._ser = serial.Serial(port, baud, timeout=timeout)
            self.get_logger().info(f'Opened serial port {port} at {baud} baud')
        except serial.SerialException as exc:
            self.get_logger().fatal(f'Cannot open serial port {port}: {exc}')
            raise SystemExit(1)

        self._serial_lock = threading.Lock()

        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

        self.get_logger().info('Serial bridge ready — listening on /cmd_vel')

    def _cmd_vel_callback(self, msg: Twist):
        linear  = msg.linear.x
        angular = msg.angular.z

        angular = max(-1.0, min(1.0, angular))
        linear  = max(-MAX_WHEEL_VEL, min(MAX_WHEEL_VEL, linear))

        left_vel  = linear - (angular * WHEELBASE_M / 2.0)
        right_vel = linear + (angular * WHEELBASE_M / 2.0)

        left_vel  = max(-MAX_WHEEL_VEL, min(MAX_WHEEL_VEL, left_vel))
        right_vel = max(-MAX_WHEEL_VEL, min(MAX_WHEEL_VEL, right_vel))

        self._send_motor_command(left_vel, right_vel)

    def _send_motor_command(self, left_vel: float, right_vel: float):
        command = f'{left_vel:.4f},{right_vel:.4f}\n'
        try:
            with self._serial_lock:
                self._ser.write(command.encode())
                self._ser.flush()
        except serial.SerialException as exc:
            self.get_logger().error(f'Serial write error: {exc}')

    def _read_loop(self):
        while not self._stop_event.is_set():
            try:
                with self._serial_lock:
                    raw = self._ser.readline()
            except serial.SerialException as exc:
                self.get_logger().error(f'Serial read error: {exc}')
                continue

            if not raw:
                continue

            try:
                line = raw.decode('utf-8', errors='ignore').strip()
            except Exception:
                continue

            self._parse_and_publish(line)

    def _parse_and_publish(self, line: str):
        parts = line.split(',')
        if len(parts) != 8:
            return

        try:
            left_ticks  = int(parts[0])
            right_ticks = int(parts[1])
            imu_ax      = float(parts[2])
            imu_ay      = float(parts[3])
            imu_az      = float(parts[4])
            imu_gx      = float(parts[5])
            imu_gy      = float(parts[6])
            imu_gz      = float(parts[7])
        except ValueError:
            return

        now = self.get_clock().now()
        self._publish_imu(now, imu_ax, imu_ay, imu_az, imu_gx, imu_gy, imu_gz)
        self._update_odometry(now, left_ticks, right_ticks)

    def _publish_imu(self, stamp: Time,
                     imu_ax, imu_ay, imu_az,
                     imu_gx, imu_gy, imu_gz):
        # IMU is rotated by 180° around up axis
        msg = Imu()
        msg.header.stamp    = stamp.to_msg()
        msg.header.frame_id = 'imu_link'

        msg.linear_acceleration.x = -imu_ay
        msg.linear_acceleration.y = -imu_ax
        msg.linear_acceleration.z =  imu_az

        msg.angular_velocity.x = -imu_gy
        msg.angular_velocity.y = -imu_gx
        msg.angular_velocity.z =  imu_gz

        msg.orientation_covariance[0] = -1.0

        for i in range(9):
            msg.linear_acceleration_covariance[i] = 0.0
            msg.angular_velocity_covariance[i]    = 0.0

        self._imu_pub.publish(msg)

    def _update_odometry(self, stamp: Time, left_ticks: int, right_ticks: int):
        if self._prev_left is None:
            self._prev_left  = left_ticks
            self._prev_right = right_ticks
            self._prev_stamp = stamp
            return

        d_left  = (left_ticks  - self._prev_left)  * DIST_PER_TICK
        d_right = (right_ticks - self._prev_right) * DIST_PER_TICK
        self._prev_left  = left_ticks
        self._prev_right = right_ticks

        d_s     = (d_left + d_right) / 2.0
        d_theta = (d_right - d_left) / WHEELBASE_M

        self._x     += d_s * math.cos(self._theta + d_theta / 2.0)
        self._y     += d_s * math.sin(self._theta + d_theta / 2.0)
        self._theta += d_theta

        self._theta = math.atan2(math.sin(self._theta), math.cos(self._theta))

        dt = (stamp - self._prev_stamp).nanoseconds / 1e9
        self._prev_stamp = stamp
        vx = d_s     / dt if dt > 0.0 else 0.0
        wz = d_theta / dt if dt > 0.0 else 0.0

        q         = _euler_to_quaternion(0.0, 0.0, self._theta)
        stamp_msg = stamp.to_msg()

        tf = TransformStamped()
        tf.header.stamp    = stamp_msg
        tf.header.frame_id = 'odom'
        tf.child_frame_id  = 'base_link'
        tf.transform.translation.x = self._x
        tf.transform.translation.y = self._y
        tf.transform.translation.z = 0.0
        tf.transform.rotation      = q
        self._tf_broadcaster.sendTransform(tf)

        odom = Odometry()
        odom.header.stamp    = stamp_msg
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_link'

        odom.pose.pose.position.x    = self._x
        odom.pose.pose.position.y    = self._y
        odom.pose.pose.position.z    = 0.0
        odom.pose.pose.orientation   = q

        odom.twist.twist.linear.x  = vx
        odom.twist.twist.angular.z = wz

        odom.pose.covariance[0]   = 0.05  # x
        odom.pose.covariance[7]   = 0.05  # y
        odom.pose.covariance[35]  = 0.1   # yaw
        odom.twist.covariance[0]  = 0.05  # vx
        odom.twist.covariance[35] = 0.1   # wz

        self._odom_pub.publish(odom)

    def destroy_node(self):
        # Stop motors before shutting down
        self._send_motor_command(0.0, 0.0)  # stop motors
        self._stop_event.set()
        if hasattr(self, '_ser') and self._ser.is_open:
            self._ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
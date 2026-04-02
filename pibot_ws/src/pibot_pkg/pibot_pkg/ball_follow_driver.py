import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from ball_follow_interface.msg import BoundingBox

import serial

class BallFollowDriver(Node):
	def __init__(self):
		super().__init__('ball_follow_driver')

		self.declare_parameter('serial_port', '/dev/ttyACM0')
		self.declare_parameter('baud_rate', 115200)
		self.declare_parameter('timeout', 1)
		self.declare_parameter('max_speed', 80.0)
		self.declare_parameter('image_width', 640.0)
		self.declare_parameter('bbox_target_width', 120.0)
		self.declare_parameter('bbox_timeout_sec', 0.5)
		self.declare_parameter('center_deadband_px', 15.0)
		self.declare_parameter('center_filter_alpha', 0.3)
		self.declare_parameter('max_angular_cmd', 35.0)

		self.declare_parameter('distance_kp', 0.6)
		self.declare_parameter('distance_ki', 0.0)
		self.declare_parameter('distance_kd', 0.05)

		self.declare_parameter('center_kp', 0.12)
		self.declare_parameter('center_ki', 0.0)
		self.declare_parameter('center_kd', 0.008)

		serial_port_name = self.get_parameter('serial_port').value
		baud_rate = self.get_parameter('baud_rate').value
		timeout = self.get_parameter('timeout').value

		try:
			self._serial_port = serial.Serial(
				port=serial_port_name,
				baudrate=baud_rate,
				timeout=timeout
			)
			self.get_logger().info(
				f'Serial connected on {serial_port_name} @ {baud_rate} baud (timeout={timeout}s)'
			)
		except serial.SerialException as e:
			self.get_logger().error(f'Failed to open serial port {serial_port_name}: {e}')
			raise

		self._max_speed = float(self.get_parameter('max_speed').value)
		self._image_width = float(self.get_parameter('image_width').value)
		self._target_width = float(self.get_parameter('bbox_target_width').value)
		self._bbox_timeout_sec = float(self.get_parameter('bbox_timeout_sec').value)
		self._center_deadband_px = float(self.get_parameter('center_deadband_px').value)
		self._center_filter_alpha = float(self.get_parameter('center_filter_alpha').value)
		self._max_angular_cmd = float(self.get_parameter('max_angular_cmd').value)

		self._distance_kp = float(self.get_parameter('distance_kp').value)
		self._distance_ki = float(self.get_parameter('distance_ki').value)
		self._distance_kd = float(self.get_parameter('distance_kd').value)

		self._center_kp = float(self.get_parameter('center_kp').value)
		self._center_ki = float(self.get_parameter('center_ki').value)
		self._center_kd = float(self.get_parameter('center_kd').value)

		self._bbox_msg = None
		self._last_bbox_time = self.get_clock().now()
		self._last_control_time = self.get_clock().now()

		self._distance_integral = 0.0
		self._distance_prev_error = 0.0
		self._center_integral = 0.0
		self._center_prev_error = 0.0
		self._filtered_bbox_center_x = None

		self._bbox_subscription = self.create_subscription(
			BoundingBox,
			'/ball/bounding_box',
			self._bbox_callback,
			10
		)

		self._timer = self.create_timer(0.05, self._driver_callback)
		self.get_logger().info('Ball follow driver ready')

	def _bbox_callback(self, msg):
		self._bbox_msg = msg
		self._last_bbox_time = self.get_clock().now()

	def _to_motor_command(self, value):
		if value > 0:
			direction = 1
		elif value < 0:
			direction = -1
		else:
			direction = 0

		speed = int(min(abs(value), self._max_speed))
		return speed, direction

	def _stop_motors(self):
		self._send_motor_command(0, 0, 0, 0)

	def _reset_pid_state(self):
		self._distance_integral = 0.0
		self._distance_prev_error = 0.0
		self._center_integral = 0.0
		self._center_prev_error = 0.0
		self._filtered_bbox_center_x = None

	def _send_motor_command(self, left_speed, right_speed, left_dir, right_dir):
		try:
			command = f'{left_speed}, {right_speed}, {left_dir}, {right_dir}'
			self._serial_port.write(f'{command}\n'.encode())
			self._serial_port.flush()
			self.get_logger().info(f'Sent motor command: {command}')
		except serial.SerialException as e:
			self.get_logger().error(f'Failed to write to serial port: {e}')

	def _driver_callback(self):
		now = self.get_clock().now()
		dt = (now - self._last_control_time).nanoseconds / 1e9
		if dt <= 0.0:
			dt = 1e-3
		self._last_control_time = now

		if self._bbox_msg is None:
			self._reset_pid_state()
			self._stop_motors()
			return

		age_sec = (now - self._last_bbox_time).nanoseconds / 1e9
		if age_sec > self._bbox_timeout_sec:
			self._reset_pid_state()
			self._stop_motors()
			return

		bbox = self._bbox_msg
		if bbox.width <= 0 or bbox.height <= 0:
			self._reset_pid_state()
			self._stop_motors()
			return

		# Distance control uses bounding-box width: smaller width means the ball is farther.
		distance_error = self._target_width - float(bbox.width)
		self._distance_integral += distance_error * dt
		distance_derivative = (distance_error - self._distance_prev_error) / dt
		self._distance_prev_error = distance_error

		linear_cmd = (
			self._distance_kp * distance_error +
			self._distance_ki * self._distance_integral +
			self._distance_kd * distance_derivative
		)

		# Centering control uses horizontal pixel error from image center.
		bbox_center_x = float(bbox.x) + (float(bbox.width) / 2.0)
		if self._filtered_bbox_center_x is None:
			self._filtered_bbox_center_x = bbox_center_x
		else:
			alpha = self._center_filter_alpha
			self._filtered_bbox_center_x = (
				alpha * bbox_center_x + (1.0 - alpha) * self._filtered_bbox_center_x
			)

		horizontal_error = self._filtered_bbox_center_x - (self._image_width / 2.0)
		if abs(horizontal_error) < self._center_deadband_px:
			horizontal_error = 0.0
			self._center_integral = 0.0
			self._center_prev_error = 0.0
		self._center_integral += horizontal_error * dt
		center_derivative = (horizontal_error - self._center_prev_error) / dt
		self._center_prev_error = horizontal_error

		angular_cmd = (
			self._center_kp * horizontal_error +
			self._center_ki * self._center_integral +
			self._center_kd * center_derivative
		)
		angular_cmd = max(-self._max_angular_cmd, min(self._max_angular_cmd, angular_cmd))

		left_cmd = linear_cmd + angular_cmd
		right_cmd = linear_cmd - angular_cmd

		left_speed, left_dir = self._to_motor_command(left_cmd)
		right_speed, right_dir = self._to_motor_command(right_cmd)

		self._send_motor_command(left_speed, right_speed, left_dir, right_dir)

def main(args=None):
	rclpy.init()
	node = BallFollowDriver()
	try:
		rclpy.spin(node)
	except (KeyboardInterrupt, ExternalShutdownException):
		pass
	finally:
		if node is not None:
			node.destroy_node()
		if rclpy.ok():
			rclpy.shutdown()

if __name__ == '__main__':
	main()
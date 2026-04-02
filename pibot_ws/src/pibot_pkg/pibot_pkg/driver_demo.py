import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

import serial

class DriverDemo(Node):
	def __init__(self):
		super().__init__('driver')

		self.declare_parameter('serial_port', '/dev/ttyACM0')
		self.declare_parameter('baud_rate', 115200)
		self.declare_parameter('timeout', 1)

		self._serial_port = serial.Serial(
			port=self.get_parameter('serial_port').value,
			baudrate=self.get_parameter('baud_rate').value,
			timeout=self.get_parameter('timeout').value
		)

		self._state = 0
		self._timestamp = self.get_clock().now()

		self._timer = self.create_timer(0.1, self._driver_callback)

	def _driver_callback(self):
		elapsed = self.get_clock().now() - self._timestamp
		if elapsed.nanoseconds > 1e9:  # 100 ms
			self._timestamp = self.get_clock().now()
			self._state = (self._state + 1) % 3
			self.get_logger().info(f'State: {self._state}')
		
		if self._state == 0:
			left_speed = 50
			right_speed = 50
			left_dir = 1
			right_dir = 1
		elif self._state == 1:
			left_speed = 50
			right_speed = 0
			left_dir = 1
			right_dir = -1
		elif self._state == 2:
			left_speed = 0
			right_speed = 0
			left_dir = 0
			right_dir = 0
		else:
			self.get_logger().warn(f'Unknown state: {self._state}')
			left_speed = 0
			right_speed = 0
			left_dir = 0
			right_dir = 0
		
		self._serial_port.write(
			f'{left_speed}, {right_speed}, {left_dir}, {right_dir}\n'.encode()
		)
		self._serial_port.flush()

def main(args=None):
	rclpy.init()
	node = DriverDemo()
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
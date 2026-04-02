import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image

import cv2
import numpy as np
from cv_bridge import CvBridge
from ball_follow_interface.msg import BoundingBox

class BallFollowTracker(Node):
	def __init__(self):
		super().__init__('ball_follow_tracker')
		self.get_logger().info('Initializing Ball Follow Tracker')

		self.bridge = CvBridge()

		self.subscription = self.create_subscription(
			Image,
			'/camera/image_raw',
			self.image_callback,
			10
		)
		self.bbox_publisher = self.create_publisher(BoundingBox, '/ball/bounding_box', 10)
		self.get_logger().info('Waiting for image...')

	def image_callback(self, msg):
		try:
			frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

			# Convert to HSV for color detection
			blurred = cv2.GaussianBlur(frame, (11, 11), 0)
			hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

			# Define yellow color range
			lower_yellow = np.array([20, 100, 100])
			upper_yellow = np.array([30, 255, 255])

			# Create mask for yellow
			mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
			mask = cv2.erode(mask, None, iterations=2)
			mask = cv2.dilate(mask, None, iterations=2)

			# Find contours in the mask
			contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

			# Filter contours by area (approximate tennis ball size: 200-2000 pixels)
			# min_area = 200
			# max_area = 2000
			# ball_contours = [c for c in contours if min_area < cv2.contourArea(c) < max_area]
			ball_contours = contours  # For now, consider all contours

			bbox = BoundingBox()
			if ball_contours:
				largest = max(ball_contours, key=cv2.contourArea)
				x, y, w, h = cv2.boundingRect(largest)
				bbox.x = x
				bbox.y = y
				bbox.width = w
				bbox.height = h

			self.bbox_publisher.publish(bbox)
		except Exception as e:
			self.get_logger().error(f'Failed to process image: {e}')

def main(args=None):
	rclpy.init()
	node = BallFollowTracker()
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
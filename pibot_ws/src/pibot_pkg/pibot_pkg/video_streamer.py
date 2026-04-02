import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ball_follow_interface.msg import BoundingBox
from flask import Flask, Response
import cv2
import threading
import time

class VideoStreamer(Node):
    def __init__(self):
        super().__init__('video_streamer')
        self.get_logger().info('Initializing Video Streamer')

        self.bridge = CvBridge()
        self.frame = None
        self.bbox = None
        self.lock = threading.Lock()

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.bbox_subscription = self.create_subscription(
            BoundingBox,
            '/ball/bounding_box',
            self.bbox_callback,
            10
        )

        self.app = Flask(__name__)

        @self.app.route('/')
        def index():
            return """
            <html>
            <head>
                <title>PiBot Video Stream</title>
            </head>
            <body>
                <h1>PiBot Video Stream</h1>
                <img src="/video_feed" style="width:100%; max-width:640px;">
            </body>
            </html>
            """

        @self.app.route('/video_feed')
        def video_feed():
            return Response(self.generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

        # Run Flask in a separate thread
        self.thread = threading.Thread(target=self.app.run, kwargs={'host': '0.0.0.0', 'port': 5000, 'debug': False})
        self.thread.daemon = True
        self.thread.start()

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self.lock:
                self.frame = frame
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')

    def bbox_callback(self, msg):
        with self.lock:
            self.bbox = msg

    def generate(self):
        while rclpy.ok():
            with self.lock:
                frame = self.frame.copy() if self.frame is not None else None
                bbox = self.bbox

            if frame is not None:
                if bbox is not None and bbox.width > 0 and bbox.height > 0:
                    cv2.rectangle(
                        frame,
                        (bbox.x, bbox.y),
                        (bbox.x + bbox.width, bbox.y + bbox.height),
                        (0, 255, 0), 2
                    )
                    for i, text in enumerate([
                        f'x: {bbox.x}',
                        f'y: {bbox.y}',
                        f'w: {bbox.width}',
                        f'h: {bbox.height}',
                    ]):
                        cv2.putText(frame, text, (10, 22 + i * 22),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                elif bbox is not None:
                    cv2.putText(frame, 'No yellow blob detected', (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

                ret, jpeg = cv2.imencode('.jpg', frame)
                if ret:
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n\r\n')

            time.sleep(0.033)

def main(args=None):
    rclpy.init(args=args)
    node = VideoStreamer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
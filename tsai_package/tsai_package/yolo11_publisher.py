# Publisher Node Code
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class VideoPublisher(Node):
    def __init__(self):
        super().__init__('video_publisher')
        self.publisher_ = self.create_publisher(Image, 'video_frames', 10)
        self.timer = self.create_timer(0.033, self.timer_callback)  # 30 FPS
        self.cap = cv2.VideoCapture(0)  # Open video file
        self.bridge = CvBridge()

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # Convert frame to ROS Image message
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher_.publish(msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    try:
        video_publisher = VideoPublisher()
        rclpy.spin(video_publisher)
    finally:
        video_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

# Subscriber Node Code
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

# Load YOLOv11 model
model = YOLO("yolo11n.pt")

class VideoSubscriberPublisher(Node):
    def __init__(self):
        super().__init__('video_subscriber_publisher')
        self.subscription = self.create_subscription(
            Image,
            'video_frames',
            self.listener_callback,
            10
        )
        self.result_publisher_ = self.create_publisher(Image, 'yolo_results', 10)
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # YOLO inference
        results = model.predict(source=frame, save=False, save_txt=False, device=0)
        annotated_frame = results[0].plot()  # Render detections on the frame

        # Convert annotated frame to ROS Image message and publish
        result_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
        self.result_publisher_.publish(result_msg)

        # Display the annotated frame
        cv2.imshow('YOLO Video Feed', annotated_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    try:
        video_subscriber_publisher = VideoSubscriberPublisher()
        rclpy.spin(video_subscriber_publisher)
    finally:
        video_subscriber_publisher.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

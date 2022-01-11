import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

import cv2
import ros2_numpy as rnp

class OpenCvCamPublisher(Node):

    def __init__(self):
        super().__init__(node_name = 'opencv_publisher')
        self.publisher = self.create_publisher(Image, 'image_raw', 10)

        self.create_timer(0.1, self.timer_callback)

        camSet='nvarguscamerasrc sensor_id=0 ! video/x-raw(memory:NVMM),width=1280, height=720, framerate=21/1, format=NV12 ! nvvidconv flip-method=0 ! video/x-raw,width=960, height=616, format=BGRx ! videoconvert ! appsink'
        self.video = cv2.VideoCapture(camSet)

        
    def timer_callback(self):

        ret, frame = self.video.read()

        if (ret):
            msg = rnp.msgify(Image, frame, encoding='bgr8' )
            self.publisher.publish(msg)

    def __del__(self):
        self.get_logger().info("destroy method called, clearing display")
        self.video.release()


def main(args=None):
    rclpy.init(args=args)

    subscriber = OpenCvCamPublisher()

    try:
        rclpy.spin(subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()
        subscriber.destroy_node()

if __name__ == '__main__':
    main()



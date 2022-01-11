import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

import cv2
import ros2_numpy as rnp

class OpenCvCamSubscriber(Node):

    def __init__(self):
        super().__init__(node_name = 'opencv_subscriber')
        self.subscription = self.create_subscription(Image, 'image_raw', self.listener_callback, 10)
        self.subscription
       
    def listener_callback(self, msg):


        img = rnp.numpify(msg)
        #msgify(Image, frame, encoding='bgr8' )
        
        cv2.imshow("Camera", img)
        cv2.waitKey(1)

    def __del__(self):
        self.get_logger().info("destroy method called, clearing display")
        cv2.destroyAllWindows()        


def main(args=None):
    rclpy.init(args=args)

    subscriber = OpenCvCamSubscriber()

    try:
        rclpy.spin(subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()
        subscriber.destroy_node()

if __name__ == '__main__':
    main()



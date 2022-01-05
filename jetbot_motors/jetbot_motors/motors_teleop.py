import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import readkeys

class MotorsTeleop(Node):

    def __init__(self):
        super().__init__(node_name = 'motor_teleop')
        self.publisher = self.create_publisher(String, 'key_vel', 10)
        self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        key = readkeys.getch()
        self.get_logger().info(key)
        msg = String()
        msg.data = key
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = MotorsTeleop()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()
        node.destroy_node()

if __name__ == '__main__':
    main()

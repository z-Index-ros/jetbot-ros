import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from jetbot_motors.Robot import Robot

class MotorsSubscriber(Node):

    def __init__(self):
        super().__init__(node_name = 'motor_subscriber')
        self.subscription = self.create_subscription(String, 'cmd_vel', self.listener_callback, 10)
        self.subscription

        self.robot = Robot()

    def listener_callback(self, msg):
        self.get_logger().info(msg.data)
        action = msg.data
        if (msg.data == "forward"): self.robot.forward(1)
        elif (msg.data == "backward"): self.robot.backward(1)
        elif (msg.data == "left"): self.robot.left(1)
        elif (msg.data == "right"): self.robot.right(1)
        else: self.robot.stop()
        
    def __del__(self):
        self.get_logger().info("destroy method called, clearing display")
        self.robot.stop


def main(args=None):
    rclpy.init(args=args)

    subscriber = MotorsSubscriber()

    try:
        rclpy.spin(subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()
        subscriber.destroy_node()

if __name__ == '__main__':
    main()

        

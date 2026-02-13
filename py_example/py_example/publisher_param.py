import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ParameterizedPublisher(Node):
    def __init__(self):
        super().__init__('parameterized_publisher')
        
        # Declare parameter for message prefix
        self.declare_parameter('message', 'Hello ROS 2!')
        
        # Get parameter value
        self.message = self.get_parameter('message').value
        
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(1.0, self.publish_message)
        self.counter = 0
        
        self.get_logger().info(f'Publisher node started! Message prefix: "{self.message}"')

    def publish_message(self):
        msg = String()
        msg.data = f'{self.message} {self.counter}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = ParameterizedPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

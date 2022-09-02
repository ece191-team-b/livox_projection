import rclpy
from rclpy.node import Node
from livox_interfaces.msg import CustomMsg

class TimeStampChanger(Node):
    def __init__(self):
        super().__init__('timestamp_changer')
        self.declare_parameter('input_topic', '/livox/lidar')
        self.declare_parameter('output_topic', '/livox/stamped')

        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value

        self.get_logger().info(f'input_topic: {self.input_topic}')
        self.get_logger().info(f'output_topic: {self.output_topic}')

        self.subscription = self.create_subscription(CustomMsg, self.input_topic, self.listener_callback, 10)
        self.publisher = self.create_publisher(CustomMsg, self.output_topic, 10)
    
    def listener_callback(self, msg):
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = TimeStampChanger()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

class JoyDebugger(Node):
    def __init__(self):
        super().__init__('joy_debugger')
        self.create_subscription(Joy, '/joy', self.callback, 10)

    def callback(self, data):
        self.get_logger().info(f"Buttons: {data.buttons}")
        self.get_logger().info(f"Axes: {data.axes}")
        self.get_logger().info('--------------------------')

def main(args=None):
    rclpy.init(args=args)
    node = JoyDebugger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

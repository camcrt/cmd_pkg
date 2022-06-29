import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('print_Z')
        self.subscription = self.create_subscription(
            PoseStamped,
            '/vrpn_client_node/Tello01/pose',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, data):
        self.get_logger().info(str(data.pose.position.z))


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
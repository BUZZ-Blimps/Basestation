import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, String


class Blimp(Node):

    def __init__(self, BlimpID):
        self.BlimpID = BlimpID
        # Define Fake Blimp's name
        self.nodeName = "Blimp" + str(BlimpID)
        # Init node
        super().__init__(self.nodeName)

        # Create publisher for /blimpID
        self.pub_blimpID = self.create_publisher(String, 'blimpID', 1)  # Relative name, within namespace

        # Create subscriber for /autoPanic
        self.sub_identify = self.create_subscription(
            Bool,
            '/identify',  # Absolute name
            self.subCallback_identify,
            1)

        # Create timer
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def subCallback_identify(self, msg):
        self.get_logger().info('Identify: %s' % str(msg.data))

    def timer_callback(self):
        # Publish /blimpID
        self.publishBlimpID()

    def publishBlimpID(self):
        msg = String()
        msg.data = str(self.BlimpID)
        self.pub_blimpID.publish(msg)
        self.get_logger().info("Published: %s" % msg.data)


def main(args=None):
    rclpy.init(args=args)

    Blimp1 = Blimp("1")

    rclpy.spin(Blimp1)

    Blimp1.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

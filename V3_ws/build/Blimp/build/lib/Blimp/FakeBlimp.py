import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, String


class FakeBlimp(Node):

    def __init__(self, BlimpID):
        self.BlimpID = BlimpID
        # Define Fake Blimp's name
        self.nodeName = "Blimp" + str(BlimpID)
        # Init node
        super().__init__(self.nodeName)

        # Create publisher for /blimpID
        self.pub_blimpID = self.create_publisher(String, 'blimpID', 1)

        # Create subscriber for /autoPanic
        self.sub_autoPanic = self.create_subscription(
            Bool,
            'autoPanic',
            self.subCallback_autoPanic,
            1)

        # Create timer
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def subCallback_autoPanic(self, msg):
        self.get_logger().info('AUTO PANIC: %s' % str(msg.data))

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

    FakeBlimp1 = FakeBlimp("1")

    rclpy.spin(FakeBlimp1)

    FakeBlimp1.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

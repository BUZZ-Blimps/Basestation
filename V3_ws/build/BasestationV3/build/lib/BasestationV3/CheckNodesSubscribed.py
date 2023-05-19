import rclpy
from rclpy.node import Node


class CheckNodesSubscribed(Node):
    def __init__(self):
        super().__init__('CheckNodesSubscribed')
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.numNodes = 0

    def timer_callback(self):
        infos = self.get_subscriptions_info_by_topic("autoPanic")
        for info in infos:
            print("info type:", type(info))
            print("info.node_name:", info.node_name)


def main(args=None):
    rclpy.init(args=args)

    checker = CheckNodesSubscribed()

    rclpy.spin(checker)

    checker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
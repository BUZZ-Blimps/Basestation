import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class BlimpNodeHandler:
    def __init__(self, parentNode, nodeName):
        self.parentNode = parentNode
        self.nodeName = nodeName
        self.timeCreated = self.parentNode.get_clock().now()

        self.blimpID = None
        self.lastReceived_blimpID = None
        self.parentNode.get_logger().info("Created node for %s" % nodeName)

    def subCallback_blimpID(self, msg):
        if self.blimpID is None:
            self.parentNode.get_logger().info('Identified new blimp with ID "%s"' % msg.data)
        self.blimpID = msg.data
        self.lastReceived_blimpID = self.parentNode.get_clock().now()


class CheckNodesSubscribed(Node):
    def __init__(self):
        super().__init__('CheckNodesSubscribed')
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.recognizedNodes = []
        self.blimpNodeHandlers = []
        self.timeout = 3

    def timer_callback(self):
        # Iterate through current nodes, look for new nodes
        infos = self.get_subscriptions_info_by_topic("autoPanic")
        for info in infos:
            nodeName = info.node_name
            if nodeName not in self.recognizedNodes:
                self.createBlimpNodeHandler(nodeName)

        # Print out names of all connected nodes
        connectedNodes = ""
        for node in self.recognizedNodes:
            connectedNodes += node + ", "
        self.get_logger().info("Connected nodes: %s" % connectedNodes)

        # Check for nodes that timed out
        for blimpNodeHandler in self.blimpNodeHandlers:
            currentTime = self.get_clock().now()
            if blimpNodeHandler.lastReceived_blimpID is None:  # Hasn't received blimpID yet
                if self.getElapsedTime(blimpNodeHandler.timeCreated) > self.timeout:
                    self.removeBlimpNodeHandler(blimpNodeHandler)
            else:  # Has received blimpID
                if self.getElapsedTime(blimpNodeHandler.lastReceived_blimpID) > self.timeout:
                    self.removeBlimpNodeHandler(blimpNodeHandler)

    def createBlimpNodeHandler(self, nodeName):
        if nodeName == "_NODE_NAME_UNKNOWN_":
            self.get_logger().info("FLAG: Node Name Unknown")
            return

        self.recognizedNodes.append(nodeName)

        # Create new node handler
        newBlimpNodeHandler = BlimpNodeHandler(self, nodeName)
        topic_blimpID = "/" + nodeName + "/blimpID"
        newBlimpNodeHandler.sub_blimpID = self.create_subscription(
            String, topic_blimpID, newBlimpNodeHandler.subCallback_blimpID, 1)

        self.blimpNodeHandlers.append(newBlimpNodeHandler)

    def removeBlimpNodeHandler(self, blimpNodeHandler):
        self.recognizedNodes.remove(blimpNodeHandler.nodeName)
        self.blimpNodeHandlers.remove(blimpNodeHandler)
        if blimpNodeHandler is not None:
            self.get_logger().info('Detected timeout of blimp ID "%s"' % blimpNodeHandler.blimpID)

    def getElapsedTime(self, prevTime):
        elapsedTime = self.get_clock().now() - prevTime
        elapsedTimeSec = elapsedTime.nanoseconds / 10**9
        return elapsedTimeSec


def main(args=None):
    rclpy.init(args=args)

    checker = CheckNodesSubscribed()

    rclpy.spin(checker)

    checker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
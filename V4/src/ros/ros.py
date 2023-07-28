# ROS2 Packages
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int64
from test_msgs.srv import BasicTypes

# Can change name of node to basestation later
class mainNode(Node):

    def __init__(self):
        super().__init__('main_node')
        self.auto_sub = self.create_subscription(Bool, '/Blimp1/auto', self.getAuto, 10)
        self.service = self.create_service(BasicTypes, 'BlimpID', self.getService)
        self.auto = None
        self.blimpName = None

        # Adam's Code #
        # Note from Austin: loopspeed not defined here
        # Create timer
        # timer_period = 1.0/self.loopSpeed
        # self.timer = self.create_timer(timer_period, self.timerLoop)

        # Define variables
        self.topicName_identify = "identify"
        self.recognizedNodes = []
        self.nodeHandlers = []
        self.timeout = 5
        self.numNewBlimps = 0
        # End of Adam's Code #

    def __del__(self):
        # Call main_node.destroy_node() to destroy the ROS 2 node
        self.destroy_node()
        print("\nDestroyed Node\n")
        # Call rclpy.shutdown() to release ROS 2 resources
        rclpy.shutdown()

    def getService(self, req, res):
        self.get_logger().info('Service request received, robot identified!')
        self.blimpName = req.string_value
        print(req.string_value)
        # Send response back to client
        res.bool_value = True
        return res

    def getAuto(self, msg):
        # self.get_logger().info('Auto: "%s"' % msg.data)
        # data_placeholder.write("Auto: {}".format(msg.data))
        self.auto = msg.data

    def getAutoData(self):
        return self.auto
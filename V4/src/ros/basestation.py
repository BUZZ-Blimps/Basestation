# ROS2 Packages
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Int64, Float64, Float64MultiArray
from test_msgs.srv import BasicTypes
import time

import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from globalVars import *

# Can change name of node to basestation later
class Basestation(Node):

    def __init__(self):
        super().__init__('Basestation')
        # Testing Subscriptions
        #self.auto_sub = self.create_subscription(Bool, '/Blimp1/auto', self.getAuto, 10)
        # Service
        # self.service = self.create_service(BasicTypes, 'BlimpID', self.getService)
        self.auto = None
        self.blimpName = None

        # Adam's Code #
        # Create timer
        self.loopSpeed = 1
        timer_period = 1.0/self.loopSpeed
        self.timer = self.create_timer(timer_period, self.timerLoop)

        # Define variables
        self.topicName_identify = "identify"
        self.recognizedNodes = []
        self.nodeHandlers = []
        self.timeout = 5
        self.numNewBlimps = 0

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
    
    def timerLoop(self):
        # Handle backend nodes
        self.handleBlimpNodes()
        # Allow publication
        for nodeHandle in self.nodeHandlers:
            nodeHandle.publish()

    def handleBlimpNodes(self):
        # Iterate through current nodes, look for new nodes
        infos = self.get_subscriptions_info_by_topic(self.topicName_identify)
        for info in infos:
            nodeName = info.node_name
            if nodeName not in self.recognizedNodes:
                self.createBlimpNodeHandler(nodeName)

        """
        # Print out names of all connected nodes
        connectedNodes = ""
        for node in self.recognizedNodes:
            connectedNodes += node + ", "
        self.get_logger().info("Connected nodes: %s" % connectedNodes)
        """

        # Check for nodes that timed out
        for blimpNodeHandler in self.nodeHandlers:
            if blimpNodeHandler.lastReceived_blimpID is None:  # Hasn't received blimpID yet
                if self.getElapsedTime(blimpNodeHandler.timeCreated) > self.timeout:
                    self.removeBlimpNodeHandler(blimpNodeHandler)
            else:  # Has received blimpID
                timeout = blimpNodeHandler.blimp.heartbeatDisconnectDelay
                if self.getElapsedTime(blimpNodeHandler.lastReceived_blimpID) > timeout:
                    self.removeBlimpNodeHandler(blimpNodeHandler)

    def createBlimpNodeHandler(self, nodeName):
        if nodeName == "_NODE_NAME_UNKNOWN_":
            self.get_logger().info("FLAG: Node Name Unknown")
            return

        self.recognizedNodes.append(nodeName)

        # Create new node handler
        newBlimpNodeHandler = NodeHandler(self, nodeName)
        topic_blimpID = "/" + nodeName + "/blimpID"
        newBlimpNodeHandler.sub_blimpID = self.create_subscription(
            String, topic_blimpID, newBlimpNodeHandler.subCallback_blimpID, 1)

        self.nodeHandlers.append(newBlimpNodeHandler)

    def removeBlimpNodeHandler(self, nodeHandler):
        self.recognizedNodes.remove(nodeHandler.nodeName)
        self.nodeHandlers.remove(nodeHandler)
        if nodeHandler is not None:
            self.get_logger().info('Detected timeout of blimp ID "%s"' % nodeHandler.blimpID)

        # Disconnect blimp
        if nodeHandler.blimp is not None:
            nodeHandler.blimp.nodeHandler = None
            nodeHandler.blimp.connected = False
            db.remove_blimp(nodeHandler.blimp)

    def getElapsedTime(self, prevTime):
        elapsedTime = self.get_clock().now() - prevTime
        elapsedTimeSec = elapsedTime.nanoseconds / 10**9
        return elapsedTimeSec

    def connectBlimp(self, blimpID, nodeHandler):
        from blimp import Blimp
        if blimpID not in db.blimps:
            # Not previously identified... identify it!
            self.numNewBlimps += 1
            blimpName = "New Blimp " + str(self.numNewBlimps)
            newBlimp = Blimp(blimpID, blimpName)
            db.blimps[blimpID] = newBlimp
            db.add_blimp(newBlimp)

            newBlimp.ID = blimpID
            newBlimp.nodeHandler = nodeHandler
            newBlimp.connected = True
            nodeHandler.blimp = newBlimp

            self.get_logger().info("Identified new blimp (id: %s). Assigned name: %s" % (str(blimpID), blimpName))
        else:
            self.get_logger().info("BlimpID already identified: %s" % blimpID)
            blimpName = "New Blimp " + str(self.numNewBlimps)
            blimp = Blimp(blimpID, blimpName)
            db.blimps[blimpID] = blimp
            db.add_blimp(blimp)

            blimp.ID = blimpID
            blimp.nodeHandler = nodeHandler
            blimp.connected = True
            nodeHandler.blimp = blimp

class NodeHandler:
    def __init__(self, parentNode, nodeName):
        self.parentNode = parentNode
        self.nodeName = nodeName
        self.timeCreated = self.parentNode.get_clock().now()

        self.blimpID = None
        self.lastReceived_blimpID = None
        self.blimp = None

        self.pub_auto = None
        self.pub_killed = None
        self.pub_motorCommands = None
        self.pub_grabbing = None
        self.pub_shooting = None
        self.pub_baseBarometer = None

    def subCallback_blimpID(self, msg):
        if self.blimpID is None:
            self.blimpID = msg.data
            self.parentNode.get_logger().info('Identified new blimp with ID "%s"' % msg.data)
            self.parentNode.connectBlimp(self.blimpID, self)
            self.createPublishers()
        self.lastReceived_blimpID = self.parentNode.get_clock().now()
        if self.blimp is not None:
            self.blimp.lastHeartbeatDetected = time.time()

    def createPublishers(self):
        topic_auto =            "/" + self.nodeName + "/auto"
        topic_killed =          "/" + self.nodeName + "/killed"
        topic_motorCommands =   "/" + self.nodeName + "/motorCommands"
        topic_grabbing =        "/" + self.nodeName + "/grabbing"
        topic_shooting =        "/" + self.nodeName + "/shooting"
        topic_baseBarometer =   "/" + self.nodeName + "/baseBarometer"

        bufferSize = 1
        self.pub_auto = self.parentNode.create_publisher(Bool, topic_auto, bufferSize)
        self.pub_killed = self.parentNode.create_publisher(Bool, topic_killed, bufferSize)
        self.pub_motorCommands = self.parentNode.create_publisher(Float64MultiArray, topic_motorCommands, bufferSize)
        self.pub_grabbing = self.parentNode.create_publisher(Bool, topic_grabbing, bufferSize)
        self.pub_shooting = self.parentNode.create_publisher(Bool, topic_shooting, bufferSize)
        self.pub_baseBarometer = self.parentNode.create_publisher(Float64, topic_baseBarometer, bufferSize)

    def publish(self):
        if self.blimp is None:
            return

        msg_auto = Bool(data=self.blimp.auto)
        msg_killed = Bool(data=self.blimp.killed)
        msg_motorCommands = Float64MultiArray(data=self.blimp.motorCommands)
        msg_grabbing = Bool(data=self.blimp.grabbing)
        msg_shooting = Bool(data=self.blimp.shooting)
        msg_baseBarometer = Float64(data=self.blimp.baseBarometer)

        self.pub_auto.publish(msg_auto)
        self.pub_killed.publish(msg_killed)
        self.pub_motorCommands.publish(msg_motorCommands)
        self.pub_grabbing.publish(msg_grabbing)
        self.pub_shooting.publish(msg_shooting)
        self.pub_baseBarometer.publish(msg_baseBarometer)

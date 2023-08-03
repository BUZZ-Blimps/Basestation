# ROS2 Packages
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Int64, Float64, Float64MultiArray
from test_msgs.srv import BasicTypes
import time

# Global Variables Package
import os
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from globalVars import *

class Basestation(Node):

    def __init__(self):
        super().__init__('Basestation')

        # Reset All Blimp Profiles
        self.reset_blimp_profiles()

        # Identify Topic for Teensy to Check if Basestation is On
        self.topicName_identify = "identify"
        
        # Number of Blimps
        self.numNewBlimps = 0

        # Recognized Blimp Nodes
        self.recognizedBlimpNodes = []

        # Blimp Node Handlers
        self.blimpNodeHandlers = []

        # Create timer
        self.loopSpeed = 0.5
        timer_period = 1.0/self.loopSpeed
        self.timer = self.create_timer(timer_period, self.timerLoop)
        self.timeout = 0.01

        """
        # Testing Subscriptions
        #self.auto_sub = self.create_subscription(Bool, '/Blimp1/auto', self.getAuto, 10)
        # Testing Services
        # self.service = self.create_service(BasicTypes, 'BlimpID', self.getService)
        # self.blimpName = None
        """

    def __del__(self):
        # Call main_node.destroy_node() to destroy the ROS 2 node
        self.destroy_node()

        print("\nDestroyed Node\n")

        # Call rclpy.shutdown() to release ROS 2 resources
        rclpy.shutdown()

    """
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
    """

    # Connect Blimp Functions #

    # Fix this function eventually when New Method is Implemented !!!
    def connectBlimp(self, blimpID, blimpNodeHandler):

        # Get Basestation's Blimp Name
        blimpName = self.getBlimpName(blimpID)

        if blimpID not in db.blimps:
            # Increase Number of New Blimps
            self.numNewBlimps += 1

            # Add Blimp
            self.addBlimp(blimpID, blimpName, blimpNodeHandler)

            # Get Filename
            filename = '../src/ros/blimps/' + blimpName.replace(' ', '') + '.txt'

            # Identify Blimp as Connected
            self.write_value(filename, 'Connected', 'True')

            # New Blimp Identified
            self.get_logger().info("Identified new blimp (id: %s). Assigned name: %s" % (str(blimpID), blimpName))

        else:
            # Re-Add Blimp
            self.addBlimp(blimpID, blimpName, blimpNodeHandler)

            # Get Filename
            filename = '../src/ros/blimps/' + blimpName.replace(' ', '') + '.txt'

            # Identify Blimp as Connected
            self.write_value(filename, 'Connected', 'True')

            # Blimp Already Identified
            self.get_logger().info("BlimpID already identified: %s" % blimpID)

    # Maybe get rid of when New Method is done?? Not sure I if still need this??
    def addBlimp(self, blimpID, blimpName, blimpNodeHandler):
        from blimp import Blimp
        blimp = Blimp(blimpID, blimpName)
        db.blimps[blimpID] = blimp
        db.add_blimp(blimp)

        blimp.ID = blimpID
        blimp.nodeHandler = blimpNodeHandler
        blimp.connected = True
        blimpNodeHandler.blimp = blimp

    def getBlimpName(self, blimpID):
        # Hard Code Blimp Names for Basestation
        if blimpID == "BurnCreamBlimp":
            blimpName = "Burn Cream Blimp"

        elif blimpID == "1":
            blimpName = "Fake Blimp"

        else:
            blimpName = "New Blimp " + str(self.numNewBlimps)

        return blimpName

    # Blimp Node Handler Functions #

    def updateBlimpNodeHandlers(self):
        # Iterate through current nodes, look for new nodes
        infos = self.get_subscriptions_info_by_topic(self.topicName_identify)
        for info in infos:
            nodeName = info.node_name
            if nodeName not in self.recognizedBlimpNodes:
                self.createBlimpNodeHandler(nodeName)

        """
        # Print out names of all connected nodes
        connectedNodes = ""
        for node in self.recognizedNodes:
            connectedNodes += node + ", "
        self.get_logger().info("Connected nodes: %s" % connectedNodes)
        """

        # Check for nodes that timed out
        for blimpNodeHandler in self.blimpNodeHandlers:

            # No Blimp ID Received
            if blimpNodeHandler.lastReceived_blimpID is None:
                # Check if Blimp Timed Out
                if self.getElapsedTime(blimpNodeHandler.timeCreated) > self.timeout:
                    self.removeBlimpNodeHandler(blimpNodeHandler)

            # Blimp ID Received
            else:
                # Get Timeout
                timeout = blimpNodeHandler.blimp.heartbeatDisconnectDelay

                # Double Check if Blimp Timed Out
                if self.getElapsedTime(blimpNodeHandler.lastReceived_blimpID) > timeout:
                    self.removeBlimpNodeHandler(blimpNodeHandler)

    def createBlimpNodeHandler(self, blimpNodeName):
        # Unknown Blimp Node Found
        if blimpNodeName == "_NODE_NAME_UNKNOWN_":
            self.get_logger().info("FLAG: Node Name Unknown")
            return

        # Add New Blimp Node to Recognized Blimp Nodes
        self.recognizedBlimpNodes.append(blimpNodeName)

        # Create New Blimp Node Handler
        newBlimpNodeHandler = BlimpNodeHandler(self, blimpNodeName)

        # Check for Blimp ID Topic
        topic_blimpID = "/" + blimpNodeName + "/blimpID"

        # Subscribe to the Blimp ID Topic
        newBlimpNodeHandler.sub_blimpID = self.create_subscription(String, topic_blimpID, newBlimpNodeHandler.subCallback_blimpID, 1)

        # Add Blimp Node Handler to List
        self.blimpNodeHandlers.append(newBlimpNodeHandler)

    # Fix this function eventually when New Method is Implemented !!!
    def removeBlimpNodeHandler(self, blimpNodeHandler):
        # Remove Blime Node Name from List
        self.recognizedBlimpNodes.remove(blimpNodeHandler.nodeName)

        # Remove Blimp Node Handler from List
        self.blimpNodeHandlers.remove(blimpNodeHandler)

        # Get Basestation's Blimp Name
        blimpName = self.getBlimpName(blimpNodeHandler.blimpID)

        # Get Filename
        filename = '../src/ros/blimps/' + blimpName.replace(' ', '') + '.txt'

        # Identify Blimp as Disconnected
        self.write_value(filename, 'Connected', 'False')

        # Timeout Detected for Blimp Node
        if blimpNodeHandler is not None:
            self.get_logger().info('Detected timeout of blimp ID "%s"' % blimpNodeHandler.blimpID)

        # Disconnect blimp
        if blimpNodeHandler.blimp is not None:
            blimpNodeHandler.blimp.nodeHandler = None
            blimpNodeHandler.blimp.connected = False
            # Change Eventually?
            db.remove_blimp(blimpNodeHandler.blimp)

    # Blimp Profile Functions #

    def read_value(self, filename, variable):
        with open(filename, 'r') as file:
            for line in file.readlines():
                if line.startswith(variable):
                    _, val = line.strip().split(' = ', 1)
                    return val
        return None  # Return None if the variable was not found in the file

    def write_value(self, filename, variable, value):
        lines = []
        with open(filename, 'r') as file:
            lines = file.readlines()

        with open(filename, 'w') as file:
            for line in lines:
                if line.startswith(variable):
                    file.write(f'{variable} = {value}\n')
                else:
                    file.write(line)

    # Can be used for the "Fleet"
    def write_value_in_all_files(self, directory, variable, value):
        for filename in os.listdir(directory):
            if filename.endswith('.txt'):  # only process text files
                self.write_value(os.path.join(directory, filename), variable, value)

    def reset_blimp_profiles(self):
        self.write_value_in_all_files('../src/ros/blimps', 'Connected', 'False')
        self.write_value_in_all_files('../src/ros/blimps', 'Autonomous', 'False')
        self.write_value_in_all_files('../src/ros/blimps', 'Stream On', 'False')

    # Timer Functions #
    
    def timerLoop(self):
        # Update Blimp Nodes
        self.updateBlimpNodeHandlers()

        # Publish Node Topics
        for blimpNodeHandler in self.blimpNodeHandlers:
            blimpNodeHandler.publish()

    def getElapsedTime(self, prevTime):
        elapsedTime = self.get_clock().now() - prevTime
        elapsedTimeSec = elapsedTime.nanoseconds / 10**9
        return elapsedTimeSec
        
class BlimpNodeHandler:
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



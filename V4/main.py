#!/usr/bin/env python3

# Flask Packages
from flask import Flask, render_template, request
from flask_socketio import SocketIO

# ROS Packages
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int64, Bool, Float64, Float64MultiArray

# Other Packages
import threading
import time

# Blimp Class
from blimp import Blimp

# Initialize Flask App and SocketIO
app = Flask(__name__)
socketio = SocketIO(app)

class Basestation(Node):

    def __init__(self):
        super().__init__('Basestation')

        # Number of Blimps
        self.numNewBlimps = 0

        # Recognized Blimp Nodes
        self.recognizedBlimpNodes = []

        # Blimp Node Handlers
        self.blimpNodeHandlers = []

        # Create timer
        self.loopSpeed = 0.1
        timer_period = 1.0/self.loopSpeed
        self.timer = self.create_timer(timer_period, self.timerLoop)
        self.timeout = 1
        
        # Identify Topic for Teensy to Check if Basestation is On
        self.topicName_identify = "identify"
    
    def connectBlimp(self, blimpNodeHandler):

        if blimpNodeHandler.nodeName not in self.recognizedBlimpNodes:
            # Increase Number of New Blimps
            self.numNewBlimps += 1

            # New Blimp Identified
            self.get_logger().info("Identified new blimp (id: %s). Node name: %s" % (str(blimpNodeHandler.blimpID), blimpNodeHandler.nodeName))

        else:
            # Blimp Already Identified
            self.get_logger().info("BlimpID already identified: %s" % blimpNodeHandler.nodeName)
    
    def updateBlimpNodeHandlers(self):
        # Testing
        print(self.recognizedBlimpNodes)

        # Iterate through current nodes, look for new nodes
        infos = self.get_subscriptions_info_by_topic(self.topicName_identify)
        for info in infos:
            nodeName = info.node_name
            # Testing
            # self.get_logger().info('Node Name: "%s"' % nodeName)
            if nodeName not in self.recognizedBlimpNodes:
                # self.get_logger().info('Node Name: "%s"' % nodeName)
                self.createBlimpNodeHandler(nodeName)

        # Check for nodes that timed out
        for blimpNodeHandler in self.blimpNodeHandlers:
            # No Blimp ID Received
            if blimpNodeHandler.lastReceived_blimpID is None:
                # Check if Blimp Timed Out
                if self.getElapsedTime(blimpNodeHandler.timeCreated) > self.timeout:
                    self.removeBlimpNodeHandler(blimpNodeHandler)
            # Blimp ID Received
            else:
                # Double Check if Blimp Timed Out
                if self.getElapsedTime(blimpNodeHandler.lastReceived_blimpID) > self.timeout:
                    self.removeBlimpNodeHandler(blimpNodeHandler)

    def createBlimpNodeHandler(self, blimp_node_name):
        # Unknown Blimp Node Found
        if blimp_node_name == "_NODE_NAME_UNKNOWN_":
            self.get_logger().info("FLAG: Node Name Unknown")
            return

        # Add New Blimp Node to Recognized Blimp Nodes
        self.recognizedBlimpNodes.append(blimp_node_name)

        # Create New Blimp Node Handler
        newBlimpNodeHandler = BlimpNodeHandler(self, blimp_node_name)

        # Check for Blimp ID Topic
        topic_blimpID = "/" + blimp_node_name + "/blimpID"

        # Subscribe to the Blimp ID Topic
        newBlimpNodeHandler.sub_blimpID = self.create_subscription(String, topic_blimpID, newBlimpNodeHandler.listener_callback, 10)

        # Add Blimp Node Handler to List
        self.blimpNodeHandlers.append(newBlimpNodeHandler)

    # Fix this function eventually when New Method is Implemented !!!
    def removeBlimpNodeHandler(self, blimpNodeHandler):
        # Remove Blime Node Name from List
        self.recognizedBlimpNodes.remove(blimpNodeHandler.nodeName)

        # Remove Blimp Node Handler from List
        self.blimpNodeHandlers.remove(blimpNodeHandler)

        # Timeout Detected for Blimp Node
        if blimpNodeHandler is not None:
            self.get_logger().info('Detected timeout of blimp ID "%s"' % blimpNodeHandler.blimpID)

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
        self.blimp_name = None
        self.goal_color = 0

        self.pub_auto = None
        self.pub_goal_color = self.goal_color
        self.pub_killed = None
        self.pub_motor_commands = None
        self.pub_grabbing = None
        self.pub_shooting = None
        # self.pub_base_barometer = None

    def listener_callback(self, msg):
        # Check blimp ID and give it a name on the Basestation
        # Make the following if statement a function !!!
        global blimps
        if self.blimpID is None:
            self.blimpID = msg.data
            self.parentNode.get_logger().info('Identified new blimp with ID "%s"' % msg.data)
            self.parentNode.connectBlimp(self)
            self.blimp_name = self.get_blimp_name()
            blimp = Blimp(self.blimp_name)
            blimps[self.blimp_name] = blimp
            self.createPublishers()

        self.lastReceived_blimpID = self.parentNode.get_clock().now()
        
        # Do we still need a heartbeat ???
        # if blimp is not None:
            # blimp.lastHeartbeatDetected = time.time()

        # Emit the blimp data to the webpage
        socketio.emit('update', blimps[self.blimp_name].to_dict())
    
        # Publish the goal color value to ROS
        # Make this independent of the listener callback !!!
        self.publish_goal_color()

    def get_blimp_name(self):
        self.blimp_name = 'Error'

        if self.blimpID == 'WaffleBlimp':
            self.blimp_name = 'Waffle Blimp'
        elif self.blimpID == 'BurnCreamBlimp':
            self.blimp_name = 'Burn Cream Blimp'
        elif self.blimpID == 'Blimp2':
            self.blimp_name = 'Blimp 2'
        elif self.blimpID == 'Blimp1':
            self.blimp_name = 'Blimp 1'

        return self.blimp_name

    def publish_goal_color(self):
        global blimps
        # Publish goal color value to the ROS topic
        msg = Int64()
        msg.data = blimps[self.blimp_name].goal_color
        self.pub_goal_color.publish(msg)

    # Update Goal Color
    @socketio.on('update_blimp_dict')
    def update_blimp_dict(data):
        global blimps
        blimp_name = data['blimp_name']
        blimps[blimp_name].update_dict(data)

    # Update Goal Color
    @socketio.on('update_goal_color')
    def update_goal_color(data):
        global blimps
        blimp_name = data['blimp_name']
        blimps[blimp_name].goal_color = data['goal_color']

        # Testing
        # goal_color = data['goal_color']
        # print(goal_color)

    def createPublishers(self):
        topic_auto =            "/" + self.nodeName + "/auto"
        topic_goal_color =      "/" + self.nodeName + "/goal_color"
        topic_killed =          "/" + self.nodeName + "/killed"
        topic_motor_commands =  "/" + self.nodeName + "/motorCommands"
        topic_grabbing =        "/" + self.nodeName + "/grabbing"
        topic_shooting =        "/" + self.nodeName + "/shooting"
        # topic_base_barometer =   "/" + self.nodeName + "/base_barometer"

        bufferSize = 1
        self.pub_auto = self.parentNode.create_publisher(Bool, topic_auto, bufferSize)
        self.pub_goal_color = self.parentNode.create_publisher(Int64, topic_goal_color, bufferSize)
        self.pub_killed = self.parentNode.create_publisher(Bool, topic_killed, bufferSize)
        self.pub_motor_commands = self.parentNode.create_publisher(Float64MultiArray, topic_motor_commands, bufferSize)
        self.pub_grabbing = self.parentNode.create_publisher(Bool, topic_grabbing, bufferSize)
        self.pub_shooting = self.parentNode.create_publisher(Bool, topic_shooting, bufferSize)
        # self.pub_base_barometer = self.parentNode.create_publisher(Float64, topic_base_barometer, bufferSize)

    def publish(self):
        pass
        """
        if self.blimp is None:
            return
        
        msg_auto = Bool(data=self.blimp.auto)
        msg_goal_color = Bool(data=self.blimp.auto)
        msg_killed = Bool(data=self.blimp.killed)
        msg_motor_commands = Float64MultiArray(data=self.blimp.motor_commands)
        msg_grabbing = Bool(data=self.blimp.grabbing)
        msg_shooting = Bool(data=self.blimp.shooting)
        msg_baseBarometer = Float64(data=self.blimp.baseBarometer)

        self.pub_auto.publish(msg_auto)
        self.pub_goal_color.publish(msg_goal_color)
        self.pub_killed.publish(msg_killed)
        self.pub_motor_commands.publish(msg_motor_commands)
        self.pub_grabbing.publish(msg_grabbing)
        self.pub_shooting.publish(msg_shooting)
        self.pub_baseBarometer.publish(msg_baseBarometer)
        """

# Handle user connection to webpage
@socketio.on('connect')
def handle_connect():
    print('Client connected with IP:', request.remote_addr)

@app.route('/')
def index():
    client_ip = request.remote_addr
    return render_template('main.html', client_ip=client_ip)

def ros_thread():
    rclpy.init()

    node = Basestation()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    # Create init function for the following values
    # Initialize default value i.e. goal color value (default: 0)
    # Could make these read from a text file to make them permanent profiles
    global blimps
    blimps = {}

    ros_thread = threading.Thread(target=ros_thread)
    ros_thread.start()

    socketio.run(app, host='0.0.0.0', port=5000)


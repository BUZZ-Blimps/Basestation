#!/usr/bin/env python3

# Flask Packages
from flask import Flask, render_template, request, Response
from flask_socketio import SocketIO

# ROS Packages
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String, Int64, Bool, Float64, Float64MultiArray
from yolo_msgs.msg import BoundingBox # type: ignore

# Livestream Packages
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

# Other Packages
import threading
import time
import sys
import signal
import os
import serial
import numpy as np
import subprocess
import json # Currently not used (Potential future use)

# Blimp Class
from blimp import Blimp

# Initialize Flask App and SocketIO
app = Flask(__name__)
socketio = SocketIO(app)

# Get the PID of the current process
current_pid = os.getpid()

class Basestation(Node):
    def __init__(self):
        super().__init__('Basestation')

        # Start up Basestation
        socketio.emit('start')

        # Number of Blimps
        self.num_blimps = 0

        # Connected Blimp Nodes
        self.connected_blimps = []

        # Blimp Node Handlers
        self.blimp_node_handlers = []

        # All Blimps Dictionary
        self.all_blimps = {}

        # Create timer
        self.loopSpeed = 250 # Depends on CPU Speed (Future To-Do: Optimize Frontend/UI to run main loop faster)
        timer_period = 1.0/self.loopSpeed
        self.timer = self.create_timer(timer_period, self.timerLoop)
        self.timeout = 5

        # Create Barometer Timer
        self.barometer = None
        self.barometerLoopSpeed = 100
        barometer_timer_period = 1.0/self.barometerLoopSpeed
        self.barometerTimer = self.create_timer(barometer_timer_period, self.barometerTimerLoop)
        
        # Identify Topic for Teensy to Check if Basestation is On
        self.topicName_identify = "/identify"
        self.identify_sub = self.create_subscription(String, self.topicName_identify, self.identify_callback, 10)

        self.catching_blimp_ids = ['BurnCreamBlimp', 'SillyAhBlimp', 'TurboBlimp', 'GameChamberBlimp', 'FiveGuysBlimp', 'SuperBeefBlimp', 'Catch1', 'Catch2']
        self.attack_blimp_ids = ['Yoshi', 'Luigi', 'Geoph', 'ThisGuy', 'Attack1', 'Attack2']

    def identify_callback(self, msg):
        global blimps

        # Debugging
        #print(msg.data)

        # Identify message is just a string with the blimp ID
        blimp_id = msg.data
        if blimp_id in self.connected_blimps:
            # Update last online here because update is only called while the node is subscribed
            blimps[blimp_id].last_online = self.get_clock().now()
        else:
            # Otherwise, check its validity and create a handler for it

            # Debugging
            # self.get_logger().info('Node Name: "%s"' % nodeName)

            # Make sure node name is valid
            if self.check_node_name(blimp_id) == True:
                print("ADDING BLIMP")
                self.create_blimp_node_handler(blimp_id)
                blimps[blimp_id].last_online = self.get_clock().now()

    def update_blimp_node_handlers(self):
        global blimps

        # Finally, check for subscription timeout for all connected blimps
        for blimp_node_handler in self.blimp_node_handlers:
            if self.getElapsedTime(blimps[blimp_node_handler.blimp_id].last_online) > self.timeout:
                print("REMOVING BLIMP")
                self.remove_blimp_node_handler(blimp_node_handler.blimp_id)
            else:
                # Run handler update routine
                blimp_node_handler.update()

        # Get All Current Blimps
        for blimp in blimps:
            self.all_blimps[blimp] = blimps[blimp].to_dict()

        # Emit Barometer Data
        socketio.emit('barometer', self.all_blimps)
    
    def create_blimp_node_handler(self, blimp_id):
        
        # Create New Blimp Node Handler
        new_blimp_node_handler = BlimpNodeHandler(self, blimp_id)

        # State Machine Topic
        topic_state_machine = "/" + blimp_id + "/state_machine"

        # Subscribe to the State Machine Topic
        new_blimp_node_handler.sub_state_machine = self.create_subscription(Int64, topic_state_machine, new_blimp_node_handler.state_machine_callback, 10)
        
        # Check for Log Topic
        topic_logs = "/" + blimp_id + "/log"

        # Subscribe to the Log Topic
        new_blimp_node_handler.sub_logs = self.create_subscription(String, topic_logs, new_blimp_node_handler.logs_callback, 10)
        
        # Check for Base Barometer Topic
        topic_baseBarometer = "/" + blimp_id + "/baseBarometer"

        # Check for Base Barometer Topic
        topic_height = "/" + blimp_id + "/height"

        # Subscribe to the State Machine Topic
        new_blimp_node_handler.sub_height = self.create_subscription(Float64, topic_height, new_blimp_node_handler.height_callback, 10)

        # Check for Base Barometer Topic
        topic_z_velocity = "/" + blimp_id + "/z_velocity"

        # Subscribe to the State Machine Topic
        new_blimp_node_handler.sub_z_velocity = self.create_subscription(Float64, topic_z_velocity, new_blimp_node_handler.z_velocity_callback, 10)

        # Check for Base Barometer Topic
        # self.connectBlimp(new_blimp_node_handler)
        new_blimp_node_handler.connect_blimp()

        # Add New Blimp Node to Connected Blimp Nodes
        self.connected_blimps.append(new_blimp_node_handler.blimp_id)

        # Add Blimp Node Handler to List
        self.blimp_node_handlers.append(new_blimp_node_handler)

    def remove_blimp_node_handler(self, blimp_id):
        global blimps

        # Timeout Detected for Blimp Node
        if blimp_id is not None:
            self.get_logger().info('Detected timeout of blimp ID "%s"' % blimp_id)

            # First, get the handle of the node handler
            handler_found = False
            handler_id = None
            for id, handler in enumerate(self.blimp_node_handlers):
                if handler.blimp_id == blimp_id:
                    handler_found = True
                    handler_id = id

            if not handler_found:
                self.get_logger().error('Blimp ID "%s" Handler not found!' % blimp_id)
                return
            
            # Now, gracefully shut down the node handler
            # Destroy subscribers (publishers will timeout anyways)
            self.blimp_node_handlers[handler_id].destroy_subscribers()
            self.blimp_node_handlers[handler_id].destroy_publishers()

            # Remove blimp from list of connected blimps
            self.connected_blimps.remove(blimp_id)

            # Finally, delete the handler object for good
            del self.blimp_node_handlers[handler_id]

            # Delete from global blimps dict
            del blimps[blimp_id]

            self.num_blimps -= 1

            # Remove blimp from frontend display
            socketio.emit('remove', blimp_id)

    # Check if Node Name is valid
    def check_node_name(self, blimp_id):
        if blimp_id in self.catching_blimp_ids or blimp_id in self.attack_blimp_ids:
            return True
        else:
            return False

    # Timer Functions #
    
    def timerLoop(self):
        # Update Blimp Nodes
        self.update_blimp_node_handlers()

    def barometerTimerLoop(self):
        # Renitialize Barometer
        if self.barometer == None:
            try:
                self.barometer = serial.Serial('/dev/ttyACM0', 115200)
                print('BAROMETER CONNECTED')
            except:
                print('BAROMETER NOT CONNECTED')

        for blimp_node_handler in self.blimp_node_handlers:
            blimp_node_handler.publish_calibrateBarometer()

        global blimps
        if self.barometer is not None:
            try:
                #Read serial data if available
                if self.barometer.in_waiting:
                    # Read a line of data from the serial port
                    data = self.barometer.readline().decode('utf-8')

                    # Debugging
                    #print(data)  # Assuming data is encoded as UTF-8

                    for blimp in blimps:
                        blimps[blimp].barometer = float(data)

            except:
                self.parent_node.barometer = None

    def getElapsedTime(self, prevTime):
        elapsedTime = self.get_clock().now() - prevTime
        elapsedTimeSec = elapsedTime.nanoseconds / 10**9
        return elapsedTimeSec

class BlimpNodeHandler:
    def __init__(self, parent_node, node_name):
        self.parent_node = parent_node
        self.blimp_id = node_name
        self.blimp_name = self.get_blimp_name(self.blimp_id)
        self.time_created = self.parent_node.get_clock().now()
        
        # Identified Count
        self.identified_count = 0

        # QoS profile for boolean latches
        self.boolean_qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # Create publishers now
        self.create_publishers()

        # Initialize all subscribers to None; parent will handle their creation
        self.sub_state_machine = None
        self.sub_image_raw = None
        self.sub_bounding_box = None
        self.sub_baseBarometer = None
        self.sub_height= None
        self.sub_z_velocity = None
        self.sub_logs = None

        self.catching_motor_timer_hz = 10
        self.attack_motor_timer_hz = 10
        self.baro_timer_hz = 100
        self.attack_data_timer_hz = 1

        # Initialize inner timer loops
        now = parent_node.get_clock().now()
        self.motor_timer = now
        self.baro_timer = now
        self.attack_data_timer = now
        
        # Livestream Most Recent Frame
        self.frame = None
        self.bridge = CvBridge()

        self.blimp_dict = {
            "blimp_id": None,
            "blimp_name": None,
            "blimp_type": None,
            "auto": None,
            "killed": None,
            "goal_color": None,
            "target_color": None,
            "state_machine": None,
            "connected": None,
            "barometer": None,
            "calibrateBarometer": None,
            "height": None,
            "z_velocity": None,
            "log": None
        }

    def update_handler_dict(self):
        global blimps
        global_blimp = blimps[self.blimp_id]
        dict_updated = False

        if global_blimp.blimp_id != self.blimp_dict["blimp_id"]:
            # print('blimp_id updating')

            dict_updated = True
            self.blimp_dict["blimp_id"] = global_blimp.blimp_id

        if global_blimp.blimp_name != self.blimp_dict["blimp_name"]:
            # print('blimp_name updating')

            dict_updated = True
            self.blimp_dict["blimp_name"] = global_blimp.blimp_name

        if global_blimp.blimp_type != self.blimp_dict["blimp_type"]:
            # print('blimp_type updating')

            dict_updated = True
            self.blimp_dict["blimp_type"] = global_blimp.blimp_type

        if global_blimp.auto != self.blimp_dict["auto"]:
            # print('auto updating')

            dict_updated = True
            self.blimp_dict["auto"] = global_blimp.auto

        if global_blimp.killed != self.blimp_dict["killed"]:
            # print('killed updating')

            dict_updated = True
            self.blimp_dict["killed"] = global_blimp.killed

        if global_blimp.goal_color != self.blimp_dict["goal_color"]:
            # print('goal_color updating')

            dict_updated = True
            self.blimp_dict["goal_color"] = global_blimp.goal_color

        if global_blimp.target_color != self.blimp_dict["target_color"]:
            # print('target_color updating')

            dict_updated = True
            self.blimp_dict["target_color"] = global_blimp.target_color

        if global_blimp.state_machine != self.blimp_dict["state_machine"]:
            # print('state_machine updating')

            dict_updated = True
            self.blimp_dict["state_machine"] = global_blimp.state_machine

        if global_blimp.connected != self.blimp_dict["connected"]:
            # print('connected updating')

            dict_updated = True
            self.blimp_dict["connected"] = global_blimp.connected

        # if global_blimp.barometer != self.blimp_dict["barometer"]:
        #     # print('barometer updating')

        #     #dict_updated = True
        #     self.blimp_dict["barometer"] = global_blimp.barometer

        # if global_blimp.calibrateBarometer != self.blimp_dict["calibrateBarometer"]:
        #     # print('calibrateBarometer updating')

        #     dict_updated = True
        #     self.blimp_dict["calibrateBarometer"] = global_blimp.calibrateBarometer

        # if global_blimp.height != self.blimp_dict["height"]:
        #     # print('height updating')

        #     #dict_updated = True
        #     self.blimp_dict["height"] = global_blimp.height

        # if global_blimp.z_velocity != self.blimp_dict["z_velocity"]:
        #     print('z_velocity updating')

        #     dict_updated = True
        #     self.blimp_dict["z_velocity"] = global_blimp.z_velocity

        if global_blimp.log != self.blimp_dict["log"]:
            # print('log updating')

            dict_updated = True
            self.blimp_dict["log"] = global_blimp.log

        #If the blimp dictionary was updated, emit it to the frontend
        if dict_updated:
            # print('emitting update')

            # Emit the blimp data to the webpage
            socketio.emit('update', blimps[self.blimp_id].to_dict())

    def connect_blimp(self):
        global blimps
        self.parent_node.get_logger().info('Identified Blimp with ID "%s"' % str(self.blimp_id))
        self.parent_node.num_blimps += 1

        # Future Todo: Fix redundancies between Blimp objects and handlers (they're one-to-one)
        # Create a Blimp object for storing the blimp state
        blimp = Blimp(self.blimp_id)
        blimp.blimp_name = self.blimp_name

        # Set the blimp type
        self.blimp_type = self.get_blimp_type(blimp)

        if self.blimp_type == True:
            # Attack blimp
            self.motor_timer_period = 1/self.attack_motor_timer_hz
            self.data_timer_period = 1/self.attack_data_timer_hz
        else:
            # Catching blimp
            self.motor_timer_period = 1/self.catching_motor_timer_hz
            self.baro_timer_period = 1/self.baro_timer_hz
    
        # Update the global blimp dictionary with this blimp
        blimps[self.blimp_id] = blimp

    def update(self):

        # Debugging
        # self.parent_node.get_logger().info("Updating")

        #Update the blimp dictionary if needed
        self.update_handler_dict()

        global blimps
        if self.blimp_id in blimps:

            now = self.parent_node.get_clock().now()

            if self.parent_node.getElapsedTime(self.motor_timer) >= self.motor_timer_period:
                try:
                    self.publish_motorCommands()
                    self.motor_timer = now
                except: 
                    pass

            # Differentiate catching and attack blimps
            if (blimps[self.blimp_id].blimp_type == False):
                # Catching-blimp specific
                if blimps[self.blimp_id].update_grabbing_pub:
                    try:
                        self.publish_grabbing()
                        blimps[self.blimp_id].update_grabbing_pub = False
                    except:
                        pass

                if blimps[self.blimp_id].update_shooting_pub:
                    try:
                        self.publish_shooting()
                        blimps[self.blimp_id].update_shooting_pub = False
                    except:
                        pass

                if blimps[self.blimp_id].update_auto_pub:
                    try:
                        self.publish_auto()
                        blimps[self.blimp_id].update_auto_pub = False
                    except:
                        pass

                if blimps[self.blimp_id].update_goal_color_pub:
                    try:
                        self.publish_goal_color()
                        blimps[self.blimp_id].update_goal_color_pub = False
                    except:
                        pass

                if blimps[self.blimp_id].update_target_color_pub:
                    try:
                        self.publish_target_color()
                        blimps[self.blimp_id].update_target_color_pub = False
                    except:
                        pass

                # Publish barometer to catching blimps on a timer
                if self.parent_node.getElapsedTime(self.baro_timer) >= self.baro_timer_period:
                    try:
                        self.publish_barometer()
                        self.baro_timer = now
                    except:
                        pass
                
                try:
                    self.update_image_subscriber()
                except:
                    pass
            else:
                # Attack blimp specific
                if self.parent_node.getElapsedTime(self.attack_data_timer) >= self.data_timer_period:
                    try:
                        self.publish_auto()
                        self.publish_target_color()
                        self.attack_data_timer = now
                    except:
                        pass

    def create_publishers(self):
        topic_auto =            "/" + self.blimp_id + "/auto"
        topic_goal_color =      "/" + self.blimp_id + "/goal_color"
        topic_target_color =      "/" + self.blimp_id + "/target_color"
        topic_killed =          "/" + self.blimp_id + "/killed"
        topic_motorCommands =  "/" + self.blimp_id + "/motorCommands"
        topic_grabbing =        "/" + self.blimp_id + "/grabbing"
        topic_shooting =        "/" + self.blimp_id + "/shooting"
        topic_baseBarometer =   "/" + self.blimp_id + "/baseBarometer"
        topic_calibrateBarometer =   "/" + self.blimp_id + "/calibrateBarometer"

        bufferSize = 1
        self.pub_auto = self.parent_node.create_publisher(Bool, topic_auto, self.boolean_qos_profile)
        self.pub_goal_color = self.parent_node.create_publisher(Int64, topic_goal_color, self.boolean_qos_profile)
        self.pub_target_color = self.parent_node.create_publisher(Int64, topic_target_color, self.boolean_qos_profile)
        self.pub_killed = self.parent_node.create_publisher(Bool, topic_killed, self.boolean_qos_profile)
        self.pub_motorCommands = self.parent_node.create_publisher(Float64MultiArray, topic_motorCommands, bufferSize)
        self.pub_grabbing = self.parent_node.create_publisher(Bool, topic_grabbing, self.boolean_qos_profile)
        self.pub_shooting = self.parent_node.create_publisher(Bool, topic_shooting, self.boolean_qos_profile)
        self.pub_baseBarometer = self.parent_node.create_publisher(Float64, topic_baseBarometer, bufferSize)
        self.pub_calibrateBarometer = self.parent_node.create_publisher(Bool, topic_calibrateBarometer, self.boolean_qos_profile)

    def destroy_publishers(self):
        self.pub_auto.destroy()
        self.pub_goal_color.destroy()
        self.pub_target_color.destroy()
        self.pub_killed.destroy()
        self.pub_motorCommands.destroy()
        self.pub_grabbing.destroy()
        self.pub_shooting.destroy()
        self.pub_baseBarometer.destroy()
        self.pub_calibrateBarometer.destroy()

    # Function to destroy subscribers for elegant behavior
    def destroy_subscribers(self):
        global blimps
        self.parent_node.destroy_subscription(self.sub_state_machine)
        self.parent_node.destroy_subscription(self.sub_logs)
        self.parent_node.destroy_subscription(self.sub_height)
        self.parent_node.destroy_subscription(self.sub_z_velocity)
        if self.sub_image_raw is not None:
            self.parent_node.destroy_subscription(self.sub_image_raw)

        if self.sub_bounding_box is not None:
            self.parent_node.destroy_subscription(self.sub_bounding_box)

    # Continually Poll State Machine Data from Teensy
    def state_machine_callback(self, msg):
        global blimps
        if self.blimp_id in blimps:
            blimps[self.blimp_id].state_machine = msg.data

    # Continually Poll State Machine Data from Teensy
    def logs_callback(self, msg):
        global blimps
        blimps[self.blimp_id].log = msg.data
        if self.blimp_id in blimps:
            # Emit the blimp's logs to the webpage
            socketio.emit('logs', blimps[self.blimp_id].to_dict())

    # Continually Poll Image Raw Data from Pi
    def image_raw_callback(self, msg):
        global blimps
        if self.blimp_id in blimps:
            try:
                # Convert the ROS Image message to a CV2 image (numpy ndarray)
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                self.frame = cv_image

                if self.frame is not None:
                    flag, jpeg = cv2.imencode('.jpg', self.frame)
                    blimps[self.blimp_id].frame = jpeg
                
                # Debugging
                # Uncomment the following line if you want to see the image using OpenCV
                # cv2.imshow("Received Image", cv_image)
                # cv2.waitKey(1)

            except Exception as e:
                self.parent_node.get_logger().error(f"Failed to convert image: {e}")

    # Continually Poll State Machine Data from Teensy
    def bounding_box_callback(self, msg):
        global blimps
        bb_dict = self.bounding_box_to_dict(msg)
        if self.blimp_id in blimps:
            if bb_dict is not None:
                blimps[self.blimp_id].bounding_box = bb_dict
                # Emit the bounding box data to the webpage
                socketio.emit('bounding_box', blimps[self.blimp_id].bounding_box)

    # Continually Poll Height Data from Teensy
    def height_callback(self, msg):
        global blimps
        if self.blimp_id in blimps:
            if msg.data is not None:
                blimps[self.blimp_id].height = msg.data

    # Continually Poll Z Velocity Data from Teensy
    def z_velocity_callback(self, msg):
        global blimps
        if self.blimp_id in blimps:
            if msg.data is not None:
                blimps[self.blimp_id].z_velocity = msg.data

    def update_image_subscriber(self):
        global blimps
        if blimps[self.blimp_id].show_image is True and self.sub_image_raw is None:

            # Debugging
            # print('Creating Image Subscriber')

            # Image Raw Topic
            topic_image_raw = "/" + self.blimp_id + "/left/image_raw"

            # Image Raw Topic QOS Profile
            qos_profile = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE
            )

            # Subscribe to the Image Raw Topic
            self.sub_image_raw = self.parent_node.create_subscription(Image, topic_image_raw, self.image_raw_callback, qos_profile)

            # Check for Bounding Box Topic
            topic_bounding_box = "/" + self.blimp_id + "/bounding_box"

            # Subscribe to the Bounding Box Topic
            self.sub_bounding_box = self.parent_node.create_subscription(BoundingBox, topic_bounding_box, self.bounding_box_callback, qos_profile)
        elif blimps[self.blimp_id].show_image is False and self.sub_image_raw is not None:

            # Debugging
            # print('Destroying Image Subscriber')

            try:
                self.parent_node.destroy_subscription(self.sub_image_raw)
                self.sub_image_raw = None
            except:
                pass
            try:
                self.parent_node.destroy_subscription(self.sub_bounding_box)
                self.sub_bounding_box = None
            except:
                pass

    def bounding_box_to_dict(self, bb_msg):
        """
        Convert BoundingBox message to Python dictionary.

        :param bb_msg: BoundingBox message instance.
        :type bb_msg: yolo_msgs.msg.BoundingBox
        :return: Dictionary representation of the message.
        :rtype: dict
        """
        return {
            "header": {
                "stamp": {
                    "sec": bb_msg.header.stamp.sec,
                    "nanosec": bb_msg.header.stamp.nanosec
                },
                "frame_id": bb_msg.header.frame_id
            },
            "balloon": {
                "x_center": bb_msg.x_center_balloon,
                "y_center": bb_msg.y_center_balloon,
                "width": bb_msg.width_balloon,
                "height": bb_msg.height_balloon
            },
            "y_goal": {
                "x_center": bb_msg.x_center_y_goal,
                "y_center": bb_msg.y_center_y_goal,
                "width": bb_msg.width_y_goal,
                "height": bb_msg.height_y_goal
            },
            "o_goal": {
                "x_center": bb_msg.x_center_o_goal,
                "y_center": bb_msg.y_center_o_goal,
                "width": bb_msg.width_o_goal,
                "height": bb_msg.height_o_goal
            }
        }

    # Used for Attack Blimps Only
    def get_blimp_type(self, blimp):
        # Only need to check for Attack Blimps since default is catching type (0)
        if self.blimp_id in self.parent_node.attack_blimp_ids:
            blimp.blimp_type = 1
        
        return blimp.blimp_type

    def get_blimp_name(self, blimp_id):
        blimp_names_by_id = {
            # Catching Blimps #
            'BurnCreamBlimp': 'Burn Cream Blimp',
            'SillyAhBlimp': 'Silly Ah Blimp',
            'TurboBlimp': 'Turbo Blimp',
            'GameChamberBlimp': 'Game Chamber Blimp',
            'FiveGuysBlimp': 'Five Guys Blimp',
	        'SuperBeefBlimp': 'Super Beef Blimp',
            # Attacking Blimps #
            'Yoshi': 'Yoshi',
            'Geoph': 'Geoph',
	        'ThisGuy': 'This Guy',
            'Luigi': 'Luigi',
            # Fake Blimps #
            'Catch1': 'Catch 1',
            'Catch2': 'Catch 2',
            'Attack1': 'Attack 1',
            'Attack2': 'Attack 2'
        }

        return blimp_names_by_id[blimp_id]

    def publish_target_color(self):
        global blimps
        # Publish goal color value to the ROS topic
        msg = Int64()
        msg.data = blimps[self.blimp_id].target_color
        self.pub_target_color.publish(msg)

    def publish_goal_color(self):
        global blimps
        # Publish goal color value to the ROS topic
        msg = Int64()
        msg.data = blimps[self.blimp_id].goal_color
        self.pub_goal_color.publish(msg)

    def publish_motorCommands(self):
        global blimps
        # Publish motor commands value to the ROS topic
        msg = Float64MultiArray()
        msg.data = blimps[self.blimp_id].motorCommands
        self.pub_motorCommands.publish(msg)

    def publish_auto(self):
        global blimps
        # Publish auto value to the ROS topic
        msg = Bool()
        msg.data = blimps[self.blimp_id].auto
        self.pub_auto.publish(msg)

    def publish_grabbing(self):
        global blimps
        # Publish grabbing value to the ROS topic
        msg = Bool()
        msg.data = blimps[self.blimp_id].grabbing
        self.pub_grabbing.publish(msg)

    def publish_shooting(self):
        global blimps
        # Publish shooting value to the ROS topic
        msg = Bool()
        msg.data = blimps[self.blimp_id].shooting
        self.pub_shooting.publish(msg)

    def publish_barometer(self):
        global blimps
        # Publish barometer value to the ROS topic
        msg = Float64()
        msg.data = blimps[self.blimp_id].barometer
        self.pub_baseBarometer.publish(msg)

    def publish_calibrateBarometer(self):
        global blimps
        # Publish shooting value to the ROS topic
        msg = Bool()
        msg.data = blimps[self.blimp_id].calibrateBarometer
        self.pub_calibrateBarometer.publish(msg)
        if blimps[self.blimp_id].calibrateBarometer == True:
            blimps[self.blimp_id].calibrateBarometer = False

    # Update Total Disconnection
    @socketio.on('kill_basestation')
    def kill_basestation():
        print('\nDestroying Basestation Node...\n')
        global node
        try:
            node.destroy_node()
        except rclpy.handle.InvalidHandle as e:
            pass
        try:
            rclpy.shutdown()
        except Exception as e:
            pass
        socketio.emit("kill")
        time.sleep(0.5)
        for blimp in blimps:
            try:
                blimps[blimp].parent_node.barometer.close()
            except:
                pass
        print('\nTerminating Program...\n')
        os.kill(current_pid, signal.SIGTERM)

    # Update Blimp Class with Dictionary Data
    @socketio.on('update_blimp_dict')
    def update_blimp_dict(data):
        global blimps
        blimp_id = data['blimp_id']
        if blimp_id in blimps:
            blimps[blimp_id] = data[blimp_id]

    # Update Connection
    @socketio.on('update_connection')
    def update_connection(data):
        global blimps
        if data in blimps:
            blimps[data].connected = True

    # Update Disconnection
    @socketio.on('update_disconnection')
    def update_disconnection(data):
        global blimps
        if data in blimps:
            blimps[data].connected = False

    # Update Total Disconnection
    @socketio.on('update_total_disconnection')
    def update_total_disconnection():
        global blimps
        for blimp in blimps:
            blimps[blimp].connected = False

    # Subscribe from Image
    @socketio.on('show_image')
    def show_image(data):
        global blimps
        if data in blimps:
            blimps[data].show_image = True

    # Destroy Image Subscriber
    @socketio.on('remove_image')
    def remove_image(data):
        global blimps
        if data in blimps:
            blimps[data].show_image = False

    # Update Motor Commands
    @socketio.on('update_motorCommands')
    def update_motorCommands(data):
        array = np.frombuffer(data, dtype=np.float64)
        motorCommands = array.tolist()

        # Debugging
        #print('\nReceived Data:', motorCommands + '\n')

        # Iterate through which blimp_name is connected
        global blimps
        for blimp in blimps:
            if blimps[blimp].connected == True:

                # Debugging
                #print(blimps[blimp].blimp_name + "connected")

                blimps[blimp].motorCommands = motorCommands

                # Debugging
                #print(blimps[blimp].motorCommands)

            else:

                # Debugging
                #print(blimps[blimp].blimp_name + "not connected")

                blimps[blimp].motorCommands = [0.0, -0.0, 0.0, -0.0]

                # Debugging
                #print(blimps[blimp].motorCommands)

    # Update All Goal Colors
    @socketio.on('update_all_goal_colors')
    def update_goal_colors():
        global blimps
        global all_goal_color

        all_goal_color = not all_goal_color

        for blimp in blimps:
            blimps[blimp].goal_color = 1 if all_goal_color else 0
            # Set update flag
            blimps[blimp].update_goal_color_pub = True

    # Update Grabbing
    @socketio.on('update_grabbing')
    def update_grabbing(data):
        global blimps
        if data in blimps:
            blimps[data].grabbing = not blimps[data].grabbing
            # Set update flag
            blimps[data].update_grabbing_pub = True

    # Update Shooting
    @socketio.on('update_shooting')
    def update_shooting(data):
        global blimps
        if data in blimps:
            blimps[data].shooting = not blimps[data].shooting
            # Set update flag
            blimps[data].update_shooting_pub = True

    # Update Shooting
    @socketio.on('update_auto')
    def update_auto(data):
        global blimps
        if data in blimps:
            blimps[data].auto = not blimps[data].auto
            # Set update flag
            blimps[data].update_auto_pub = True

    # Update Autonomous Mode
    @socketio.on('update_auto_panic')
    def update_auto_panic():
        global blimps
        global auto_panic

        auto_panic = not auto_panic
        
        for blimp in blimps:
            blimps[blimp].auto = auto_panic
            # Set update flag
            blimps[blimp].update_auto_pub = True
        
    # Update All Target Colors
    @socketio.on('update_all_target_colors')
    def update_target_colors():
        global blimps
        global all_target_color

        all_target_color = not all_target_color

        for blimp in blimps:
            blimps[blimp].target_color = 1 if all_target_color else 0
            # Set update flag
            blimps[blimp].update_target_color_pub = True

    # Update Target Color
    @socketio.on('update_target_color')
    def update_target_color(data):
        global blimps
        blimp_id = data['blimp_id']
        if blimp_id in blimps:
            blimps[blimp_id].target_color = data['target_color']

            # Set update flag
            blimps[blimp_id].update_target_color_pub = True

    # Update Goal Color
    @socketio.on('update_goal_color')
    def update_goal_color(data):
        global blimps
        blimp_id = data['blimp_id']
        if blimp_id in blimps:
            blimps[blimp_id].goal_color = data['goal_color']

            # Set update flag
            blimps[blimp_id].update_goal_color_pub = True

    # Calibrate Barometer
    @socketio.on('calibrate_barometer')
    def calibrate_barometer(data):
        global blimps
        if data in blimps:
            blimps[data].calibrateBarometer = True

# Handle user connection to webpage
@socketio.on('connect')
def handle_connect():
    print('Client connected with IP:', request.remote_addr)

@socketio.on('update_frontend')
def update_frontend():
    global blimps
    print('emitting blimp update')
    for blimp in blimps:
        socketio.emit('update', blimps[blimp].to_dict())

# Main Basestation Page
@app.route('/')
def index():
    client_ip = request.remote_addr
    return render_template('main.html', client_ip=client_ip)

# Streaming Feeds/Pages
def generate(feed_name):
    global blimps
    while True:
        if feed_name in blimps:
            if blimps[feed_name].frame is not None:
                # Yield the output frame in the byte format
                yield(b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + bytearray(blimps[feed_name].frame) + b'\r\n')

@app.route('/video_feed/<string:feed_name>')
def video_feed(feed_name):
    global blimps
    if feed_name in blimps:
        if blimps[feed_name].frame is not None:
            return Response(generate(feed_name), mimetype='multipart/x-mixed-replace; boundary=frame')
        else:
            return Response(status=204)
    else:
        return Response(status=204)

@app.route('/BurnCreamBlimp')
def burnCreamBlimpPage():
    return render_template('BurnCreamBlimp.html')

@app.route('/SillyAhBlimp')
def sillyAhhBlimpPage():
    return render_template('SillyAhBlimp.html')

@app.route('/TurboBlimp')
def turboBlimpPage():
    return render_template('TurboBlimp.html')  

@app.route('/GameChamberBlimp')
def gameChamberBlimpPage():
    return render_template('GameChamberBlimp.html')

@app.route('/FiveGuysBlimp')
def fiveGuysBlimpPage():
    return render_template('FiveGuysBlimp.html')

@app.route('/SuperBeefBlimp')
def superBeefBlimpPage():
    return render_template('SuperBeefBlimp.html')

@app.route('/Catch1')
def catch1Page():
    return render_template('Catch1.html')

@app.route('/Catch2')
def catch2Page():
    return render_template('Catch2.html')

@app.route('/Barometer')
def baroPage():
    return render_template('Barometer.html')

@app.route('/Logs')
def logsPage():
    return render_template('Logs.html')

@app.route('/Documentation')
def docsPage():
    return render_template('Documentation.html')

# ROS 2 Thread
def ros_thread():
    rclpy.init()

    global node
    node = Basestation()

    try:
        rclpy.spin(node)
    except:
        pass

    try:
        node.destroy_node()
    except rclpy.handle.InvalidHandle:
        pass

    try:
        rclpy.shutdown()
    except Exception as e:
        pass

# Terminate Code
def terminate(sig, frame):
    print('\nDestroying Basestation Node...\n')
    global node
    try:
        node.destroy_node()
    except rclpy.handle.InvalidHandle as e:
        pass
    try:
        rclpy.shutdown()
    except Exception as e:
        pass
    socketio.emit("kill")
    time.sleep(0.5)
    for blimp in blimps:
            try:
                blimps[blimp].parent_node.barometer.close()
            except:
                pass
    print('\nTerminating Program...\n')
    try:
        os.kill(current_pid, sig.SIGTERM)
    except AttributeError:
        os.kill(current_pid, signal.SIGTERM)

# Check Wifi (Currently Not Used)
def check_wifi_ssid():
    output = subprocess.check_output(["iwconfig", "2>/dev/null | grep 'ESSID:' | cut -d '\"' -f 2"], shell=True)
    ssid = output.decode('utf-8').strip()
    if(ssid.find("COREBlimp") == -1 and ssid.find("COREBlimp_5G_1") == -1 and ssid.find("COREBlimp_5G_2") == -1):
        print("Invalid WiFi selected! Must be on COREBlimp")
        return False
    else:
        return True

# Main
if __name__ == '__main__':
    # Future To-Do
    # Create init function for the following values
    # Initialize default value i.e. goal color value (default: 0)
    # Could make these read from a text file to make them permanent profiles

    # Blimps
    global blimps
    blimps = {}

    # All Autonomous
    global auto_panic
    auto_panic = False

    # All Goal Colors
    global all_goal_color
    all_goal_color = False

    # All Target Colors
    global all_target_color
    all_target_color = False

    # Terminate if Ctrl+C Caught
    signal.signal(signal.SIGINT, terminate)

    # Create and Start ROS 2 Thread
    ros_thread = threading.Thread(target=ros_thread)
    ros_thread.start()

    # Start Web Application
    socketio.run(app, host=sys.argv[1], port=5000)

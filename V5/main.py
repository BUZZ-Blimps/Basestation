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
from sensor_msgs.msg import Joy

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

        # Connected Blimp Nodes
        self.connected_blimps = []
        self.ordered_connected_blimps = []

        # Blimp Node Handlers
        self.blimp_node_handlers = []

        # All Blimps Dictionary
        self.all_blimps = {}

        # Create timer
        self.loopSpeed = 10 # Depends on CPU Speed (Future To-Do: Optimize Frontend/UI to run main loop faster)
        timer_period = 1.0/self.loopSpeed
        # self.timer = self.create_timer(timer_period, self.timerLoop)
        self.timeout = 5

        # Create Barometer Timer
        self.barometer = None
        self.barometerLoopSpeed = 100
        barometer_timer_period = 1.0/self.barometerLoopSpeed
        # self.barometerTimer = self.create_timer(barometer_timer_period, self.barometerTimerLoop)
        
        # Identify Topic for Teensy to Check if Basestation is On
        self.topicName_identify = "/identify"
        self.identify_sub = self.create_subscription(String, self.topicName_identify, self.identify_callback, 10)

        #Subscribe to joystick topic
        self.joy_sub = self.create_subscription(Joy, "/Basestation/joy", self.joy_callback, 1)

        self.joy_state = Joy()
        self.joy_state.axes = [0.0,0.0,1.0,0.0,0.0,1.0,0.0,0.0]
        self.joy_state.buttons = [0,0,0,0,0,0,0,0,0,0,0,0]

        self.axis_labels = [
            'Left Joy R/L',
            'Left Joy U/D',
            'L trigger',
            'Rt Joy R/L',
            'Rt Joy U/D',
            'R trigger',
            'Dpad R/L',
            'Dpad U/D'
        ]

        self.button_labels = [
            "A",
            "B",
            "X",
            "Y",
            "L bumper",
            "R bumper",
            "Windows",
            "Three Bars",
            "Unused",
            "Left Joy Press",
            "Right Joy Press",
            "Arrow out of box"
        ]

        self.catching_blimp_ids = ['BurnCreamBlimp', 'SillyAhBlimp', 'TurboBlimp', 'GameChamberBlimp', 'FiveGuysBlimp', 'SuperBeefBlimp', 'Catch1', 'Catch2']
        self.attack_blimp_ids = ['Yoshi', 'Luigi', 'Geoph', 'ThisGuy', 'Attack1', 'Attack2']

        self.controlled_blimp_id = ""
        self.auto_panic = False

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
                self.create_blimp_node_handler(blimp_id)
                blimps[blimp_id].last_online = self.get_clock().now()

    def joy_callback(self, msg):

        # Axes:
        #Joysticks
        for i in [0,1,3,4]:
            if msg.axes[i] != self.joy_state.axes[i]:
                self.joy_state.axes[i] = msg.axes[i]
                # self.get_logger().info('{} moved'.format(self.axis_labels[i]))

        # 1: Left Joy U/D
        if msg.axes[1] != self.joy_state.axes[1]:
            self.joy_state.axes[1] = msg.axes[1]
            # self.get_logger().info('Left joystick U/D')

        # # 2: L trigger
        if msg.axes[2] != self.joy_state.axes[2]:
            if self.joy_state.axes[2] > 0.5 and msg.axes[2] < -0.5:
                #Pressed down
                self.joy_state.axes[2] = msg.axes[2]
                # self.get_logger().info('Left trigger pressed')
            elif self.joy_state.axes[2] < -0.5 and msg.axes[2] > 0.5:
                #Released
                # self.get_logger().info('Left trigger released')
                self.update_auto_panic()
                self.joy_state.axes[2] = msg.axes[2]

        if msg.axes[5] != self.joy_state.axes[5]:
            if self.joy_state.axes[5] > 0.5 and msg.axes[5] < -0.5:
                #Pressed down
                self.joy_state.axes[5] = msg.axes[5]
                # self.get_logger().info('Left trigger pressed')
            elif self.joy_state.axes[5] < -0.5 and msg.axes[5] > 0.5:
                #Released
                # self.get_logger().info('Right trigger released')
                #Toggle autonomous
                self.update_auto()
                self.joy_state.axes[5] = msg.axes[5]

        # # 6: Dpad R/L (not sure what these do - ask Austin)
        if msg.axes[6] != self.joy_state.axes[6]:
            if msg.axes[6] == 0:
                if self.joy_state.axes[6] == 1:
                    self.get_logger().info('Left DPad released')
                else:
                    self.get_logger().info('Right DPad released')

            self.joy_state.axes[6] = msg.axes[6]

            # self.get_logger().info('Dpad R/L')

        # 7: Dpad U/D
        if msg.axes[7] != self.joy_state.axes[7]:
            if msg.axes[7] == 0:
                #Update with the negative of what was sent bc of top-down list
                self.update_controlled_blimp(-self.joy_state.axes[7])

                #Update the frontend for all blimps to reflect the change
                self.update_all_frontend()

                # if self.joy_state.axes[7] == 1:
                #     self.get_logger().info('Up DPad released')
                # else:
                #     self.get_logger().info('Down DPad released')

            self.joy_state.axes[7] = msg.axes[7]



        for i in range(12):
            if msg.buttons[i] != self.joy_state.buttons[i]:
                #Button released
                if msg.buttons[i] == 0:
                    self.get_logger().info('{} button released'.format(self.button_labels[i]))
                    # self.button_labels = [
                    #     "A",
                    #     "B",
                    #     "X",
                    #     "Y",
                    #     "L bumper",
                    #     "R bumper",
                    #     "Windows",
                    #     "Three Bars",
                    #     "Unused",
                    #     "Left Joy Press",
                    #     "Right Joy Press",
                    #     "Arrow out of box"
                    # ]
                    if i == 0:
                        #A button - reload page via emit (TODO)
                        pass
                    elif i == 1:
                        #B button - do nothing
                        pass
                    elif i == 2:
                        #X button
                        pass
                    elif i == 3:
                        pass

                self.joy_state.buttons[i] = msg.buttons[i]

        #Update the motor commands every time a joy update is received
        self.update_motor_commands()

    def update_controlled_blimp(self, direction):
        # print('updating controlled blimp {}'.format(direction))

        #Make sure at least one blimp is currently connected, otherwise do nothing
        num_blimps_connected = len(self.ordered_connected_blimps)
        if num_blimps_connected > 0:
            #First, get the index of the current connected blimp
            valid_index = False
            try:
                index = self.ordered_connected_blimps.index(self.controlled_blimp_id)
                valid_index = True
            except ValueError:
                #No blimp is currently controlled, so select corresponding endpoint
                if direction > 0:
                    self.controlled_blimp_id = self.ordered_connected_blimps[0]
                else:
                    self.controlled_blimp_id = self.ordered_connected_blimps[num_blimps_connected-1]

            if valid_index:
                if direction > 0:
                    if index < num_blimps_connected-1:
                        new_index = index + 1
                    else:
                        #Wraparound
                        new_index = 0
                else:
                    if index > 0:
                        new_index = index - 1
                    else:
                        new_index = num_blimps_connected-1
            
                self.controlled_blimp_id = self.ordered_connected_blimps[new_index]

            #Update handlers to stay consistent
            for blimp_node_handler in self.blimp_node_handlers:
                if blimp_node_handler.blimp.id == self.controlled_blimp_id:
                    blimp_node_handler.blimp.controlled = True
                else:
                    blimp_node_handler.blimp.controlled = False

            print('{} selected for control'.format(self.controlled_blimp_id))

    def update_motor_commands(self):
        for blimp_node_handler in self.blimp_node_handlers:
            if blimp_node_handler.blimp.id == self.controlled_blimp_id:
                left_stick_x = self.joy_state.axes[0]
                left_stick_y = self.joy_state.axes[1]
                right_stick_x = self.joy_state.axes[3]
                right_stick_y = self.joy_state.axes[4]
                controller_cmd = [left_stick_x, left_stick_y, right_stick_x, right_stick_y]
                blimp_node_handler.blimp.motor_commands = controller_cmd
            else:
                blimp_node_handler.blimp.motor_commands = [0.0, 0.0, 0.0, 0.0]
            
            #Publish commands only if blimp is in manual control mode
            if not blimp_node_handler.blimp.auto:
                blimp_node_handler.publish_motor_commands()

    def update_auto(self):
        for blimp_node_handler in self.blimp_node_handlers:
            if blimp_node_handler.blimp.id == self.controlled_blimp_id:
                blimp_node_handler.blimp.auto = not blimp_node_handler.blimp.auto
                blimp_node_handler.publish_auto()
                blimp_node_handler.update_frontend()

    def update_auto_panic(self):
        self.auto_panic = not self.auto_panic
        for blimp_node_handler in self.blimp_node_handlers:
            blimp_node_handler.blimp.auto = self.auto_panic
            blimp_node_handler.publish_auto()
            blimp_node_handler.update_frontend()

    def update_grab(self):
        for blimp_node_handler in self.blimp_node_handlers:
            if blimp_node_handler.blimp.id == self.controlled_blimp_id:
                blimp_node_handler.blimp.grabbing = not blimp_node_handler.blimp.grabbing
                blimp_node_handler.publish_grabbing()
                blimp_node_handler.update_frontend()

    def update_shoot(self):
        for blimp_node_handler in self.blimp_node_handlers:
            if blimp_node_handler.blimp.id == self.controlled_blimp_id:
                blimp_node_handler.blimp.shooting = not blimp_node_handler.blimp.shooting
                blimp_node_handler.publish_shooting()
                blimp_node_handler.update_frontend()

    def update_all_frontend(self):
        for blimp_node_handler in self.blimp_node_handlers:
            blimp_node_handler.update_frontend()

    def update_blimp_node_handlers(self):
        global blimps

        # Finally, check for subscription timeout for all connected blimps
        for blimp_node_handler in self.blimp_node_handlers:
            if self.getElapsedTime(blimps[blimp_node_handler.blimp.id].last_online) > self.timeout:
                self.get_logger().info('Removing blimp {}'.format(blimp_node_handler.blimp.id))
                self.remove_blimp_node_handler(blimp_node_handler.blimp.id)
            # else:
                # Run handler update routine
                # blimp_node_handler.update()

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
        # topic_baseBarometer = "/" + blimp_id + "/baseBarometer"

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
        # new_blimp_node_handler.connect_blimp()

        # Add New Blimp Node to Connected Blimp Nodes
        self.connected_blimps.append(blimp_id)

        #Go ahead and reorder connected blimps now
        self.reorder_connected_blimps()

        # Add Blimp Node Handler to List
        self.blimp_node_handlers.append(new_blimp_node_handler)

    def reorder_connected_blimps(self):
        self.ordered_connected_blimps.clear()

        #Add catching blimps in order first
        for catching_blimp_id in self.catching_blimp_ids:
            for blimp_id in self.connected_blimps:
                if blimp_id == catching_blimp_id:
                    self.ordered_connected_blimps.append(blimp_id)
                    break

        #Next, add attack blimps in order
        for attack_blimp_id in self.attack_blimp_ids:
            for blimp_id in self.connected_blimps:
                if blimp_id == attack_blimp_id:
                    self.ordered_connected_blimps.append(blimp_id)
                    break

    def remove_blimp_node_handler(self, blimp_id):
        global blimps

        # Timeout Detected for Blimp Node
        if blimp_id is not None:
            self.get_logger().info('Detected timeout of blimp ID "%s"' % blimp_id)

            # First, get the handle of the node handler
            handler_found = False
            handler_id = None
            for id, handler in enumerate(self.blimp_node_handlers):
                if handler.blimp.id == blimp_id:
                    handler_found = True
                    handler_id = id

            if not handler_found:
                self.get_logger().error('Blimp ID "%s" Handler not found!' % blimp_id)
                return
            
            # Now, gracefully shut down the node handler
            # Destroy subscribers (publishers will timeout anyways)
            self.blimp_node_handlers[handler_id].destroy_subscribers()
            self.blimp_node_handlers[handler_id].destroy_publishers()

            if self.blimp.controlled:
                self.parent_node.controlled_blimp_id = ""

            # Remove blimp from list of connected blimps
            self.connected_blimps.remove(blimp_id)

            #Reorder connected blimps again
            self.reorder_connected_blimps()

            # Finally, delete the handler object for good
            del self.blimp_node_handlers[handler_id]

            # Delete from global blimps dict
            del blimps[blimp_id]

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
                try:
                    self.barometer = serial.Serial('/dev/ttyACM1', 115200)
                    print('BAROMETER CONNECTED')
                except:
                    print('BAROMETER NOT CONNECTED')

        for blimp_node_handler in self.blimp_node_handlers:
            blimp_node_handler.publish_calibrateBarometer()

        global blimps
        if self.barometer is not None:
                #Read serial data if available
                if self.barometer.in_waiting:
                    # Read a line of data from the serial port
                    data = self.barometer.readline()

                    # Debugging
                    #print(data)  # Assuming data is encoded as UTF-8

                    for blimp in blimps:
                        blimps[blimp].barometer = float(data.decode('utf-8'))

    def getElapsedTime(self, prevTime):
        elapsedTime = self.get_clock().now() - prevTime
        elapsedTimeSec = elapsedTime.nanoseconds / 10**9
        return elapsedTimeSec

class BlimpNodeHandler:
    def __init__(self, parent_node, blimp_id):
        self.parent_node = parent_node
        self.time_created = self.parent_node.get_clock().now()
        
        # Future Todo: Fix redundancies between Blimp objects and handlers (they're one-to-one)
        # Create a Blimp object for storing the blimp state
        self.blimp = Blimp(blimp_id)
        self.blimp.name = self.get_blimp_name(blimp_id)

        # Set the blimp type
        self.blimp_type = self.get_blimp_type(blimp_id)

        # Update the global blimp dictionary with this blimp
        global blimps
        blimps[self.blimp.id] = self.blimp

        # if self.blimp_type == True:
        #     # Attack blimp
        #     self.motor_timer_period = 1/self.attack_motor_timer_hz
        #     self.data_timer_period = 1/self.attack_data_timer_hz
        # else:
        #     # Catching blimp
        #     self.motor_timer_period = 1/self.catching_motor_timer_hz
        #     self.baro_timer_period = 1/self.baro_timer_hz

        self.parent_node.get_logger().info('Identified Blimp with ID "%s"' % str(self.blimp.id))

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
        global_blimp = blimps[self.blimp.id]
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

        if global_blimp.controlled != self.blimp_dict["controlled"]:
            # print('connected updating')

            dict_updated = True
            self.blimp_dict["controlled"] = global_blimp.controlled

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
            socketio.emit('update', blimps[self.blimp.id].to_dict())

    def update_frontend(self):
        socketio.emit('update', self.blimp.to_dict())

    # def update(self):

        # Debugging
        # self.parent_node.get_logger().info("Updating")

        #Update the blimp dictionary if needed
        # self.update_handler_dict()

        # global blimps
        # if self.blimp.id in blimps:

        #     now = self.parent_node.get_clock().now()

            # if self.parent_node.getElapsedTime(self.motor_timer) >= self.motor_timer_period:
            #     try:
            #         self.publish_motor_commands()
            #         self.motor_timer = now
            #     except: 
            #         pass

            # Differentiate catching and attack blimps
            # if (blimps[self.blimp.id].blimp_type == False):
            #     # Catching-blimp specific
            #     if blimps[self.blimp.id].update_grabbing_pub:
            #         try:
            #             self.publish_grabbing()
            #             blimps[self.blimp.id].update_grabbing_pub = False
            #         except:
            #             pass

            #     if blimps[self.blimp.id].update_shooting_pub:
            #         try:
            #             self.publish_shooting()
            #             blimps[self.blimp.id].update_shooting_pub = False
            #         except:
            #             pass

            #     if blimps[self.blimp.id].update_auto_pub:
            #         try:
            #             self.publish_auto()
            #             blimps[self.blimp.id].update_auto_pub = False
            #         except:
            #             pass

            #     if blimps[self.blimp.id].update_goal_color_pub:
            #         try:
            #             self.publish_goal_color()
            #             blimps[self.blimp.id].update_goal_color_pub = False
            #         except:
            #             pass

            #     if blimps[self.blimp.id].update_target_color_pub:
            #         try:
            #             self.publish_target_color()
            #             blimps[self.blimp_id].update_target_color_pub = False
            #         except:
            #             pass

                # # Publish barometer to catching blimps on a timer
                # if self.parent_node.getElapsedTime(self.baro_timer) >= self.baro_timer_period:
                #     try:
                #         self.publish_barometer()
                #         self.baro_timer = now
                #     except:
                #         pass
                
                # try:
                #     self.update_image_subscriber()
                # except:
                #     pass
            # else:
            #     # Attack blimp specific
            #     if self.parent_node.getElapsedTime(self.attack_data_timer) >= self.data_timer_period:
            #         try:
            #             self.publish_auto()
            #             self.publish_target_color()
            #             self.attack_data_timer = now
            #         except:
            #             pass

    def create_publishers(self):
        topic_auto =            "/" + self.blimp.id + "/auto"
        topic_goal_color =      "/" + self.blimp.id + "/goal_color"
        topic_target_color =      "/" + self.blimp.id + "/target_color"
        topic_killed =          "/" + self.blimp.id + "/killed"
        topic_motor_commands =  "/" + self.blimp.id + "/motorCommands"
        topic_grabbing =        "/" + self.blimp.id + "/grabbing"
        topic_shooting =        "/" + self.blimp.id + "/shooting"
        topic_baseBarometer =   "/" + self.blimp.id + "/baseBarometer"
        topic_calibrateBarometer =   "/" + self.blimp.id + "/calibrateBarometer"

        bufferSize = 1
        self.pub_auto = self.parent_node.create_publisher(Bool, topic_auto, self.boolean_qos_profile)
        self.pub_goal_color = self.parent_node.create_publisher(Int64, topic_goal_color, self.boolean_qos_profile)
        self.pub_target_color = self.parent_node.create_publisher(Int64, topic_target_color, self.boolean_qos_profile)
        self.pub_killed = self.parent_node.create_publisher(Bool, topic_killed, self.boolean_qos_profile)
        self.pub_motor_commands = self.parent_node.create_publisher(Float64MultiArray, topic_motor_commands, bufferSize)
        self.pub_grabbing = self.parent_node.create_publisher(Bool, topic_grabbing, self.boolean_qos_profile)
        self.pub_shooting = self.parent_node.create_publisher(Bool, topic_shooting, self.boolean_qos_profile)
        self.pub_baseBarometer = self.parent_node.create_publisher(Float64, topic_baseBarometer, bufferSize)
        self.pub_calibrateBarometer = self.parent_node.create_publisher(Bool, topic_calibrateBarometer, self.boolean_qos_profile)

    def destroy_publishers(self):
        self.pub_auto.destroy()
        self.pub_goal_color.destroy()
        self.pub_target_color.destroy()
        self.pub_killed.destroy()
        self.pub_motor_commands.destroy()
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
        if self.blimp.id in blimps:
            blimps[self.blimp.id].state_machine = msg.data

    # Continually Poll State Machine Data from Teensy
    def logs_callback(self, msg):
        global blimps
        blimps[self.blimp.id].log = msg.data
        if self.blimp.id in blimps:
            # Emit the blimp's logs to the webpage
            socketio.emit('logs', blimps[self.blimp.id].to_dict())

    # Continually Poll Image Raw Data from Pi
    def image_raw_callback(self, msg):
        global blimps
        if self.blimp.id in blimps:
            try:
                # Convert the ROS Image message to a CV2 image (numpy ndarray)
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                self.frame = cv_image

                if self.frame is not None:
                    flag, jpeg = cv2.imencode('.jpg', self.frame)
                    blimps[self.blimp.id].frame = jpeg
                
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
        if self.blimp.id in blimps:
            if bb_dict is not None:
                blimps[self.blimp.id].bounding_box = bb_dict
                # Emit the bounding box data to the webpage
                socketio.emit('bounding_box', blimps[self.blimp.id].bounding_box)

    # Continually Poll Height Data from Teensy
    def height_callback(self, msg):
        global blimps
        if self.blimp.id in blimps:
            if msg.data is not None:
                blimps[self.blimp.id].height = msg.data

    # Continually Poll Z Velocity Data from Teensy
    def z_velocity_callback(self, msg):
        global blimps
        if self.blimp.id in blimps:
            if msg.data is not None:
                blimps[self.blimp.id].z_velocity = msg.data

    def update_image_subscriber(self):
        global blimps
        if blimps[self.blimp.id].show_image is True and self.sub_image_raw is None:

            # Debugging
            # print('Creating Image Subscriber')

            # Image Raw Topic
            topic_image_raw = "/" + self.blimp.id + "/left/image_raw"

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
            topic_bounding_box = "/" + self.blimp.id + "/bounding_box"

            # Subscribe to the Bounding Box Topic
            self.sub_bounding_box = self.parent_node.create_subscription(BoundingBox, topic_bounding_box, self.bounding_box_callback, qos_profile)
        elif blimps[self.blimp.id].show_image is False and self.sub_image_raw is not None:

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
    def get_blimp_type(self, blimp_id):
        blimp_type = 1 if blimp_id in self.parent_node.attack_blimp_ids else 0        
        return blimp_type

    def get_blimp_name(self, blimp_id):
        blimp_names_by_id = {
            # Catching Blimps #
            'BurnCreamBlimp': 'Burn Cream',
            'SillyAhBlimp': 'Silly Ah',
            'TurboBlimp': 'Turbo',
            'GameChamberBlimp': 'Game Chamber',
            'FiveGuysBlimp': 'Five Guys',
	        'SuperBeefBlimp': 'Super Beef',
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
        # Publish goal color value to the ROS topic
        msg = Int64()
        msg.data = self.blimp.target_color
        self.pub_target_color.publish(msg)

    def publish_goal_color(self):
        # Publish goal color value to the ROS topic
        msg = Int64()
        msg.data = self.blimp.goal_color
        self.pub_goal_color.publish(msg)

    def publish_motor_commands(self):
        # Publish motor commands value to the ROS topic
        msg = Float64MultiArray()
        msg.data = self.blimp.motor_commands
        self.pub_motor_commands.publish(msg)

    def publish_auto(self):
        # Publish auto value to the ROS topic
        msg = Bool()
        msg.data = self.blimp.auto
        self.pub_auto.publish(msg)

    def publish_grabbing(self):
        # Publish grabbing value to the ROS topic
        msg = Bool()
        msg.data = self.blimp.grabbing
        self.pub_grabbing.publish(msg)

    def publish_shooting(self):
        # Publish shooting value to the ROS topic
        msg = Bool()
        msg.data = self.blimp.shooting
        self.pub_shooting.publish(msg)

    def publish_barometer(self):
        # Publish barometer value to the ROS topic
        msg = Float64()
        msg.data = self.blimp.barometer
        self.pub_baseBarometer.publish(msg)

    def publish_calibrateBarometer(self):
        global blimps
        # Publish shooting value to the ROS topic
        msg = Bool()
        msg.data = self.blimp.calibrateBarometer
        self.pub_calibrateBarometer.publish(msg)
        if self.blimp.calibrateBarometer == True:
            self.blimp.calibrateBarometer = False

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
    # @socketio.on('update_connection')
    # def update_connection(data):
    #     global blimps
    #     if data in blimps:
    #         blimps[data].connected = True

    # Update Disconnection
    # @socketio.on('update_disconnection')
    # def update_disconnection(data):
    #     global blimps
    #     if data in blimps:
    #         blimps[data].connected = False

    # Update Total Disconnection
    # @socketio.on('update_total_disconnection')
    # def update_total_disconnection():
    #     global blimps
    #     for blimp in blimps:
    #         blimps[blimp].connected = False

    # Subscribe from Image
    # @socketio.on('show_image')
    # def show_image(data):
    #     global blimps
    #     if data in blimps:
    #         blimps[data].show_image = True

    # Destroy Image Subscriber
    # @socketio.on('remove_image')
    # def remove_image(data):
    #     global blimps
    #     if data in blimps:
    #         blimps[data].show_image = False

    # Update Motor Commands
    # @socketio.on('update_motor_commands')
    # def update_motor_commands(data):
    #     array = np.frombuffer(data, dtype=np.float64)
    #     motor_commands = array.tolist()

    #     # Debugging
    #     #print('\nReceived Data:', motor_commands + '\n')

    #     # Iterate through which blimp_name is connected
    #     global blimps
    #     for blimp in blimps:
    #         if blimps[blimp].controlled == True:

    #             # Debugging
    #             #print(blimps[blimp].blimp_name + "connected")

    #             blimps[blimp].motor_commands = motor_commands

    #             # Debugging
    #             #print(blimps[blimp].motor_commands)

    #         else:

    #             # Debugging
    #             #print(blimps[blimp].blimp_name + "not connected")

    #             blimps[blimp].motor_commands = [0.0, -0.0, 0.0, -0.0]

                # Debugging
                #print(blimps[blimp].motor_commands)

    # Update All Goal Colors
    # @socketio.on('update_all_goal_colors')
    # def update_goal_colors():
    #     global blimps
    #     global all_goal_color
    #     all_goal_color = not all_goal_color
    #     for blimp in blimps:
    #         blimps[blimp].goal_color = 1 if all_goal_color else 0
    #         # Set update flag
    #         blimps[blimp].update_goal_color_pub = True

    # Update Grabbing
    # @socketio.on('update_grabbing')
    # def update_grabbing(data):
    #     global blimps
    #     if data in blimps:
    #         blimps[data].grabbing = not blimps[data].grabbing
    #         # Set update flag
    #         blimps[data].update_grabbing_pub = True

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
def ros_node():
    rclpy.init()

    global node
    node = Basestation()
    rclpy.spin(node)

    # try:
    #     rclpy.spin(node)
    # except:
    #     print('rclpy.spin failed')
    #     pass

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
    ros_thread = threading.Thread(target=ros_node)
    ros_thread.start()

    # Start Web Application
    socketio.run(app, allow_unsafe_werkzeug=True, host=sys.argv[1], port=5000)

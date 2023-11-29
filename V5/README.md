# BasestationV5 Guide 🌐📡🎮

## Basic Startup Tutorial
1. To host the web server, open a terminal in the V5 directory and run the command:
```bash
./run.sh
```
2. Open the webpage URL provided in the terminal.
3. To connect to the web server on another device, verify you are on the same Wi-Fi network as the web server.

## Summary
This repository contains the basestation code. The basestation is primarily used to view all available blimps and send commands to the currently connected blimp through the use of an Xbox controller or keyboard. A secondary feature of the basestation is the ability to view livestreams from each blimp, which is available through a hyperlink using the stream section of the main page on the basestation.

The basestation uses Python with the Flask framework for the backend and HTML and JavaScript for the frontend. 
<p align="center">
  
![basestation_main_page](https://github.com/SWAMP-Blimps/Basestation/assets/56363833/1d5f02be-7292-46ce-9468-db5c454bfb45)
<p align="center">
<em>Figure 1. Basestation UI</em>
</p>
</p>

## Communication Overview
The basestation uses a communication network called ROS 2 to publish and subscribe to data. When data is being published (sent) by a device with ROS 2, other devices with ROS 2 can subscribe (access) that data. For our use case, this allows the basestation to know when a blimp is online since a blimp ID is being published by the blimps, which the basestation subscribes to. This communication network allows for bidirectional communication between the basestation and blimps.

In terms of communication between the frontend and backend for the basestation, we use Flask's SocketIO library to have ROS 2 data appear on our UI.
<p align="center">
  
![image](https://github.com/SWAMP-Blimps/Basestation/assets/56363833/8c45b3cc-ed2a-42b6-883f-abc7fef2104d)
<p align="center">
<em>Figure 2. A high-level overview of how the basestation and blimps handle communication</em>
</p>
</p>

## Requirements

- A device with the Ubuntu 20.04 Operating System (https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview)
- Python 3.8.10 installed on the device (https://www.python.org/downloads/release/python-3810/)
- ROS2-Foxy installed on the device (https://docs.ros.org/en/foxy/Installation.html)
- Pip install the following packages: Rclpy, Flask, Flask-SocketIO, Simple-Websocket, OpenCV-Python, Pyserial, and Numpy

## How to Use

1. Run “./run.sh” within a terminal to startup basestation.
2. Click the URL link provided in the terminal to navigate to the basestation.
3. Connect a controller to the administrator's computer. 
4. Activate a blimp to connect to the basestation by plugging in batteries for Orange Pi and Teensy. 
5. Connect a controller to the blimp by using the Xbox controller's d-pad’s up or down arrows to select which blimp to connect to. 
6. As an administrator, select the corresponding goal color for catching blimps by toggling the "Goal" button by clicking it with a mouse or pressing “Y” on the controller. For attack blimps, they can toggle the “Target” button by clicking it with a mouse or pressing “X” on the controller. 
7. Activate autonomous mode on the blimp by pressing “RT” on the controller. Press "LT" to send all the blimps into autonomous mode. 
8. To maneuver manually, deactivate autonomous mode by pressing “RT” or "LT" again and using the left and right sticks on the controller. 
9. Click the "View Stream" hyperlink for the corresponding blimp to navigate to the livestream for that blimp. 
10. Use the sidebar menu to navigate to the Main, Logs, Barometer, or Documentation pages.

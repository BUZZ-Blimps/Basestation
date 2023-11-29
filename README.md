# SWAMP Blimps Basestation 🎈💻🎮

## Summary
- The basestation is used as an interface between human operators and blimps
- [BasestationV5](V5) is the latest working version (Web-based UI with ROS2)
  
![Screenshot](https://github.com/SWAMP-Blimps/Basestation/assets/116739351/7a92a1f9-1ac5-49fb-bf95-2db0ca944dee)

## Description
The "basestation" has been used since the beginning of SWAMP Blimps and serves as an interface between human operators and our swarm of blimps. For most of its history, the basestation has taken the form of a Python program, with a UI rendered with Pygame (shown below).

![image](https://github.com/SWAMP-Blimps/Basestation/assets/116739351/27851673-6e29-45d7-bc16-5b4270a3c82b)

The primary functions of the basestation are:
- List all inputs (keyboard, controllers)
    - Show previews of joystick input
- List all blimps
    - Show whether blimp is in manual/autonomous mode (by text color)
    - Show states of blimps
    - Allow operator to set target goal and enemy colors
- Allow operators to connect specific inputs to specific blimps

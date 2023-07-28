# Basestation V4 README

# Description
Basestation V4 is web-based and uses Flask with the code written in Python for the backend and HTML and Javascript for the frontend. The backend and frontend communicate using the Flask-SocketIO library. A Basestation ROS 2 node is run in parallel to the Flask application.

# Install
Run install.sh in the install directory to install the necessary packages.

# Run
Run run.sh in the run directory to start the webserver. Click the second link to open the webpage. The webpage name is static and can be saved in your browser.

# Src
The Src directory contains the source code. The source code is broken up into four folders as shown below:

src

├── database

│   └── (Files in Database Folder)

├── ros

│   └── (Files in ROS Folder)

├── webserver

│   └── (Files in Webserver Folder)

├── globalVars.py

└── main.py

# To-Do

# Mandatory
- Add Blimp Idenifier to the ROS Node Class and Implement using Global Database Object (db object: db.add_blimp_name(blimp_name_here))
- Create Blimp Class and Fleet Class (In future create separate classes for CatchingBlimp and Attacking Blimp, they will be children of the parent class Fleet)
- Fix WASD and Controller Inputs (Make them happen in the backend and then sent to the frontend; currently it is happening in the frontend only)
- Make Controller attach to a blimp using a variable in the Blimp Class and in HTML
- Add Autonomous Variable that changes Blimp Name to Green in the Blimp Class and in HTML
- Add all Controller Input Features
- Get State of State Machine from James' Teensy Code
- Add Barometer Display

# Other
- All Autonomous Button
- Buttons to Flash Each Pi Wirelessly and all Pi's Wirelessly (Flash All Button)
- Integrate Teensy Code to the Basestation Codebase and Flash Teensy Wirelessly
- Integrate All Vision Code to the Basestation Codebase
- Overall Target Color, Overall Goal Color
- Get Streaming to Basestation Working (Austin and Bryan have some code for this)
- Turn Stream On/Off Button
- Dedicated Streaming Page
- Dedicated Page showing ROS Current Nodes and Topics
- Dedicated Page for debugging that grabs current data from specific ROS topics
- Dedicated Page for tuning PID parameters in real-time
- Manually Change State of Blimp from Basestation (Very Difficult)
- Add Xbox Button Indicator for Basestation Functionality
- Documentation Page of how to use the Basestation with link to YouTube Video


# 🥶 Basestation V3 Launch Tutorial (Basic) 🥶
1. Go to BasestationV3 workspace directory:
```bash
cd /home/GitHub/Basestation/V3_ws
```
2. Source the workspace:
```bash
source install/local_setup.bash
```
3. Run the launch file:
```bash
ros2 launch launch/Basestation.py
```

# 🥵 ROS2 Guide for Basestation V3 with Python (Advanced) 🥵
ROS2 Tutorials Documentation: https://docs.ros.org/en/foxy/Tutorials.html


## ROS2 Installation
General Install Documentation: https://docs.ros.org/en/foxy/Installation.html

Ubuntu Debian Install Documentation: https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html#id2

### Automatic Install Script
1. Go to GitHub Repository "CatchingBlimp"
2. Run the install script
```bash
./InstallROS2.sh
```

Once the script finishes, it will tell you the following steps:
1. Run the following command: 
```bash
nano ~/.bashrc
```
2. At the bottom, add the line: 
```bash
source /opt/ros/foxy/setup.sh
```
3. Save and exit ~/.bashrc
4. Run the following command:
```bash
source ~/.bashrc
```

### Verify Installation
Open a terminal and run:
```bash
ros2 -h
```
If the installation was successful, you should get a nicely formatted help message from ROS2.


## Installing colcon
Documentation: https://colcon.readthedocs.io/en/released/user/installation.html

Run the following commands:
```bash
sudo sh -c 'echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'

curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add

sudo apt update

sudo apt install python3-colcon-common-extensions
```


## Create workspace, package, and Hello World node
Documentation: https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html

To create workspace "V3_ws", run command:
```bash
mkdir -p V3_ws/src
```

To create package "BasestationV3", run command within the V3_ws/src folder:
```bash
ros2 pkg create --build-type ament_python BasestationV3
```

To create node "HelloWorld", within the folder BasestationV3/BasestationV3, create a file named "HelloWorld.py". To make it print "Hello World! :)", within the file "HelloWorld.py", write the following code:
```python
def main():
    print("Hello World! :)")
```

To make this node execute, within BasestationV3/setup.py, add a new entry point, written as:
```python
entry_points={
    'console_scripts': [
        'HelloWorld = BasestationV3.HelloWorld:main'
    ],
},
```

Additionally, open BasestationV3/setup.cfg and make sure there are no dash-commands. As in, make sure
```
script-dir -> script_dir
install-scripts -> install_scripts
```

ALSO, because ros2 and setuptools are bad, make sure your setuptools install is old (lmao). setuptools is deprecated and recent versions print a warning. The common solution to this is to *downgrade* setuptools to a version that doesn't print warnings (although we should really try to find a solution to this). In a terminal, run the command:
```bash
pip install setuptools==58.2.0
```

Finally, to build this package, go to the workspace directory and run the command:
```bash
colcon build
```

To run the HelloWorld node:
1. Source the package by going to the workspace directory and running the command:
```bash
source install/local_setup.bash
```
2. Run the node by running the command:
```bash
ros2 run BasestationV3 HelloWorld
```

## Notes for connecting to the basestation
For a blimp to connect to the basestation, it needs to be running a ROS2 node that fulfills the following 2 conditions:
1. The node needs to subscribe to the topic "/identify"
2. The node needs to publish its blimp ID under the topic /\<node name>/blimpID

The basestation is constantly looking at all nodes that are subscribed to "/identify". When it detects that a new node has subscribed to "/identify", the basestation then subscribes to "/\<node name>/blimpID". Once the basestation receives a blimpID from this topic, the basestation considers the blimp "connected" and communication will begin. Information about the blimp will then be published under "/\<node name>/\<information>", including information such as motor commands, barometer data, and various blimp state data.

It is recommended that the blimp's node is namespaced for easy access to its relevant data. For instance, if we go through the above process with node named "BlimpX", then the basestation will publish topics such as "/blimpX/motorCommands", "/blimpX/baseBarometer", and "/blimpX/grabbing". Within this blimp's node, we can easily access the published blimp data by subscribing to the respective topics "motorCommands", "baseBarometer", and "grabbing", allowing the namespace to add the appropriate prefixes.

For backwards compatibility, the published blimpID from each node should probably be that device’s IP address, expressed as a string.

## Notes
- In Python, it seems like paths get changed a bit when built with ROS2, so path names should be changed:
    - From Text import Text -> From .Text import Text

## Allow PyCharm to see ROS2 Packages
1. File -> Settings -> Python Interpreter
2. Select drop-down menu -> Show All
3. Select python interpreter (likely Python 3.8) -> Show Interpreter Paths
4. Click the "+" to add a new path
5. Add /opt/ros/foxy/lib/python3.8/site-packages

## Create a launch file
Documentation: https://docs.ros.org/en/foxy/Tutorials/Intermediate/Launch/Creating-Launch-Files.html

1. Go to workspace directory
2. Create new folder, "launch"
3. Create a new python file to be your new launch file
4. Add the following contents for a basic launch file:
```python
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='Blimp',
            namespace='Blimp1',
            executable='Blimp',
            name='sim'
        )
    ])
```
5. To run the launch file, go to the "launch" directory and run the following command:
```bash
ros2 launch <launch file name>.py
```
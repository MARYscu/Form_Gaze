# Form_Gaze

## Nao Usage 
1. Download the ROS-TCP-Endpoint ros package (https://github.com/Unity-Technologies/ROS-TCP-Endpoint) into the src folder of the catkin workspace. 
2. Run `roscore` in one terminal, open a second terminal and `rosrun` the endpoint (more instructions provided by the package's github page) to open a server that provides the connection between the robot and the game. 
3. In a third terminal use the command `rosrun nao_robot_study robot_controller.py` to run Nao's script (command may require adjusting depending on the os and the setup of the CMakeLists)

## Kuka Usage
1. Use the Kuka laptop! The control wrapper and ros indigo is already setup and configured on it. 
2. Download the ROS-TCP-Endpoint ros package (https://github.com/Unity-Technologies/ROS-TCP-Endpoint) into the src folder of the catkin workspace. 
3. Run the kuka control wrapper in one terminal, open a second terminal and `rosrun` the endpoint (more instructions provided by the package's github page) to open a server that provides the connection between the robot and the game. 
4. In a third terminal use the command `rosrun youbot_ros_hello_world youbot_ros_hello_world` to run Kuka's script (command may require adjusting depending on the os and the setup of the CMakeLists)

## for self-reference
export PYTHONPATH=${PYTHONPATH}:/Library/Frameworks/Python.framework/Versions/2.7/lib/python2.7/site-packages/pynaoqi/lib/python2.7/site-packages
export   DYLD_LIBRARY_PATH=${DYLD_LIBRARY_PATH}:/Library/Frameworks/Python.framework/Versions/2.7/lib/python2.7/site-packages/pynaoqi/lib
https://www.aldebaran.com/en/support/nao-6/downloads-softwares

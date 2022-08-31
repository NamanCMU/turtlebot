The package 'xihelm_turtlebot' performs the task of orienting the robot to the Polaris/ North star from anywhere on the Earth.
1. It does that by subscribing to the IMU of the turtlebot and finding the YAW.
2. Then, it runs a simple PID controller to correct the angle by publishing the desired velocity to the robot.

There are two modes:
1. The robot continuously tries to orient itself to the North star autonomously.
2. The robot moves around randomly.
You can switch between these modes by pressing 't' on the keyboard.

The package has the following structure:
CMakeLists.txt
package.xml
setup.py
test
  - xihelm_node_utest.py
  - xihelm_node_utest.launch
configs
  - runlint.py
msg
  - NodeStatus.msg
launch
  - xihelm_simulator.launch
  - xihelm_control.launch
scripts/xihelm_turtlebot
  - __init__.py
  - xihelm_node.py
  - xihelm_keyboard_control.py
  - xihelm_controller.py
  - xihelm_pid.py
  - xihelm_transformation.py
media
  - rqt_graph.png
  - Xihelm_Blockdiagram.jpeg
  - Xihelm_swchallenge.mkv

Steps to run
(Please note that to run, we will require turtlebot3_simulations (https://github.com/ROBOTIS-GIT/turtlebot3_simulations) 
, turtlebot3 (https://github.com/ROBOTIS-GIT/turtlebot3) and turtlebot3_msgs (https://github.com/ROBOTIS-GIT/turtlebot3_msgs) packages.
Please run 'make' in the catkin_ws/ before running the following commands.
1. In terminal 1, run 'roslaunch xihelm_turtlebot xihelm_simulator.launch'
2. In terminal 2, run 'rosrun xihelm_turtlebot xihelm_keyboard_control.py'
3. In terminal 3, run 'roslaunch xihelm_turtlebot xihelm_control.launch'

A) Once you run the above commands, you will see the robot orienting itself to the Polaris.
B) Then, go to the terminal (terminal 2) where xihelm_keyboard_control.py was running and press 't'.
The robot will start to move around randomly. Press 't' again and see the robot orienting itself
to the Polaris. Repeat this to test multiple times.

Key points:
1. I am using roslint to perform linting. I have also added a python linting file 'roslint.py' in the configs folder.
2. I am using rostest and unittest to perform testing. (See test folder)
3. I have also added in the 'Node health status monitor' which can be extended to monitor
the status of each node. (Check xihelm_node.py and msg/NodeStatus.msg)
4. I have added a Makefile in catkin_ws to ease in building the code.
a) make : This will first clean, then build, then run linting and eventually run the tests.
b) make lint : This will run the linting test. 
c) make tests : This will run the unit/ integration tests.
5. I have also attached a block diagram/ high-level software architecture, 'Xihelm_Blockdiagram.jpeg' inside the media folder.
This simply shows the connection/ relation between different nodes.
6. Also, there is a screenshot of the rqt graph, 'rqt_graph.png' inside the media folder.
7. Lastly, I have attached a video as well, 'Xihelm_swchallenge.mkv' inside the media folder.

Thank you.
Naman

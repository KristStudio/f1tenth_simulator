# F1TENTH Racecar Simulator
modified for TechX 2020, originally at https://github.com/tianbot/tianracer


## HW for Aug 8 and Aug 9
1. Follow this README to install relavant dependencies and catkin_make this project.
2. Try to modify node/auto_drive.py to make the car go around without crashing. You may reference
these websites
- Simple Wall Following: https://drive.google.com/file/d/1tzyfGYq3JjvLlYHIiSTyvoNq4kcPf53n/view.
A simple P controller will suffice. That is,`output = P * error`, where P is a constant
- For advanced students, Disparity Extender: https://www.nathanotterness.com/2019/04/the-disparity-extender-algorithm-and.html 
- For more advanced students, mixed wall-following and disparity extender: https://medium.com/@chardorn/running-an-f1tenth-car-like-a-real-racecar-f5da160d8573 

## ROS

### Dependencies
If you have ```ros-melodic-desktop``` installed, the additional dependencies you must install are:

- tf2_geometry_msgs
- ackermann_msgs
- joy
- map_server

You can install them by running:

    sudo apt-get install ros-melodic-tf2-geometry-msgs ros-melodic-ackermann-msgs ros-melodic-joy ros-melodic-map-server

The full list of dependencies can be found in the ```package.xml``` file.

### Installation

To install the simulator package, clone the repo with the simulator and starter code into your catkin workspace:

    cd ~/catkin_ws/src
    git clone https://github.com/chenyx512/f1tenth_simulator.git
    
Then run ```catkin_make``` to build it:

    cd ~/catkin_ws
    catkin_make
    source devel/setup.bash

## Quick Start

To run the simulator on its own, run:

    roslaunch f1tenth_simulator simulator.launch

This will launch everything you need for a full simulation; roscore, the simulator, a preselected map, a model of the racecar and the joystick server.

To manually control the car using a keyboard, press k, then use the standard WASD buttons for acceleration and steering, and pressing the space bar will bring the car to a halt.

To start the stupid auto_drive program, press p

Note: when crushed, you may need to press p or k again to reenter keyboard/auto control

### RVIZ Visualization

You can use keyboard joystick to drive the car around, or you can place the car manually by clicking the "2D Pose Estimate button" on the top of the screen and dragging your mouse on the desired pose.

### Change of Maps

Under directory f1tenth_simulator/maps you can see some different maps. The default is `levine_blocked` but you can change it after trying on the first map. To change map, modify line 7 of `launch/simulator.launch`, for example change that line to  

`  <arg name="map" default="$(find f1tenth_simulator)/maps/ columbia.yaml"/>`
 
will change the map to `columbia`
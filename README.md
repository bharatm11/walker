# walker
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

## Overview

This is a ROS package that contains a walker node which makes a Turtlebot perform obstacle avoidance. The node subscribes
to the topic /scan to detect obstacle and uses this information to calculate and publish to the 
/mobile_base/commands/velocity topic.

## Build and install the package

```
#Make catkin workspace
mkdir -p catkin_ws/src
cd ~/catkin_ws
#Initialize catkin workspace
catkin_make 
cd src/
#Clone package
git clone https://github.com/bharatm11/walker
cd ..
#Build Workspace
catkin_make 
```

## Running roslaunch

```
roslaunch walker walker.launch
```

By default, rosbag file recording is disabled. To run the node while recording the topics in a rosbag file, run:

```
roslaunch walker walker.launch record:=true
```

## Rosbag playback

Now terminate the process and open three new terminals

```
roscore
```
Open another terminal
```
rosbag play ./results/myrosbag_2018-11-19-17-16-15.bag
```
Open another new terminal again
```
rostopic echo /scan
```
to view the recorded /scan topic

Note: The name of the rosbag file can be different. The turtlebot enviroment on Gazebo needs to be running to 
visualize the rosbag file. This can be done by running,
```
roslaunch turtlebot_gazebo turtlebot_world.launch
```

## Dependencies

ROS kinetic

/**
* BSD 3-Clause LICENSE
*
* Copyright (c) 2018, Bharat Mathur
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are
* met:
*
* 1. Redistributions of source code must retain the above copyright notice,
* this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its
* contributors may be used to endorse or promote products derived from this
* software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
* IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
* THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
* PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
* CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
* THE POSSIBILITY OF SUCH DAMAGE.

/**
* @file main.cpp
* @author Bharat Mathur
* @date 17 November 2018
* @copyright 2018 Bharat Mathur
* @brief This file implements the walker node to move the turtlebot in order
* to avoid obstacles
*/

#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include "ros/ros.h"
#include "walker/walker_sensor.h"
#include "walker/walker.h"

int main(int argc, char **argv) {
  //initiate node
  ros::init(argc, argv, "walker");
  //initiate class objects
  walker_sensor wkSense;
  walker wk;
  //threshold is the distance from object when robot starts turning
  double threshold = 1;
  //initiate node
  ros::NodeHandle n;
  //this publishes geometry_msgs::Twist commands to the robot
  ros::Publisher robPub =
  n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000);
  //this reads the laser sensor
  ros::Subscriber laserSensorSub
  = n.subscribe("/scan", 100, &walker_sensor::laserCallback, &wkSense);
  ros::Rate loop_rate(10);
  //output is the geometry_msgs::Twist fed to the robot
  geometry_msgs::Twist output;
  while (ros::ok()) {
    //if detect obstacle returns true
    if (wkSense.detectObstacle(threshold)) {
      output = wk.turn();
      ROS_INFO_STREAM("Turning");
    } else {  //if detect obstacle returns false
      output = wk.goStraight();
      ROS_INFO_STREAM("Going Straight");
    }
    robPub.publish(output);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

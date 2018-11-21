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
* @file walker_sensor.cpp
* @author Bharat Mathur
* @date 17 November 2018
* @copyright 2018 Bharat Mathur
* @brief This file implements the methods for class "walker_sensor"
* This class cpp file defines data members and methods applicable for class
* walker_sensor to read laser sensor values and determine whether the robot
* needs to turn or not
*/

#include "walker/walker_sensor.h"
/**
* @brief This is the callback function for the laser sensor. It also gets the
* minimum distance to an obstacle
* @param [in] scan is a sensor_msgs::LaserScan message which contains distance
* data.
* @return void
*/
void walker_sensor::laserCallback(const sensor_msgs::LaserScanConstPtr& scan) {
  //get minimum distance value from scan and equate to walker_sensor::distMin
  // arbitrary max value
  float min = 100;
  // custom min function to get closest point
  for (auto i : scan->ranges) {
    if (!std::isnan(i)) {
      if (i < min) { min = i;}
    }
  }
  ROS_DEBUG("Distance: %f", min);
  this->distMin = min;
}
/**
* @brief This function determines if the robot needs to turn or not by
* comparing the distMin with distThreshold
* @param [in] distThreshold is the minimum threshold distance at which the
* robot is supposed to turn
* @return bool
*/
bool walker_sensor::detectObstacle(double distThreshold) {
  //if there is an obstacke
  if (this->distMin < distThreshold) {
    return true;
  } else { //if there is no obstacke
    return false;
  }
}

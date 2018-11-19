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
* @file walker_sensor.h
* @author Bharat Mathur
* @date 16 November 2018
* @copyright 2018 Bharat Mathur
* @brief This file defines the methods for class "walker_sensor" to read laser
* sensor values and determine whether the robot needs to turn or not
*/
#ifndef ENPM_SOFT_DEV_COURSE_WALKER_WS_SRC_WALKER_INCLUDE_WALKER_WALKER_SENSOR_H_
#define ENPM_SOFT_DEV_COURSE_WALKER_WS_SRC_WALKER_INCLUDE_WALKER_WALKER_SENSOR_H_

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <ros/console.h>
#include "ros/ros.h"

class walker_sensor {
  private:
    /**
    * @brief distMin is the minimum distance value recorded from the laser sensor
    */
    double distMin;

  public:
    /**
    * @brief This is the callback function for the laser sensor. It also gets the
    * minimum distance to an obstacle
    * @param [in] scan is a sensor_msgs::LaserScan message which contains distance
    * data.
    * @return void
    */
    void laserCallback(const sensor_msgs::LaserScanConstPtr& scan);
    /**
    * @brief This function determines if the robot needs to turn or not by
    * comparing the distMin with distThreshold
    * @param [in] distThreshold is the minimum threshold distance at which the
    * robot is supposed to turn
    * @return bool
    */
    bool detectObstacle(double distThreshold);
  };



#endif  // ENPM_SOFT_DEV_COURSE_WALKER_WS_SRC_WALKER_INCLUDE_WALKER_WALKER_SENSOR_H_

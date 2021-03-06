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
* @file walker.cpp
* @author Bharat Mathur
* @date 17 November 2018
* @copyright 2018 Bharat Mathur
* @brief This file implements the methods for class "walker"
* This class cpp file defines data members and methods applicable for class
* walker to make the robot go straight or turn
*/

#include "walker/walker.h"

/**
* @brief This is makes the robot go on in a straight line
* @return geometry_msgs::Twist
*/
geometry_msgs::Twist walker::goStraight() {
  rob_output.linear.x = 0.5;
  rob_output.linear.y = 0;
  rob_output.linear.z = 0;
  rob_output.angular.x = 0;
  rob_output.angular.y = 0;
  rob_output.angular.z = 0;
  ROS_INFO_STREAM("wk.rob_output.linear.x");
  return rob_output;
}
/**
* @brief This makes the robot turn
* @return geometry_msgs::Twist
*/
geometry_msgs::Twist walker::turn() {
  rob_output.linear.x = 0;
  rob_output.linear.y = 0;
  rob_output.linear.z = 0;
  rob_output.angular.x = 0;
  rob_output.angular.y = 0;
  rob_output.angular.z = M_PI_2;
  ROS_INFO_STREAM("hgut");
  return rob_output;
}

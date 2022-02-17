// Copyright 2022 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef FSM_BUMP_GO_BUMPGO_ADVANCED_H
#define FSM_BUMP_GO_BUMPGO_ADVANCED_H

#include "ros/ros.h"

#include "sensor_msgs/LaserScan.h"
#include "fsm_bump_go/BumpGo.h"

namespace fsm_bump_go
{

class BumpGoLaser : BumpGo
{
public:
  BumpGoLaser();
  
  void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg);
  void LaserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
  void step();
private:
  static const int TURNING_RIGHT = 3;
  const float DISTANCE_ = 0.4;  // minim distance the kobuki change state
  const float SECURE_DISTANCE_ = 0.3;  // minim distance the kobuki change state
  int max_array_;  // contains the max size of the array
  int max_sweep_left;  // max left angle kobuki uses to detect
  int min_sweep_right;  // min left angle kobuki uses to detect (center)
  int max_sweep_right;  // max rigth angle kobuki uses to detect
  int min_sweep_left;  // min rigth angle kobuki uses to detect (center)
  int security_max_sweep;  // max back angle kobuki uses to detect backwards objects
  int security_min_sweep;  // min back angle kobuki uses to detect backwards objects
  int laser_detected_;  // true when the laser detect something (on any side)
  bool left_laser_detected_;  // true when the laser detect something on the left side. False on the rigth side.
  bool left_bumper_pressed_;  // true when the bumper detect something on the left side. False on the rigth side.
  int security_laser_detected_;  // true when the laser detect something on the back

  ros::Subscriber sub_laser_;
  ros::Publisher pub_led2_;
};

}  // namespace fsm_bump_go

#endif  // FSM_BUMP_GO_BUMPGO_ADVANCED_H

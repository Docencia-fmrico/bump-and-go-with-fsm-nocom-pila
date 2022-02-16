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

#include "kobuki_msgs/BumperEvent.h"
#include "kobuki_msgs/Led.h"
#include "geometry_msgs/Twist.h"
#include "fsm_bump_go/BumpGo.h"

namespace fsm_bump_go
{

class BumpGoAD : BumpGo
{
public:
  BumpGoAD() : BumpGo(){
    pub_led2_ = n_.advertise<kobuki_msgs::Led>("/mobile_base/commands/led2", 1);
    };

  void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg);
  void step(); 
private:
  bool left_pressed_;
  static const int TURNING_RIGHT = 3;
  ros::Publisher pub_led2_;
  

};

}  // namespace fsm_bump_go

#endif  // FSM_BUMP_GO_BUMPGO_ADVANCED_H

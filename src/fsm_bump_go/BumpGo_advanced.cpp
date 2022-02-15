// Copyright 2019 Intelligent Robotics Lab
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

#include "ros/ros.h"
#include "fsm_bump_go/BumpGo_advanced.h"

namespace fsm_bump_go
{

  void BumpGoAD :: bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
  {
      BumpGo::bumperCallback(msg);
      left_pressed_ = msg->bumper == kobuki_msgs::BumperEvent::LEFT; // Updates the variable "left_pressed_"
  }

  void BumpGoAD :: step()
  {
    if (left_pressed_) // If left bumper is pressed, turning speed is changed to TURNING_RIGHT ( <0 ) in order to avoid the obstacle
    {
      TURNING_SPEED = TURNING_RIGHT;
    } // If other bumper is detected, the default speed becomes TURNING_LEFT ( >0 )
    else
    {
      TURNING_SPEED = TURNING_LEFT;
    }
    BumpGo::step();
  }

}

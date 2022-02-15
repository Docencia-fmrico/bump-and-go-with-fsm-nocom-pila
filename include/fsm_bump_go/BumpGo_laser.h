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

#include "kobuki_msgs/BumperEvent.h" // modificar el include para el laser.
#include "geometry_msgs/Twist.h"
#include "fsm_bump_go/BumpGo.h"

namespace fsm_bump_go
{

class BumpGoLaser : BumpGo
{
public:
  BumpGoLaser() : BumpGo(){};

  void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg);
  void step();
private:
  const float TURNING_LEFT = 0.35;
  const float TURNING_RIGHT = -0.35;

};

}  // namespace fsm_bump_go

#endif  // FSM_BUMP_GO_BUMPGO_ADVANCED_H

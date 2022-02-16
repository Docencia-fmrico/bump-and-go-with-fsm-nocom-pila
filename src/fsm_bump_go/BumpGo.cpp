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
#include "fsm_bump_go/BumpGo.h"

namespace fsm_bump_go
{
BumpGo::BumpGo(): state_(GOING_FORWARD), pressed_(false)
{
  sub_bumber_ = n_.subscribe("/mobile_base/events/bumper", 1, &BumpGo::bumperCallback, this); // ("<topic al que te suscribes>", 1, &clase::callbackNecesario, this)
  pub_vel_ = n_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1); // advertise<tipo de mensaje>("<topic en el que publicar>", 1)
  //pub_led1_ = n_.advertise<
}

void BumpGo::bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
{
  pressed_ = msg->state == kobuki_msgs::BumperEvent::PRESSED;   // pressed_ cambiara a True si msg->state es PRESSED (1)

  // para detectar que lado ha sido presionado se anadira aqui

}

void BumpGo::step()
{
  geometry_msgs::Twist cmd;

  switch (state_)
  {
  case GOING_FORWARD:
    cmd.linear.x = LINEAR_SPEED;
    if (pressed_)
    {
      press_ts_ = ros::Time::now();
      state_ = GOING_BACK;
      ROS_INFO("GOING_FORWARD -> GOING_BACK");
    }
    break;

  case GOING_BACK:
    cmd.linear.x = -LINEAR_SPEED;
    if ((ros::Time::now() - press_ts_).toSec() > BACKING_TIME )
    {
      turn_ts_ = ros::Time::now();
      state_ = TURNING;
      ROS_INFO("GOING_BACK -> TURNING");
    }
    break;

  case TURNING:
    cmd.angular.z = TURNING_SPEED;
    if ((ros::Time::now()-turn_ts_).toSec() > TURNING_TIME )
    {
      state_ = GOING_FORWARD;
      ROS_INFO("TURNING -> GOING_FORWARD");
    }
    break;
  }

  pub_vel_.publish(cmd);
}
}

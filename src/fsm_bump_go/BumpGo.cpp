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
  sub_bumber_ = n_.subscribe("/mobile_base/events/bumper", 1, &BumpGo::bumperCallback, this);
  pub_vel_ = n_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
  pub_led1_ = n_.advertise<kobuki_msgs::Led>("/mobile_base/commands/led1", 1);
}

void BumpGo::bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
{
  pressed_ = msg->state == kobuki_msgs::BumperEvent::PRESSED; // Updates the variable "pressed_"
}

void BumpGo::step()
{
  geometry_msgs::Twist cmd;
  kobuki_msgs::Led led;

  switch (state_)
  {
  case GOING_FORWARD:  // first state of the state machine. Kobuki turn off leds and go forward
    cmd.linear.x = LINEAR_SPEED;
    led.value = LED_APAGADO;
    pub_led1_.publish(led);
    if (pressed_)
    {
      press_ts_ = ros::Time::now();
      state_ = GOING_BACK;
      ROS_INFO("GOING_FORWARD -> GOING_BACK");
    }
    break;

  case GOING_BACK:  // second state of the state machine. Kobuki go backwards
    cmd.linear.x = -LINEAR_SPEED;
    if ((ros::Time::now() - press_ts_).toSec() > BACKING_TIME )
    {
      turn_ts_ = ros::Time::now();
      state_ = TURNING_LEFT;
      ROS_INFO("GOING_BACK -> TURNING");
    }
    break;

  case TURNING_LEFT:  // third state of the state machine. Kobuki turn on led and turn
    cmd.angular.z = TURNING_SPEED;
    led.value = LED_ROJO;
    pub_led1_.publish(led);
    if ((ros::Time::now()-turn_ts_).toSec() > TURNING_TIME )
    {
      state_ = GOING_FORWARD;
      ROS_INFO("TURNING_LEFT -> GOING_FORWARD");
    }
    break;
  }
  pub_vel_.publish(cmd);
}
}
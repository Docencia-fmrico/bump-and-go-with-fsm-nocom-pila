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
#include "fsm_bump_go/BumpGo_laser.h"

namespace fsm_bump_go
{
  BumpGoLaser::BumpGoLaser()
  {
    sub_laser_ = n_.subscribe("/scan", 1, &BumpGoLaser::LaserCallback, this); // ("<topic al que te suscribes>", 1, &clase::callbackNecesario, this)
    pub_vel_ = n_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1); // advertise<tipo de mensaje>("<topic en el que publicar>", 1)
    pub_led1_ = n_.advertise<kobuki_msgs::Led>("/mobile_base/commands/led1", 1);
    pub_led2_ = n_.advertise<kobuki_msgs::Led>("/mobile_base/commands/led2", 1);
  }

  void BumpGoLaser :: LaserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
  {
    min_sweep_left = msg->ranges.size() - msg->ranges.size()/8;
    max_sweep_left = msg->ranges.size() -1;
    min_sweep_right = 0;
    max_sweep_right = msg->ranges.size()/8;
    
    for (int i = min_sweep_left; i < max_sweep_left; i++)
    {
      measurement_ = msg->ranges[i] >= 0.3;
      if(!measurement_)
      {
        izquierda_ = true;
        break;
      }
    }
    for (int i = min_sweep_right; i < max_sweep_right; i++)
    {
      if(!measurement_)
      {
        break;
      }
      measurement_ = msg->ranges[i] >= 0.3;
      if(!measurement_)
      {
        izquierda_ = false;
        left_pressed_ = false;
        break;
      }
    }    
  } 


  void BumpGoLaser :: bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
  {
      BumpGo::bumperCallback(msg);
      left_pressed_ = msg->bumper == kobuki_msgs::BumperEvent::LEFT; // Updates the variable "left_pressed_"
      if(!left_pressed_)
      {
        izquierda_ = false;
      }
  }


  void BumpGoLaser :: step()
  {
    geometry_msgs::Twist cmd;
    kobuki_msgs::Led led;
    led.value = LED_ROJO; 

    switch (state_)
    {
    case GOING_FORWARD:
      cmd.linear.x = LINEAR_SPEED;
      if ((!measurement_) || (pressed_))
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
        if((left_pressed_) || (izquierda_))
        {
          state_ = TURNING_RIGHT;
        }else{
          state_ = TURNING_LEFT;
        }
        ROS_INFO("GOING_BACK -> TURNING");
      }
      break;

    case TURNING_LEFT:
      cmd.angular.z = TURNING_SPEED;
      pub_led1_.publish(led);
      if ((ros::Time::now()-turn_ts_).toSec() > TURNING_TIME )
      {
        state_ = GOING_FORWARD;
        ROS_INFO("TURNING_LEFT -> GOING_FORWARD");
        led.value = LED_APAGADO;
        pub_led1_.publish(led);
      }
      break;
    

    case TURNING_RIGHT:
      cmd.angular.z = - TURNING_SPEED;
      pub_led2_.publish(led);
      if ((ros::Time::now()-turn_ts_).toSec() > TURNING_TIME )
      {
        state_ = GOING_FORWARD;
        ROS_INFO("TURNING_RIGTH -> GOING_FORWARD");
        led.value = LED_APAGADO;
        pub_led2_.publish(led);
      }
      break;
    } 

    pub_vel_.publish(cmd);

  }
}
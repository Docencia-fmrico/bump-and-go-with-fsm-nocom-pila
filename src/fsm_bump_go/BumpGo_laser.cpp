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
  sub_laser_ = n_.subscribe("/scan", 1, &BumpGoLaser::LaserCallback, this);
  pub_vel_ = n_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
  pub_led1_ = n_.advertise<kobuki_msgs::Led>("/mobile_base/commands/led1", 1);
  pub_led2_ = n_.advertise<kobuki_msgs::Led>("/mobile_base/commands/led2", 1);
}

void BumpGoLaser :: LaserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  max_array_ = (msg->angle_max - msg->angle_min) / msg->angle_increment;

  max_sweep_left = max_array_ - max_array_/10;  // initialization of every angle of detection
  min_sweep_left = max_array_;
  min_sweep_right = 0;
  max_sweep_right = max_array_/10;

  max_array_ = (msg->angle_max - msg->angle_min) / msg->angle_increment;  // initialization of every angle of security detection
  security_min_sweep = (2 * max_array_ / 6);
  security_max_sweep = (4 * max_array_ / 6);

  for (int i = max_sweep_left; i < min_sweep_left; i++)  // reads every value in the range (LEFT)
  {
    laser_detected_ = msg->ranges[i] >= DISTANCE_;  // updates the value of laser_detected_ (true if is under 0.4m)
    if(!laser_detected_)
    {
      left_laser_detected_ = true;
      break;
    }
  }
  
  for (int i = min_sweep_right; i < max_sweep_right; i++)  // reads every value in the range (RIGTH)
  {
    if(!laser_detected_)  // to prevent the case that the last value of the left range detect an object and the rigth range overwrite it
    {
      break;
    }
    laser_detected_ = msg->ranges[i] >= DISTANCE_;  // Updates the value of laser_detected_ (true if is under 0.4m)
    if(!laser_detected_)
    {
      left_laser_detected_ = false;
      left_bumper_pressed_ = false;
      break;
    }
  }  

  for (int i = security_min_sweep; i < security_max_sweep; i++)  // reads every value in the back range
  {
    security_laser_detected_ = msg->ranges[i] >= SECURE_DISTANCE_;  // updates the value of laser_detected_ (true if is under 0.3m)
    if(!security_laser_detected_)
    {
      break;
    }
  }  
} 


void BumpGoLaser :: bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
{
    BumpGo::bumperCallback(msg);
    left_bumper_pressed_ = msg->bumper == kobuki_msgs::BumperEvent::LEFT; // Updates the variable "left_bumper_pressed_"
    if(!left_bumper_pressed_)
    {
      left_laser_detected_ = false;
    }
}

void BumpGoLaser :: step()
{
  geometry_msgs::Twist cmd;
  kobuki_msgs::Led led;

  switch (state_)
  {
  case GOING_FORWARD:  // first state of the state machine. Kobuki turn off all the leds and go forward
    cmd.linear.x = LINEAR_SPEED;
    led.value = LED_APAGADO;
    pub_led1_.publish(led);
    if ((!laser_detected_) || (pressed_))
    {
      press_ts_ = ros::Time::now();
      state_ = GOING_BACK;
      ROS_INFO("GOING_FORWARD -> GOING_BACK");
    }
    break;

  case GOING_BACK:  // second state of the state machine. Kobuki go backwards
    cmd.linear.x = -LINEAR_SPEED;
    if (((ros::Time::now() - press_ts_).toSec() > BACKING_TIME) || !security_laser_detected_)
    {
      turn_ts_ = ros::Time::now();
      if((left_bumper_pressed_) || (left_laser_detected_))
      {
        state_ = TURNING_RIGHT;
        ROS_INFO("GOING_BACK -> TURNING_RIGTH");
      }
      else
      {
        state_ = TURNING_LEFT;
        ROS_INFO("GOING_BACK -> TURNING_LEFT");
      }
    }
    break;

  case TURNING_LEFT:  // third state of the state machine. Kobuki turn on the left led and turn left
    cmd.angular.z = TURNING_SPEED;
    led.value = LED_ROJO; 
    pub_led1_.publish(led);
    if ((ros::Time::now()-turn_ts_).toSec() > TURNING_TIME )
    {
      state_ = GOING_FORWARD;
      ROS_INFO("TURNING_LEFT -> GOING_FORWARD");
      led.value = LED_APAGADO;
      pub_led1_.publish(led);
    }
    break;

  case TURNING_RIGHT:  // fourth state of the state machine. Kobuki turn on the rigth led and turn rigth
    cmd.angular.z = - TURNING_SPEED;
    led.value = LED_ROJO; 
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
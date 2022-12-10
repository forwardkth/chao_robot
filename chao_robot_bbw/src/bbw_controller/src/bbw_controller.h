// File name: bbw_controller.h
// Created on: Dec 01, 2021
// Author: Chao Li, chao.li.arthur@gmail.com
// ros package: bbw_controller
// ros node: bbw_controller_node
// Description:
// this is the gateway on beaglebone to receive cmd from jetson and control
// the executors 

#ifndef WIFI_ROBOT_SUB_H_
#define WIFI_ROBOT_SUB_H_

#include <stdio.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> //necessary to include
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/Twist.h>
#include "../../BlackLib/AdafruitBBIOServo/AdafruitBBIOServo.h"
#include "../../BlackLib/AdafruitBBIOLib/bbio.h"

using namespace std;
using namespace adafruit::bbio;
using adafruit::bbio::lib_options;
using adafruit::bbio::Gpio;
using adafruit::bbio::Pwm;
using adafruit::bbio::BBIOError;

//class WifiRobotSub

class WifiRobotSub {
 public:
  explicit WifiRobotSub(const ros::NodeHandle& nhl,
                        // robot motor init
                        const Gpio& GPIO1_12,
                        const Gpio& GPIO1_13,
                        const Gpio& GPIO1_14,
                        const Gpio& GPIO1_15,
                        // robot pwm init
                        const Pwm& PWM1A,
                        const Pwm& PWM1B,
                        float pwm_duty_cycle,
                        float linear_x,
                        float angular_z);

  ~WifiRobotSub();

  //Subscriber callback functions
  void wifiRobotCmdCallback(const geometry_msgs::Twist::ConstPtr& msg);

 private:
  ros::NodeHandle hl_;
  ros::NodeHandle nhlocal_;  

  // robot motor init
  Gpio GPIO1_12_;
  Gpio GPIO1_13_;
  Gpio GPIO1_14_;
  Gpio GPIO1_15_;
  Pwm pwm_motor_a_;
  Pwm pwm_motor_b_;
  float pwm_duty_cycle_;
  float linear_x_;
  float angular_z_;

  // ROS subscribers
  ros::Subscriber WifiRobotCmdSub_;
};

#endif // WIFI_ROBOT_SUB_H_

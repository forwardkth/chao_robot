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
                        // int servoxy_angle,  // degree
                        // int servoz_angle,   // degree
                        // robot PLZ init
                        // const string& PWM1B,
                        // const string& PWM2B,
                        // const AdafruitBBIOServo& servoXY, //P9_16
                        // const AdafruitBBIOServo& servoZ,  //p8_13
                        // robot motor init
                        const Gpio& GPIO1_12,
                        const Gpio& GPIO1_13,
                        const Gpio& GPIO1_14,
                        const Gpio& GPIO1_15);

  ~WifiRobotSub();

  //callback function
  void wifiRobotCmdCallback(const geometry_msgs::Twist::ConstPtr& msg);

 private:
  ros::NodeHandle hl_;
  ros::NodeHandle nhlocal_;  // http://wiki.ros.org/roscpp_tutorials/Tutorials/AccessingPrivateNamesWithNodeHandle
  // int servoxy_angle_;  // degree
  // int servoz_angle_;   // degree
  // robot PLZ init
  // string PWM1B_;
  // string PWM2B_;
  // AdafruitBBIOServo servoXY_; //P9_16
  // AdafruitBBIOServo servoZ_;  //p8_13

  // robot motor init
  Gpio GPIO1_12_;
  Gpio GPIO1_13_;
  Gpio GPIO1_14_;
  Gpio GPIO1_15_;
  //tf2_ros::Buffer tf_buffer_;
  //tf2_ros::TransformListener tf_listener_;
  ros::Subscriber WifiRobotCmdSub_;
};

#endif // WIFI_ROBOT_SUB_H_

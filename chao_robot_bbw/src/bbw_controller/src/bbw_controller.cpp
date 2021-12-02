// File name: bbw_controller.cpp
// Created on: Dec 01, 2021
// Author: Chao Li, chao.li.arthur@gmail.com
// ros package: bbw_controller
// ros node: bbw_controller_node
// Description:
// this is the gateway on beaglebone to receive cmd from jetson and control
// the executors 
#include "bbw_controller.h"

using namespace std;
using namespace adafruit::bbio;
using adafruit::bbio::lib_options;
using adafruit::bbio::Gpio;
using adafruit::bbio::Pwm;
using adafruit::bbio::BBIOError;

WifiRobotSub::WifiRobotSub(
  const ros::NodeHandle& nhl,
  // robot motor init
  const Gpio& GPIO1_12,
  const Gpio& GPIO1_13,
  const Gpio& GPIO1_14,
  const Gpio& GPIO1_15)
    : hl_(nhl),
      nhlocal_("~"),
      // robot motor init
      GPIO1_12_(GPIO1_12), // p8_12
      GPIO1_13_(GPIO1_13), // p8_11
      GPIO1_14_(GPIO1_14), // p8_16
      GPIO1_15_(GPIO1_15) // p8_15
{
  WifiRobotCmdSub_ = hl_.subscribe("/key_vel", 1, &WifiRobotSub::wifiRobotCmdCallback, this);
}


WifiRobotSub::~WifiRobotSub()
{
}

// control cmd callback
void WifiRobotSub::wifiRobotCmdCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  auto x_linear = msg->linear.x;
  auto z_angular = msg->angular.z;
  
  if (x_linear != 0.0 || z_angular != 0.0) 
  {
    if(x_linear > 0.0) // motor forward
    {
      // printf("%s\n","Go forward");
      GPIO1_12_.set_value(Gpio::Value::High);
      GPIO1_15_.set_value(Gpio::Value::High);
      ros::Duration(0.01).sleep();
    }
    else if(x_linear < 0.0) // motor backward
    {
      // printf("%s\n","Go backward");
      GPIO1_14_.set_value(Gpio::Value::High);
      GPIO1_13_.set_value(Gpio::Value::High);
      ros::Duration(0.01).sleep();    
    }
    
    if(z_angular > 0.0) // // motor turn left
    {
      // printf("%s\n","turn left");
      GPIO1_14_.set_value(Gpio::Value::High);
      GPIO1_12_.set_value(Gpio::Value::High);
      ros::Duration(0.01).sleep();
    }
    else if(z_angular < 0.0) // // motor turn right
    {
      // printf("%s\n","turn right");
      GPIO1_13_.set_value(Gpio::Value::High);
      GPIO1_15_.set_value(Gpio::Value::High);
      ros::Duration(0.01).sleep();    
    }
  } 
  else //motor stop 
  {
    // printf("%s\n","Brake to Stop");
    GPIO1_13_.set_value(Gpio::Value::Low);
    GPIO1_12_.set_value(Gpio::Value::Low);
    GPIO1_14_.set_value(Gpio::Value::Low);
    GPIO1_15_.set_value(Gpio::Value::Low);
    ros::Duration(0.01).sleep();
  }
}

// Main function

int main(int argc, char *argv[])
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "bbw_controller_node");
  
  // Init BBIO
  adafruit::bbio::init({LOG_DEBUG, nullptr, LOG_PERROR});
  
  // robot PLZ init
  // string PWM1B("P9_16");
  // string PWM2B("P8_13");
  // AdafruitBBIOServo servoXY(PWM1B); //P9_16
  // AdafruitBBIOServo servoZ(PWM2B);  //p8_13
  // servoXY.write_angle(servoxy_angle);
  // servoZ.write_angle(servoz_angle);
  // servoXY.ReleasePWM();
  // servoZ.ReleasePWM();

  // robot motor init
  Gpio GPIO1_12("P8_12", Gpio::Direction::Output);
  Gpio GPIO1_13("P8_11", Gpio::Direction::Output);
  Gpio GPIO1_14("P8_16", Gpio::Direction::Output);
  Gpio GPIO1_15("P8_15", Gpio::Direction::Output);

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle nh;

  WifiRobotSub wifi_robot_sub(nh,
                              // robot motor init
                              GPIO1_12,
                              GPIO1_13,
                              GPIO1_14,
                              GPIO1_15);
  printf("%s\n","bbw_controller ROS node start running.........");
  ros::spin();
  return 0;
}

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
  const Gpio& GPIO1_15,
  const Pwm& PWM1A,
  const Pwm& PWM1B,
  float pwm_duty_cycle,
  float linear_x,
  float angular_z)
    : hl_(nhl),
      nhlocal_("~"),
      // robot motor init
      GPIO1_12_(GPIO1_12), // p8_12
      GPIO1_13_(GPIO1_13), // p8_11
      GPIO1_14_(GPIO1_14), // p8_16
      GPIO1_15_(GPIO1_15), // p8_15
      pwm_motor_a_(PWM1A),  // P9_14
      pwm_motor_b_(PWM1B),   // P9_16
      pwm_duty_cycle_(pwm_duty_cycle),
      linear_x_(linear_x),
      angular_z_(angular_z)
{
  WifiRobotCmdSub_ = hl_.subscribe("/key_vel", 1, &WifiRobotSub::wifiRobotCmdCallback, this);
  pwm_motor_a_.start(0., 1e9 / (1 * 1000 * 1000), Pwm::Polarity::Normal);
  pwm_motor_b_.start(0., 1e9 / (1 * 1000 * 1000), Pwm::Polarity::Normal);
}


WifiRobotSub::~WifiRobotSub()
{
}

// control cmd callback
void WifiRobotSub::wifiRobotCmdCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  pwm_duty_cycle_ = msg->linear.z; // use linear z to transmit the pwm duty cycle value
  linear_x_ = msg->linear.x;
  angular_z_ = msg->angular.z;
  float sleep_duration = 0.01;
  
  if (linear_x_ != 0.0 || angular_z_ != 0.0) 
  {
    if(linear_x_ > 0.0) // motor forward
    {
      // printf("%s\n","Go forward");
      pwm_motor_a_.set_duty_cycle(pwm_duty_cycle_);
      pwm_motor_b_.set_duty_cycle(pwm_duty_cycle_);
      GPIO1_12_.set_value(Gpio::Value::High);
      GPIO1_15_.set_value(Gpio::Value::High);
      ros::Duration(sleep_duration).sleep();
    }
    else if(linear_x_ < 0.0) // motor backward
    {
      // printf("%s\n","Go backward");
      pwm_motor_a_.set_duty_cycle(pwm_duty_cycle_);
      pwm_motor_b_.set_duty_cycle(pwm_duty_cycle_);
      GPIO1_14_.set_value(Gpio::Value::High);
      GPIO1_13_.set_value(Gpio::Value::High);
      ros::Duration(sleep_duration).sleep();    
    }
    
    if(angular_z_ > 0.0) // // motor turn left
    {
      // printf("%s\n","turn left");
      pwm_motor_a_.set_duty_cycle(pwm_duty_cycle_);
      pwm_motor_b_.set_duty_cycle(pwm_duty_cycle_);
      GPIO1_14_.set_value(Gpio::Value::High);
      GPIO1_12_.set_value(Gpio::Value::High);
      ros::Duration(sleep_duration).sleep();
    }
    else if(angular_z_ < 0.0) // // motor turn right
    {
      // printf("%s\n","turn right");
      pwm_motor_a_.set_duty_cycle(pwm_duty_cycle_);
      pwm_motor_b_.set_duty_cycle(pwm_duty_cycle_);
      GPIO1_13_.set_value(Gpio::Value::High);
      GPIO1_15_.set_value(Gpio::Value::High);
      ros::Duration(sleep_duration).sleep();    
    }
  } 
  else //motor stop 
  {
    // pwm_motor_a.set_duty_cycle(pwm_duty_cycle_);
    // pwm_motor_b.set_duty_cycle(pwm_duty_cycle_);
    // printf("%s\n","Brake to Stop");
    GPIO1_13_.set_value(Gpio::Value::Low);
    GPIO1_12_.set_value(Gpio::Value::Low);
    GPIO1_14_.set_value(Gpio::Value::Low);
    GPIO1_15_.set_value(Gpio::Value::Low);
    ros::Duration(sleep_duration).sleep();
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

  // robot motor init
  Gpio GPIO1_12("P8_12", Gpio::Direction::Output);
  Gpio GPIO1_13("P8_11", Gpio::Direction::Output);
  Gpio GPIO1_14("P8_16", Gpio::Direction::Output);
  Gpio GPIO1_15("P8_15", Gpio::Direction::Output);

  // PWN init
  Pwm pwm_1a("P9_14");
  Pwm pwm_1b("P9_16");
  float pwm_duty_cycle = 30.0;
  float linear_x = 0.0;
  float angular_z = 0.0;
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
                              GPIO1_15,
                              pwm_1a,
                              pwm_1b,
                              pwm_duty_cycle,
                              linear_x,
                              angular_z);

  printf("%s\n","bbw_controller ROS node is started.........");
  ros::spin();
  return 0;
}

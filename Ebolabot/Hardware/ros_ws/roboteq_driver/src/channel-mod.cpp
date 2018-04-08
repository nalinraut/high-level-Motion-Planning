/**
Software License Agreement (BSD)

  - modified by ks & ba @ hstartech
  - added method to write params from a configuration callback

\file      channel.cpp
\authors   Mike Purvis <mpurvis@clearpathrobotics.com>
\copyright Copyright (c) 2013, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the following
   disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
   disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote products
   derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "roboteq_driver/channel.h"
#include "roboteq_driver/controller.h"
#include <math.h>
#include "ros/ros.h"
#include "roboteq_msgs/Feedback.h"
#include "roboteq_msgs/Command.h"
#include "roboteq_msgs/Configuration.h"

//Begin Mods
#include <roboteq_driver/MotConfig.h>
#include <dynamic_reconfigure/server.h>
#include "serial/serial.h"
// Simple conversions between rad/s and RPM.
#define TO_RPM(x) (double(x) * 60 / (2 * M_PI))
#define FROM_RPM(x) (double(x) * (2 * M_PI) / 60)

namespace roboteq {

void Channel::writeParam( const char * param ) {
  //std::cout << "\t[write]: " << param << config_map[param] << std::endl;
  controller_->param << param << config_map[param] << controller_->send;
  controller_->flush();
}
void Channel::writeAllParams() {
  for ( config_map_t::iterator it = config_map.begin(); it != config_map.end(); it ++ ) {
    //ROS_INFO( "writing config param [%s] to driver", it->first.c_str() );
    writeParam( it->first.c_str() );
  }
}
void Channel::queryParam( const char * param ) {
  controller_->configQuery << param << controller_->send;
  controller_->flush();
}
void Channel::queryAllParams() {
  for ( config_map_t::iterator it = config_map.begin(); it != config_map.end(); it ++ ) {
    ROS_INFO( "writing config param [%s] to driver", it->first.c_str() );
    queryParam( it->first.c_str() );
  }
}
void Channel::setConfigMapFromConfig( const roboteq_msgs::Configuration & config ) {
  config_map["KP"] = round( config.pid_kp * 10 );
  config_map["KI"] = round( config.pid_ki * 10 );
  config_map["KD"] = round( config.pid_kd * 10 );
  config_map["ALIM"] = round( config.current_limit * 10 );
  config_map["ICAP"] = config.int_cap;
  config_map["OVL"] = config.ovl * 10;
  config_map["UVL"] = config.uvl * 10;
  config_map["MMOD"] = config.control_mode;
  //config_map["CLERD"] = config.err_detection;
  config_map["MAC"] = round( config.motor_accel * 10 );
  config_map["MDEC"] = round( config.motor_decel * 10 );
  config_map["MXRPM"] = config.max_rpm;
  config_map["BPOL"] = config.pole_pairs;
}

void Channel::sendEStop( bool state ) {
  controller_->command << ( state ? "EX" : "MG" ) << controller_->send;
  controller_->flush();
  ROS_WARN( "e stop: [%s]", state ? "pressed" : "released" );
}

void Channel::enableMotors( bool state ) {
  ROS_WARN( "[%s] motors", state ? "enable" : "disable" );
}

void Channel::saveEeprom() {
  ROS_INFO( "Saving to EEPROM." );
  controller_->command << "EES" << controller_->send;
  controller_->flush(); 
}

Channel::Channel(int channel_num, std::string ns, Controller* controller) :
  channel_num_(channel_num), nh_(ns), controller_(controller), max_rpm_(3500)
{
  sub_cmd_ = nh_.subscribe("cmd", 1, &Channel::cmdCallback, this);
  
  pub_feedback_ = nh_.advertise<roboteq_msgs::Feedback>("feedback", 1);

  timer_init_ = nh_.createTimer(ros::Duration(1.0), &Channel::timerCallback, this);

  //sub_cfg = nh_.subscribe("/roboteq_driver_config/motor_config", 1, &Channel::cfgCallback, this);
  sub_cfg = nh_.subscribe("cfg", 1, &Channel::cfgCallback, this);
}

void Channel::cmdCallback(const roboteq_msgs::Command& command) {
  // First convert the user's command from rad/s to RPM.
  float commanded_rpm = TO_RPM(command.commanded_velocity);

  // Now get the -1000 .. 1000 command as a proportion of the maximum RPM.
  int roboteq_command = int((commanded_rpm / max_rpm_) * 1000.0);
  //ROS_INFO_STREAM("Sending command value of " << roboteq_command << " to motor driver.");

  // Write the command.
  controller_->command << "G" << channel_num_ << roboteq_command << controller_->send;
  controller_->flush();
}

//Callback for configuration subscriber
//Called whenever message published on dynamic reconfiguration topic
void Channel::cfgCallback(const roboteq_msgs::Configuration& config) {

  // disable motors 
  // controller_->command << "G" << 1 << 100 << controller_->send;
  // controller_->flush();

  //ROS_INFO("Stop: %i, Release: %i", E_Stop, E_Release);
  if ( config.e_stop ) sendEStop( true );
  if ( config.e_release ) sendEStop( false );

  setConfigMapFromConfig( config );
  writeAllParams();
  saveEeprom();
  //queryAllParams();
}



void Channel::feedbackCallback(std::vector<std::string> fields) {
  roboteq_msgs::Feedback msg;
  msg.header.stamp = last_feedback_time_ = ros::Time::now();

  try {
    msg.motor_current = boost::lexical_cast<float>(fields[2]) / 10;
    msg.commanded_velocity = FROM_RPM(boost::lexical_cast<double>(fields[3]));
    //msg.motor_power = boost::lexical_cast<float>(fields[4]) / 1000.0;
    msg.measured_position_hall = boost::lexical_cast<float>(fields[4]);
    msg.measured_velocity = FROM_RPM(boost::lexical_cast<double>(fields[5]));
    //msg.measured_position = boost::lexical_cast<double>(fields[6]) * 2 * M_PI / 4096;
    msg.measured_position = boost::lexical_cast<double>(fields[6]);
    msg.supply_voltage = boost::lexical_cast<float>(fields[7]) / 10.0;
    msg.supply_current = boost::lexical_cast<float>(fields[8]) / 10.0;
    msg.motor_temperature = boost::lexical_cast<int>(fields[9]) * 0.020153 - 4.1754;
    msg.channel_temperature = boost::lexical_cast<int>(fields[10]);
  } catch (std::bad_cast& e) {
    ROS_WARN("Failure parsing feedback data. Dropping message.");
    return;
  }
  pub_feedback_.publish(msg);
}

void Channel::timerCallback(const ros::TimerEvent&) {
  if (ros::Time::now() - last_feedback_time_ > ros::Duration(1.0)) {
    // Not receiving feedback, attempt to start it.
    controller_->setUserBool(1, 1);
    //controller_->setUserBool(2, 1);  // uncomment if using dual-channel
    controller_->flush();
  }
}

} // namespace


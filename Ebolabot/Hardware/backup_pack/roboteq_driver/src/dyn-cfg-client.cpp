#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <roboteq_driver/MotConfig.h>

#include "roboteq_driver/controller.h"
#include "roboteq_driver/channel.h"

#include "roboteq_msgs/Configuration.h"

ros::Publisher pub_config;

void callback(roboteq_driver::MotConfig &config, uint32_t level) {

  //Declare Configuration Object
  roboteq_msgs::Configuration SetConfig;

  //Pull Variables From Configuration
  SetConfig.pid_kp = config.pid_kp;
  SetConfig.pid_ki = config.pid_ki;
  SetConfig.pid_kd = config.pid_kd;
  SetConfig.int_cap = config.int_cap;
  SetConfig.error_detection = config.error_detection;
  SetConfig.control_mode = config.control_mode;
  SetConfig.current_limit = config.current_limit;
  SetConfig.pole_pairs = config.pole_pairs;
  SetConfig.ovl = config.ovl; 
  SetConfig.uvl = config.uvl; 
  SetConfig.motor_accel = config.motor_accel; 
  SetConfig.motor_decel = config.motor_decel; 
  SetConfig.max_rpm = config.max_rpm; 
  SetConfig.save_eeprom = config.save_eeprom; 
  SetConfig.e_stop = config.e_stop; 
  SetConfig.e_release = config.e_release; 

  /*//Reset Save Variable
  if(Save) { config.Save_EEPROM = false; }
  //Emergency Stop Logic
  // E_Stop Holds until reset
  if(E_Release) { config.Stop_Release = false; }
  if(E_Stop) { config.Emergency_Stop = false; }*/

  //Publish configuration
  pub_config.publish(SetConfig);
  ros::spinOnce();
}



int main(int argc, char **argv) {

  ros::init(argc, argv, "roboteq_driver_config");
  ros::NodeHandle n;

  //Setup Publisher for configuration updates
  pub_config = n.advertise<roboteq_msgs::Configuration>("/roboteq_driver_config/motor_config", 1);
  dynamic_reconfigure::Server<roboteq_driver::MotConfig>::CallbackType g;
  g  = boost::bind(&callback, _1, _2);

  dynamic_reconfigure::Server<roboteq_driver::MotConfig> srv;
  srv.setCallback(g);

  ROS_INFO("Spinning...");
  ros::spin();
  return 0;
}

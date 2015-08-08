#include <string>
#include <boost/asio/io_service.hpp>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <hotwheels_hardware/hotwheels_hardware.h>
#include <rosserial_server/serial_session.h>

void controlThread(ros::Rate rate, hotwheels_hardware::hotwheelsHardware* robot, controller_manager::ControllerManager* cm)
{
  ros::Time last_time;

  while(true)
  {
    ros::Time this_time = ros::Time::now();
    robot->read();
    cm->update(this_time, this_time - last_time);
    robot->write();
    last_time = this_time;
    rate.sleep();
  }
}

int main(int argc, char* argv[])
{
  // Initialize ROS node.
  ros::init(argc, argv, "hotwheels_node");
  
  // Fetch parameters from server
/*  std::string port;
  ros::param::param<std::string>("~port", port, "/dev/ttyACM0");
  int baud_rate;
  ros::param::param<int>("~baud", baud_rate, 115200);
 */ 
  int node_rate;
  ros::param::param<int>("~rate", node_rate, 50);

  // Create robot_hw
  hotwheels_hardware::hotwheelsHardware robot_hw;

  // Create the serial rosserial server in a background ASIO event loop.
 /* boost::asio::io_service io_service;
  new rosserial_server::SerialSession(io_service, port, baud_rate);
  boost::thread(boost::bind(&boost::asio::io_service::run, &io_service));
*/
  // Background thread for the controls callback.
  ros::NodeHandle controller_nh("");
  controller_manager::ControllerManager cm(&robot_hw, controller_nh);
  boost::thread(boost::bind(controlThread, ros::Rate(node_rate), &robot_hw, &cm));

  // Foreground ROS spinner for ROS callbacks, including rosserial, diagnostics
  ros::spin();

  return 0;
}

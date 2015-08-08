#ifndef hotwheels_HARDWARE_H
#define hotwheels_HARDWARE_H

#include <ros/ros.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_publisher.h>
#include <boost/thread.hpp>
#include <dynamic_reconfigure/server.h>

// messages from-to tiva
#include <hotwheels_msgs/Control.h>
#include <hotwheels_msgs/Feedback.h>

// local
#include <hotwheels_hardware/pwm_converter.h>


namespace hotwheels_hardware
{

class hotwheelsHardware : public hardware_interface::RobotHW
{
public:
  hotwheelsHardware();
  void read();
  void write();

private:
  void headFeedbackCallback(const hotwheels_msgs::Feedback::ConstPtr& msg);
  void bodyFeedbackCallback(const hotwheels_msgs::Feedback::ConstPtr& msg);

  ros::NodeHandle nh_;
  ros::Subscriber body_feedback_sub_;
  realtime_tools::RealtimePublisher<hotwheels_msgs::Control> base_cmd_pub_;

  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::VelocityJointInterface velocity_joint_interface_;
  hardware_interface::PositionJointInterface position_joint_interface_;

  // These are mutated on the controls thread only.
  struct Joint
  {
    double act_pos;
    double act_vel;
    double ref_pos;
    double ref_vel;

    Joint() : act_pos(0), act_vel(0), ref_pos(0), ref_vel(0)
    {
    }
  }
  // wheel joints THEN pos joints
  joints_[9];

  // This pointer is set from the ROS thread.

  hotwheels_msgs::Feedback::ConstPtr body_feedback_msg_;
  boost::mutex body_feedback_msg_mutex_;

  const double dummy_effort = 0.;

  // multipliers to convert from joint to PWM 
  std::vector<PWMConverter> pwm_converters_;

};

}  // namespace hotwheels_hardware

#endif  // hotwheels_HARDWARE_H

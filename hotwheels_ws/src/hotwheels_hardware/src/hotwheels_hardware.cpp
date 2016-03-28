#include <boost/assign.hpp>
#include <hotwheels_hardware/hotwheels_hardware.h>

namespace hotwheels_hardware
{

hotwheelsHardware::hotwheelsHardware()
{
  ros::V_string vel_joint_names = boost::assign::list_of("front_left_wheel_joint")
      ("front_right_wheel_joint")("rear_left_wheel_joint")("rear_right_wheel_joint");
 

  if(nh_.hasParam("motor_parameters"))
  {
    for(size_t i=0; i<vel_joint_names.size(); ++i)
    {
      std::vector<double> params;
      if(nh_.hasParam("motor_parameters/" + vel_joint_names[i]))
      {
        nh_.getParam("motor_parameters/" + vel_joint_names[i], params);
        // values in params mean: jnt min, jnt max, pwm min, pwm max, pwm center
        PWMConverter converter(params[0], params[1], params[2], params[3], params[4], params[5]);
        ROS_ASSERT(params[5] == 1 or params[5] == -1);
        pwm_converters_.push_back(converter);
      }
      else
      {
        ROS_FATAL_STREAM(vel_joint_names[i] << " is missing from motor_parameters!");
      }
    }
  }
  else
  {
      ROS_FATAL("Didn't get motor_parameters!");
  }


  for (size_t i = 0; i < vel_joint_names.size(); ++i)
  {
    hardware_interface::JointStateHandle joint_state_handle(vel_joint_names[i],
        &joints_[i].act_pos, &joints_[i].act_vel, &dummy_effort);
    joint_state_interface_.registerHandle(joint_state_handle);

    hardware_interface::JointHandle joint_handle(
        joint_state_handle, &joints_[i].ref_vel);
    velocity_joint_interface_.registerHandle(joint_handle);
  }
  

  registerInterface(&joint_state_interface_);
  registerInterface(&velocity_joint_interface_);


  body_feedback_sub_ = nh_.subscribe("tiva_body/feedback", 1, &hotwheelsHardware::bodyFeedbackCallback, this);

  // Realtime publisher, initializes differently from regular ros::Publisher

  base_cmd_pub_.init(nh_, "tiva/control_msg", 1);


}

/**
 * Populates the internal joint state struct from the most recent Feedback message
 * Called from the controller thread.
 */
void hotwheelsHardware::read()
{
  boost::mutex::scoped_lock body_feedback_msg_lock(body_feedback_msg_mutex_, boost::try_to_lock);
  if(body_feedback_msg_ and body_feedback_msg_lock)
  {
    // wheel velocity feedback
    for(int i = 0; i < 4; i++)
    {
      //joints_[i].act_pos = ?
      joints_[i].act_vel = body_feedback_msg_->act_vel[i];
    }
  
  }

}

/**
 * Populates and publishes Control message based on the controller outputs.
 *
 * Called from the controller thread.
 */
void hotwheelsHardware::write()
{
  if(base_cmd_pub_.trylock())
  {
    // TODO: hardcoding number of joints here is kinda fine but still ugly as hell
    for(size_t i=0; i<4; ++i)
      base_cmd_pub_.msg_.vel_commands[i] = pwm_converters_[i](joints_[i].ref_vel);
    // TODO: magic number 4 = number of velocity joints

    boost::mutex::scoped_lock body_feedback_msg_lock(body_feedback_msg_mutex_, boost::try_to_lock);
    if(body_feedback_msg_ and body_feedback_msg_lock)
    {
      
    }

    base_cmd_pub_.unlockAndPublish();
  }

  
}


void hotwheelsHardware::bodyFeedbackCallback(const hotwheels_msgs::Feedback::ConstPtr& msg)
{
  // Update the feedback message pointer to point to the current message. Block
  // until the control thread is not using the lock.
  boost::mutex::scoped_lock lock(body_feedback_msg_mutex_);
  body_feedback_msg_ = msg;
}


}  // namespace hotwheels_hardware


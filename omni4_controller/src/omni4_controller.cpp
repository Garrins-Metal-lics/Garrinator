#include "omni4_controller.h"

namespace omni4_controller
{

bool Omni4Controller::init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
	/*
	// for each joint, get name and sets joint handle
	std::string joint_name;
    if (!n.getParam("rim_front_left_joint", joint_name))
    {
        ROS_ERROR("Could not find joint name");
        return false;
    }
    joint_front_left_ = hw->getHandle(joint_name);  // throws on failure

	//initializes kinematics parae


    // Load gain using gains set on parameter server
    if (!n.getParam("gain", gain_))
    {
        ROS_ERROR("Could not find the gain parameter value");
        return false;
    }

    // Start command subscriber
    sub_command_ = n.subscribe<std_msgs::Float64>("command", 1, &Omni4Controller::setCommandCB, this);
*/
    return true;
}

void Omni4Controller::update(const ros::Time& time, const ros::Duration& period)
{
	/*
    double error = command_ - joint_.getPosition();
    double commanded_effort = error*gain_;
    joint_.setCommand(commanded_effort);
	*/
}

void Omni4Controller::starting(const ros::Time& time)
{

}

void Omni4Controller::stopping(const ros::Time& time)
{

}

}  // end of namespace

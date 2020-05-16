#include "omni4_controller.h"

namespace omni4_controller
{

bool Omni4Controller::init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
	// for each joint, get name and sets joint handle
	std::string joint_name;
    if (!controller_nh.getParam("rim_front_left_joint", joint_name))
    {
        ROS_ERROR("Could not find joint name");
        return false;
    }
    joint_front_left_ = hw->getHandle(joint_name);  // throws on failure
	// TODO for the other joints


	//initializes kinematics parameters: constants required to solve the kinematics equations
	// TODO

    // Init twist subscriber
	twist_subscriber_ = controller_nh.subscribe("cmd_vel", 1, &Omni4Controller::commandTwistCallback, this);

	// returns
    return true;
}

void Omni4Controller::update(const ros::Time& time, const ros::Duration& period)
{
	Eigen::Vector3d twist;
	double w_fl, w_fr, w_bl, w_br; //joint (wheel) speeds [rad/s]

	// get twist from the buffer
	twist = *(command_buffer_.readFromRT());

	//Compute the inverse kinematics to obtain joint speeds
	//TODO

	// Set joint speeds
	joint_front_left_.setCommand(w_fl);
	//TODO for the other wheels

}

void Omni4Controller::starting(const ros::Time& time)
{
	// set zero to all joints
	joint_front_left_.setCommand(0);
	// TODO for the other joints

	// init RT command buffer
	command_buffer_.initRT( Eigen::Vector3d(0,0,0) );
}

void Omni4Controller::stopping(const ros::Time& time)
{
	// set zero to all joints
	joint_front_left_.setCommand(0);
	// TODO for the other joints
}

void Omni4Controller::commandTwistCallback(const geometry_msgs::Twist& _twist)
{
	// sets the incoming twist to command buffer
	command_buffer_.writeFromNonRT( Eigen::Vector3d(_twist.linear.x,_twist.linear.y,_twist.angular.z) );
}

}  // end of namespace

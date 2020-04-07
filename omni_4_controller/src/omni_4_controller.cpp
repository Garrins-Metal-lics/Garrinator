#include <controller_interface/controller.h>
#include <pluginlib/class_list_macros.h>
#include <hardware_interface/joint_command_interface.h>
#include <std_msgs/Float64.h>

namespace omni_4_controller
{
  class Omni4Controller : public controller_interface::Controller<hardware_interface::VelocityJointInterface>
  {

    bool init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle &n)
        {
            std::string front_left_rim;
            if (!n.getParam("rim_front_left_joint", front_left_rim))
            {
                ROS_ERROR("Could not find joint name");
                return false;
            }
            joint_ = hw->getHandle(front_left_rim);  // throws on failure
            command_ = joint_.getPosition();   // set the current joint goal to the current joint position

            // Load gain using gains set on parameter server
            if (!n.getParam("gain", gain_))
            {
                ROS_ERROR("Could not find the gain parameter value");
                return false;
            }

            // Start command subscriber
            sub_command_ = n.subscribe<std_msgs::Float64>("command", 1, &Omni4Controller::setCommandCB, this);

            return true;
        }

        void update(const ros::Time& time, const ros::Duration& period)
        {
            double error = command_ - joint_.getPosition();
            double commanded_effort = error*gain_;
            joint_.setCommand(commanded_effort);
        }

        void setCommandCB(const std_msgs::Float64ConstPtr& msg)
        {
            command_ = msg->data;
        }

        void starting(const ros::Time& time) { }
        void stopping(const ros::Time& time) { }


        private:
            hardware_interface::JointHandle joint_;
            double gain_;
            double command_;
            ros::Subscriber sub_command_;
  };

PLUGINLIB_EXPORT_CLASS(omni_4_controller::Omni4Controller, controller_interface::ControllerBase);
}

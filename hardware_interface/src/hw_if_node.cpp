#include "odrive_hw_if.h"
#include <controller_manager/controller_manager.h>
int main(int argc, char **argv)
{
	ros::init(argc, argv, "odrive_hw_iface");
	ros::start();
	ros::NodeHandle nh;

	ros::AsyncSpinner spinner(1);
	spinner.start();

	garrinator_hardware_interface::OdriveHwIf odrive;
	ros::Time ts, prev_ts;
	ros::Duration ds;

	if (!odrive.init(nh,nh))
	{
		std::cout << "Error at init(). EXIT" << '\n';
		return -1;
	}

	controller_manager::ControllerManager cm(&odrive,nh);

	prev_ts=ros::Time::now();

	int freq=10;//Hz of update rate
  	for (size_t i = 0; i < 300*freq; i++)
	{
    	for (size_t j = 0; j < odrive.velocities_cmmd_.size(); j++)
		{
        odrive.velocities_cmmd_[j]=4;
    }
		ts=ros::Time::now();
		ds=ts-prev_ts;
		odrive.read(ts,ds);
		cm.update(ts, ds);
		odrive.write(ts,ds);
		prev_ts=ts;
		odrive.print();
		sleep(1/freq);
	}

	odrive.read(ts,ds);
	odrive.print();

  	return 1;
}

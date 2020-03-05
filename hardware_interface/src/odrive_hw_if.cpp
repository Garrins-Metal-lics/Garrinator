#include "odrive_hw_if.h"
// blocking read of a single byte

OdriveHwIf::OdriveHwIf()
{
  positions_fb_.resize(4);
  velocities_fb_.resize(4);
  efforts_fb_.resize(4);
  velocities_cmmd_.resize(4);
}


OdriveHwIf::~OdriveHwIf()
{
  // stop wheels

  //stop motor 0 of each motor driver
  mensg_="v 0 0 \n";
  ::write(serial_id_od1,mensg_.c_str(),mensg_.size());// send the message
  ::write(serial_id_od2,mensg_.c_str(),mensg_.size());// send the message

  //stop motor 1 of each motor driver
  mensg_="v 1 0 \n";
  ::write(serial_id_od1,mensg_.c_str(),mensg_.size());// send the message
  ::write(serial_id_od2,mensg_.c_str(),mensg_.size());// send the message

	sleep(1);

  // close serials
  close(serial_id_od1);
  close(serial_id_od2);
  //close comms
  //tcsetattr( STDIN_FILENO, TCSANOW, &stdInOldSettings );
}


bool OdriveHwIf::init(ros::NodeHandle& _root_nh,ros::NodeHandle& _robot_hw_nh)
{
  // open serials
  //open serial 1
  serial_id_od1 = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_NONBLOCK);
	if ( serial_id_od1 < 0 )
	{
	        std::cout << "Error opening serial port: " << "/dev/ttyACM0" << std::endl;
	        return false;
	}

  //open serial 2
  serial_id_od2 = open("/dev/ttyACM1", O_RDWR | O_NOCTTY | O_NONBLOCK);
  if ( serial_id_od2 < 0 )
  {
          std::cout << "Error opening serial port: " << "/dev/ttyACM1" << std::endl;
          return false;
  }
	//2. configures serial comm's

  //configure comm 1
	tcgetattr(serial_id_od1, &ttySettings_1);//gets current config
	ttySettings_1.c_cflag = ( B115200 | CLOCAL | CREAD | CS8 );
	ttySettings_1.c_iflag = ( IGNBRK ); //Ignores break condition on input
	ttySettings_1.c_oflag = 0x0;
	ttySettings_1.c_lflag = 0x0;
	ret_value = tcsetattr(serial_id_od1, TCSANOW, &ttySettings_1); //Sets configuration immediately.
	if ( ret_value < 0 )
	{
	        std::cout << "Error configuring serial communications" << std::endl << std::endl;
	        return false;
	}
  //configure comm 2

  tcgetattr(serial_id_od2, &ttySettings_2);//gets current config
  ttySettings_2.c_cflag = ( B115200 | CLOCAL | CREAD | CS8 );
  ttySettings_2.c_iflag = ( IGNBRK ); //Ignores break condition on input
  ttySettings_2.c_oflag = 0x0;
  ttySettings_2.c_lflag = 0x0;
  ret_value = tcsetattr(serial_id_od2, TCSANOW, &ttySettings_2); //Sets configuration immediately.
  if ( ret_value < 0 )
  {
          std::cout << "Error configuring serial communications" << std::endl << std::endl;
          return false;
  }


	//3. configure stdin
	//Important! ( please see http://www.cplusplus.com/forum/general/5304/ )
	/*tcgetattr( STDIN_FILENO, &stdInOldSettings );
	stdInNewSettings = stdInOldSettings;
	stdInNewSettings.c_lflag &= (~ICANON & ~ECHO);
	tcsetattr( STDIN_FILENO, TCSANOW, &stdInNewSettings );*/


	//X4
	hardware_interface::JointStateHandle wheel_1_s_handle("rim_front_left_joint",&positions_fb_[0],&velocities_fb_[0],&efforts_fb_[0]);
	state_joint_interface_.registerHandle(wheel_1_s_handle);

	hardware_interface::JointHandle wheel_1_v_handle(state_joint_interface_.getHandle("rim_front_left_joint"),&velocities_cmmd_[0]);
	velocity_joint_interface_.registerHandle(wheel_1_v_handle);


	registerInterface(&state_joint_interface_);
	registerInterface(&velocity_joint_interface_);

	return true;

}


void OdriveHwIf::read(const ros::Time& _time,const ros::Duration& _period )
{
	float counts_to_rads=2*M_PI/2048;// factor that changes counts to rad/s

	//-- read first odrive
	// read first wheel
  mensg_="f 0 \n";
  ::write(serial_id_od1,mensg_.c_str(),mensg_.size());// send the message
  read_msg_.erase();
  do {
    byte_=(readByte(serial_id_od1));
    read_msg_.push_back(byte_);

  } while(byte_!='\n');

	// read_msg_ have position and speed of wheel 1
	delimiter_ = " ";
	token_ = read_msg_.substr(0, read_msg_.find(delimiter_));
	read_msg_.erase(0, read_msg_.find(delimiter_)+delimiter_.length());

	// finished parse string
	//upload information
	to_rad_=std::atof(token_.c_str())*counts_to_rads;
	to_rad_=(to_rad_ / (2*M_PI));
	to_rad_=(to_rad_-trunc(to_rad_))*(2*M_PI);
	positions_fb_[0]=to_rad_;

	velocities_fb_[0]=std::atof(read_msg_.c_str())*counts_to_rads;

	//--
	// read second wheel
  mensg_="f 1 \n";
  ::write(serial_id_od1,mensg_.c_str(),mensg_.size());// send the message
  read_msg_.erase();
  do {
    byte_=(readByte(serial_id_od1));
    read_msg_.push_back(byte_);

  } while(byte_!='\n');

	// read_msg_ have position and speed of wheel 1
	delimiter_ = " ";
	token_ = read_msg_.substr(0, read_msg_.find(delimiter_));
	read_msg_.erase(0, read_msg_.find(delimiter_)+delimiter_.length());

	// finished parse string
	//upload information

	to_rad_=std::atof(token_.c_str())*counts_to_rads;
	to_rad_=(to_rad_ / (2*M_PI));
	to_rad_=(to_rad_-trunc(to_rad_))*(2*M_PI);
	positions_fb_[1]=to_rad_;
	velocities_fb_[1]=std::atof(read_msg_.c_str())*counts_to_rads;


	//-- read second odrive
	// read first wheel
  mensg_="f 0 \n";
  ::write(serial_id_od2,mensg_.c_str(),mensg_.size());// send the message
  read_msg_.erase();
  do {
    byte_=(readByte(serial_id_od2));
    read_msg_.push_back(byte_);

  } while(byte_!='\n');

	// read_msg_ have position and speed of wheel 1
	delimiter_ = " ";
	token_ = read_msg_.substr(0, read_msg_.find(delimiter_));
	read_msg_.erase(0, read_msg_.find(delimiter_)+delimiter_.length());

	// finished parse string
	//upload information
	to_rad_=std::atof(token_.c_str())*counts_to_rads;
	to_rad_=(to_rad_ / (2*M_PI));
	to_rad_=(to_rad_-trunc(to_rad_))*(2*M_PI);
	positions_fb_[2]=to_rad_;
	velocities_fb_[2]=std::atof(read_msg_.c_str())*counts_to_rads;

	//--
	// read second wheel
  mensg_="f 1 \n";
  ::write(serial_id_od2,mensg_.c_str(),mensg_.size());// send the message
  read_msg_.erase();
  do {
    byte_=(readByte(serial_id_od2));
    read_msg_.push_back(byte_);

  } while(byte_!='\n');

	// read_msg_ have position and speed of wheel 1
	delimiter_ = " ";
	token_ = read_msg_.substr(0, read_msg_.find(delimiter_));
	read_msg_.erase(0, read_msg_.find(delimiter_)+delimiter_.length());

	// finished parse string
	//upload information
	to_rad_=std::atof(token_.c_str())*counts_to_rads;
	to_rad_=(to_rad_ / (2*M_PI));
	to_rad_=(to_rad_-trunc(to_rad_))*(2*M_PI);
	positions_fb_[3]=to_rad_;
	velocities_fb_[3]=std::atof(read_msg_.c_str())*counts_to_rads;}


void OdriveHwIf::write(const ros::Time& _time,const ros::Duration& _period )
{
	float rad_to_count=(2048)/(2*M_PI);// factor that changes rad/s to counts

	//--write first odrive
	//write first wheel speed

	mensg_="v 0 "+ std::to_string(velocities_cmmd_[0]*rad_to_count)+" \n";
  ::write(serial_id_od1,mensg_.c_str(),mensg_.size());// send the message

	//second wheel update
	mensg_="v 1 "+ std::to_string(velocities_cmmd_[1]*rad_to_count)+" \n";
	::write(serial_id_od1,mensg_.c_str(),mensg_.size());// send the message

	//--write second odrive
	//write first wheel speed
	mensg_="v 0 "+ std::to_string(velocities_cmmd_[2]*rad_to_count)+" \n";
	::write(serial_id_od2,mensg_.c_str(),mensg_.size());// send the message

	//second wheel update
	mensg_="v 1 "+ std::to_string(velocities_cmmd_[3]*rad_to_count)+" \n";
	::write(serial_id_od2,mensg_.c_str(),mensg_.size());// send the message

}


unsigned char OdriveHwIf::readByte(const int & _serial_id)
{
	unsigned char byte;
	fd_set fdSet;

	//reconfigure fdSet for device entry
	FD_ZERO (&fdSet);
	FD_SET (_serial_id, &fdSet);

	//blocking read, up to an input arrives to some channel at fdSet
	select (_serial_id+1, &fdSet,NULL,NULL,NULL);

	//reads 1 byte from device and return it
	::read(_serial_id,&byte,1);
	return byte;
}

void OdriveHwIf::print() const
{
	std::cout  <<"------------------------" <<std::endl;
	for (size_t i = 0; i < 4; i++) {
		std::cout << "wheel_" <<i<< " position: "<<positions_fb_[i]<<" rad"<<std::endl;
		std::cout << "wheel_" <<i<< " velocity: "<<velocities_fb_[i]<<" rad/s"<<std::endl<<std::endl;
	}

}

//PLUGINLIB_EXPORT_CLASS(OdriveHwIf,hardware_interface::RobotHW)

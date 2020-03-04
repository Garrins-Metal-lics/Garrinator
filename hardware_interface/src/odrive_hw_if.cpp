#include "odrive_hw_if.h"
// blocking read of a single byte
unsigned char readByte(int __serial_id)
{
	unsigned char byte;
	fd_set fdSet;

	//reconfigure fdSet for device entry
	FD_ZERO (&fdSet);
	FD_SET (__serial_id, &fdSet);

	//blocking read, up to an input arrives to some channel at fdSet
	select (__serial_id+1, &fdSet,NULL,NULL,NULL);

	//reads 1 byte from device and return it
	read(__serial_id,&byte,1);
	return byte;
}


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


  // close serials
  close(serial_id_od1);
  close(serial_id_od2);
  //close comms
  tcsetattr( STDIN_FILENO, TCSANOW, &stdInOldSettings );
}


bool OdriveHwIf::init(ros::NodeHandle& _root_nh,ros::NodeHandle& _robot_hw_nh)
{
  // open serials
  //open serial 1
  serial_id_od1 = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_NONBLOCK);
	if ( serial_id_od1 < 0 )
	{
	        std::cout << "Error opening serial port: " << "/dev/ttyACM0" << std::endl;
	        return -1;
	}

  //open serial 2
  serial_id_od2 = open("/dev/ttyACM1", O_RDWR | O_NOCTTY | O_NONBLOCK);
  if ( serial_id_od2 < 0 )
  {
          std::cout << "Error opening serial port: " << "/dev/ttyACM1" << std::endl;
          return -1;
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
	        return -1;
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
          return -1;
  }


	//3. configure stdin
	//Important! ( please see http://www.cplusplus.com/forum/general/5304/ )
	tcgetattr( STDIN_FILENO, &stdInOldSettings );
	stdInNewSettings = stdInOldSettings;
	stdInNewSettings.c_lflag &= (~ICANON & ~ECHO);
	tcsetattr( STDIN_FILENO, TCSANOW, &stdInNewSettings );


}


void OdriveHwIf::read(const ros::Time& _time,const ros::Duration& _period )
{
	int counts_to_rads=0;// factor that changes counts to rad/s

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
	delimiter_ = "\t";
	token_ = read_msg_.substr(0, read_msg_.find(delimiter_));
	read_msg_.erase(0, read_msg_.find(delimiter_)+delimiter_.length());

	// finished parse string
	//upload information
	positions_fb_[1]=std::atof(token_.c_str())*counts_to_rads;
	velocities_fb_[1]=std::atof(read_msg_.c_str())*counts_to_rads;

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
	delimiter_ = "\t";
	token_ = read_msg_.substr(0, read_msg_.find(delimiter_));
	read_msg_.erase(0, read_msg_.find(delimiter_)+delimiter_.length());

	// finished parse string
	//upload information
	positions_fb_[2]=std::atof(token_.c_str())*counts_to_rads;
	velocities_fb_[2]=std::atof(read_msg_.c_str())*counts_to_rads;


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
	delimiter_ = "\t";
	token_ = read_msg_.substr(0, read_msg_.find(delimiter_));
	read_msg_.erase(0, read_msg_.find(delimiter_)+delimiter_.length());

	// finished parse string
	//upload information
	positions_fb_[3]=std::atof(token_.c_str())*counts_to_rads;
	velocities_fb_[3]=std::atof(read_msg_.c_str())*counts_to_rads;

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
	delimiter_ = "\t";
	token_ = read_msg_.substr(0, read_msg_.find(delimiter_));
	read_msg_.erase(0, read_msg_.find(delimiter_)+delimiter_.length());

	// finished parse string
	//upload information
	positions_fb_[4]=std::atof(token_.c_str())*counts_to_rads;
	velocities_fb_[4]=std::atof(read_msg_.c_str())*counts_to_rads;
}


void OdriveHwIf::write(const ros::Time& _time,const ros::Duration& _period )
{
	int rad_to_count=0;// factor that changes rad/s to counts

	//--write first odrive
	//write first wheel speed

	mensg_="v 0 "+ std::to_string(velocities_cmmd_[1]*rad_to_count)+" \n";
  ::write(serial_id_od1,mensg_.c_str(),mensg_.size());// send the message

	//second wheel update
	mensg_="v 1 "+ std::to_string(velocities_cmmd_[2]*rad_to_count)+" \n";
	::write(serial_id_od1,mensg_.c_str(),mensg_.size());// send the message

	//--write second odrive
	//write first wheel speed
	mensg_="v 0 "+ std::to_string(velocities_cmmd_[3]*rad_to_count)+" \n";
	::write(serial_id_od2,mensg_.c_str(),mensg_.size());// send the message

	//second wheel update
	mensg_="v 1 "+ std::to_string(velocities_cmmd_[4]*rad_to_count)+" \n";
	::write(serial_id_od2,mensg_.c_str(),mensg_.size());// send the message

}

// includes
#include <iostream>
#include <termios.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <string>

#include <math.h>

//--- new changes
#include <vector>


// constants
const unsigned int BUFFER_SIZE = 4; // 4 bytes, two per encoder: [D0_MSB,D0_LSB,D1_MSB,D1_LSB]
const std::string portName = "/dev/ttyACM0";
const double PI  =3.141592653589793238463;

std::vector<double> positions_fb_(4);
std::vector<double> velocities_fb_(4);
std::vector<double> velocities_cmmd_(4);

std::string read_msg_;
std::string token_;
std::string delimiter_;
std::string mensg_;// variable that will provide flexibility to the messages
unsigned char byte_;


int serial_id_od1, serial_id_od2;
int ret_value;
termios ttySettings_1, ttySettings_2; //termios variable to configure serial port
termios stdInOldSettings, stdInNewSettings;


double to_rad_;




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

void read(void) {
	float counts_to_rads=2*PI/2048;// factor that changes counts to rad/s

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
	to_rad_=(to_rad_ / (2*PI));
	to_rad_=(to_rad_-trunc(to_rad_))*(2*PI);
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
	to_rad_=(to_rad_ / (2*PI));
	to_rad_=(to_rad_-trunc(to_rad_))*(2*PI);
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
	to_rad_=(to_rad_ / (2*PI));
	to_rad_=(to_rad_-trunc(to_rad_))*(2*PI);
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
	to_rad_=(to_rad_ / (2*PI));
	to_rad_=(to_rad_-trunc(to_rad_))*(2*PI);
	positions_fb_[3]=to_rad_;
	velocities_fb_[3]=std::atof(read_msg_.c_str())*counts_to_rads;
}

void write(void){
	float rad_to_count=(2048)/(2*PI);// factor that changes rad/s to counts

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

int init(void)
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


int main()
{
	unsigned int ii; //byte counter
	init();
	//relax to allow devices to be ready
	sleep(0.5);

	ii = 0;//reset byte counter

	for (size_t i = 0; i < 4; i++) {
		velocities_cmmd_[i]=10;
	}

	write();
	//3. main loop. Syncs and read data
	while (1)
	{
		//send speed
		ii+=1;
		std::cout << "iteration nº: "<< ii<< std::endl;

		read();

		sleep(1);
		std::cout << "Vectors status" << '\n';
		for (size_t i = 0; i < 4; i++) {
			std::cout << "wheel_" <<i<< " position: "<<positions_fb_[i]<<" rad"<<'\n';
			std::cout << "wheel_" <<i<< " velocity: "<<velocities_fb_[i]<<" rad/s"<<'\n';
		}


	}

	//closes serial comms and restores old settings
	close(serial_id_od1);
	close(serial_id_od2);
	tcsetattr( STDIN_FILENO, TCSANOW, &stdInOldSettings );
	return 0;
}

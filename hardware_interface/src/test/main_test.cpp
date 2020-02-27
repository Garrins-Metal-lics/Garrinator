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



// constants
const unsigned int BUFFER_SIZE = 4; // 4 bytes, two per encoder: [D0_MSB,D0_LSB,D1_MSB,D1_LSB]
const std::string portName = "/dev/ttyACM0";

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


int main()
{
	int ret_value, serial_id;
	unsigned int ii; //byte counter
	unsigned char byte; //device input
	unsigned char buffer[BUFFER_SIZE];
	termios ttySettings; //termios variable to configure serial port
	termios stdInOldSettings, stdInNewSettings;
	unsigned short d0,d1;

	//1. opens serial port. read() will not block
	serial_id = open(portName.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
	if ( serial_id < 0 )
	{
	        std::cout << "Error opening serial port: " << portName << std::endl;
	        return -1;
	}

	//2. configures serial comm's
	tcgetattr(serial_id, &ttySettings);//gets current config
	ttySettings.c_cflag = ( B115200 | CLOCAL | CREAD | CS8 );
	ttySettings.c_iflag = ( IGNBRK ); //Ignores break condition on input
	ttySettings.c_oflag = 0x0;
	ttySettings.c_lflag = 0x0;
	ret_value = tcsetattr(serial_id, TCSANOW, &ttySettings); //Sets configuration immediately.
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

	//relax to allow devices to be ready
	sleep(1);

	ii = 0;//reset byte counter

	std::string mensg;
	float vel=12343;
	std::string read_msg;

	unsigned char caracter;
	std::string mystring;

	//3. main loop. Syncs and read data
	while (1)
	{
		//send speed
		std::cout << "enter speed: \n";
		std::cin >> mystring;
		mensg="v 0 "+ mystring+" \n";
		std::cout << mensg;
    write(serial_id,mensg.c_str(),mensg.size());// send the message

		//feedback
		mensg="f 0 \n";
		write(serial_id,mensg.c_str(),mensg.size());// send the message

		read_msg.erase();
		do {
			caracter=(readByte(serial_id));
			read_msg.push_back(caracter);

		} while(caracter!='\n');
		std::cout << read_msg << std::endl<< std::endl;

/*
		//check for sync
		if (readByte(serial_id) == 0xFF)
		{
			//keep reading for data and store it in the buffer
			for(ii=0; ii<BUFFER_SIZE; ii++) buffer[ii] = readByte(serial_id);

			// when buffer is full, decode data
			d0 = 0;
			d1 = 0;
			d0 = buffer[1] | buffer[0]<<6;
			d1 = buffer[3] | buffer[2]<<6;

			// print data into the screen
			std::cout << "encoder 0: " << d0 << std::endl;
			std::cout << "encoder 1: " << d1 << std::endl << std::endl;
		}*/
		sleep(1);


	}

	//closes serial comms and restores old settings
	close(serial_id);
	tcsetattr( STDIN_FILENO, TCSANOW, &stdInOldSettings );
	return 0;
}

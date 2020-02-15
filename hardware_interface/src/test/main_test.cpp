// C library headers
#include <stdio.h>
#include <string.h>
#include <math.h>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

unsigned char transcriptor(int);
int main()
{
  // open serial port
  int serial_port;
  serial_port = open("/dev/ttyACM0",O_RDWR | O_NOCTTY);
	if (serial_port < 0) {
	    printf("Error %i from open: %s\n", errno, strerror(errno));
	}
  else{
    printf("Acces permited! \n" );
  }



  // Create new termios struc, we call it 'tty' for convention
  struct termios tty;
  memset(&tty, 0, sizeof tty);

  // Read in existing settings, and handle any error
  if(tcgetattr(serial_port, &tty) != 0) {
      printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
  }

  // configure the termios
  tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
  tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
  tty.c_cflag |= CS8; // 8 bits per byte (most common)
  tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
  tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)
  tty.c_lflag &= ~ICANON; // desable canonical mode
  tty.c_lflag &= ~ECHO; // Disable echo
  tty.c_lflag &= ~ECHOE; // Disable erasure
  tty.c_lflag &= ~ECHONL; // Disable new-line echo
  tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
  tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
  tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
  tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
  tty.c_cc[VMIN] = 0;

  // Set in/out baud rate to be 9600 other option is 115200
  cfsetispeed(&tty, B9600);
  cfsetospeed(&tty, B9600);

  // Save tty settings, also checking for error
if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
    printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
}

/* ASCII comand structure:
v 0 1000 0

v: for velocity
motor: is the motor number, 0 or 1.
velocity: is the desired velocity in counts/s.
current_ff: is the current feed-forward term, in A (optional)

So: v 0 1000 0
velocity comand; motor 0; 1000 counts/s; (optional)


counts/s to RPM:

counts per revolution= 2048
1min=60s

n rpms = (n*2048)/60 counts/s


*/
// ==== all tries works only once ===

// 1st try (works)
// Write to serial port
//unsigned char msg[] = { 'v', ' ', '0', ' ', '1','0','0','0','0', '\n' };

// 2nd try (works)
unsigned char mensg[] = { "v 0 10000\r\n" };


// 3nd try ( not work, other fault not communication)
//int rpm =60;//number of rpms that the motor should go
//----float vel;// parameter that will inject the velocity
//----vel = transcriptor(rpm); // function to transcrip from rpm to cpp/s
//                                and add it in a array in ASCII format
//unsigned char mensg[] = { 'v', ' ', '0', ' ', vel, '\n' };


/*======================================================
                  ISSUE:
Some times it works some times doesn't. Mainly it sends the first message (mensg)
and the motor starts to run, but the second message (mensg2) doesn't stop them.

To know if it was a comand issue i've installed arduino and oppened a port (serial monitor)
and from there i was able to send the commands and recieve a response in case they are
wrong, both commands work in arduino ide but not here, so I'm supossing that it is
an issue of the communication.
======================================================*/

//== Don't know why this one works this time but next dont--> "v 0 10000\r\n"
printf("%s\n",mensg );// debug, show the message before send it
write(serial_port, mensg, sizeof(mensg));// send the message

  char read_buf [256];
  memset(&read_buf, '\0', sizeof(read_buf));

  // Read bytes. The behaviour of read() (e.g. does it block?,
  // how long does it block for?) depends on the configuration
  // settings above, specifically VMIN and VTIME
  int num_bytes = read(serial_port, &read_buf, sizeof(read_buf));

  // n is the number of bytes read. n may be 0 if no bytes were received, and can also be -1 to signal an error.
  if (num_bytes < 0) {
      printf("Error reading: %s", strerror(errno));
  }

  // Here we assume we received ASCII data, but you might be sending raw bytes (in that case, don't try and
  // print it to the screen like this!)
  printf("Read %i bytes. Received message: %s \n", num_bytes, read_buf);

  long time=2000;
  usleep(time);


  unsigned char mensg2 [] = { "v 0 0\r\n" };

  printf("%s\n",mensg2 );
  write(serial_port, mensg2, sizeof(mensg));

    memset(&read_buf, '\0', sizeof(read_buf));

    // Read bytes. The behaviour of read() (e.g. does it block?,
    // how long does it block for?) depends on the configuration
    // settings above, specifically VMIN and VTIME
    num_bytes = read(serial_port, &read_buf, sizeof(read_buf));

    // n is the number of bytes read. n may be 0 if no bytes were received, and can also be -1 to signal an error.
    if (num_bytes < 0) {
        printf("Error reading: %s", strerror(errno));
    }

    // Here we assume we received ASCII data, but you might be sending raw bytes (in that case, don't try and
    // print it to the screen like this!)
    printf("Read %i bytes. Received message: %s \n", num_bytes, read_buf);
















  close(serial_port);
	return 0;
}




unsigned char transcriptor(int rpm){
float cpr;// factor of counts per revolution
cpr=2048;// number of counts per revolution
float vel;// result variable
vel=(rpm*cpr)/60;// compute the rev/s through  counts/s
printf("vel fin: : %f\n",vel );
int x;
int counter;
counter=0;
x=vel;
  while ( x > 0 )
  {
    x= x/10;
    counter++;
  }
  printf("counter: : %i\n",counter );

  printf("velocity: %f\n",vel );
  unsigned char v[counter];
  int i=counter ;            // index
  int p=0;
  int var;
  int power;// exponential variable
  unsigned char test;

while ( vel > 0 )
{
  power=(counter-p-1);
  var=vel;//var = 2048
  var=var/(pow(10,power)); //var =2048/1000  =2,048
  var=trunc(var) ;//var =2
  test = (unsigned char)(var+48);
  v[i] = var+48;// added to the array in ascii 2+48=50 --> ascii =2
  printf("vel: %f power: %i var: %i test: %i \n",vel,power,var,test );
  vel-=var*pow(10,power);// 2048- 2000
  i-- ;
  p++;
  printf("vector: %s\n",v );
}

  printf(" vector: %s\n",v );

  return test;

}

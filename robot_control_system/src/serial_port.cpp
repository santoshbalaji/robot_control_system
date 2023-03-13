#include <robot_control_system/serial_port.hpp>

using SerialPort = robot_control_system::SerialPort;

SerialPort::SerialPort(string port)
{
  this->port_ = port;
}

/**
 * Function for connecting with serial controller
*/
int SerialPort::connect()
{
  this->file_descriptor_ = open(this->port_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  if (this->file_descriptor_ == -1)
  {
    std::cout << "Error: Unable to open serial port." << std::endl;
    return -1;
  }

  struct termios options;
  tcgetattr(this->file_descriptor_, &options);
  options.c_cflag = B9600 | CS8 | CLOCAL | CREAD;
  options.c_iflag = IGNPAR;
  options.c_oflag = 0;
  options.c_lflag = 0;
  tcflush(this->file_descriptor_, TCIFLUSH);
  tcsetattr(this->file_descriptor_, TCSANOW, &options);

  return 0;
}

/**
 * Function to read from serial controllers
*/
int SerialPort::read_from_serial(int bytes_to_read, char * output)
{
  int bytes_read = read(this->file_descriptor_, output, bytes_to_read);
  if (bytes_read < 0) 
  {
    std::cout << "Error: Unable to read from serial port." << std::endl;
    return -1;
  }
  return 0;
}
 
/**
 * Function to write on serial controllers
*/
int SerialPort::write_to_serial(const char * data)
{
  int bytes_written = write(this->file_descriptor_, data, sizeof(data));
  if (bytes_written < 0) 
  {
    std::cout << "Error: Unable to write to serial port." << std::endl;
    return -1;
  }
  return 0;
}

/**
 * Function to disconnect from serial controller
*/
void SerialPort::disconenct()
{
  if(this->file_descriptor_ != -1)
  {
    close(this->file_descriptor_);
  }
}
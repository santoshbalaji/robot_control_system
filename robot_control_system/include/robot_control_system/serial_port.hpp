#ifndef ROBOT_CONTROL_SYSTEM__SERIAL_PORT_HPP_
#define ROBOT_CONTROL_SYSTEM__SERIAL_PORT_HPP_

#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <thread>

#include <iostream>

using namespace std;

namespace robot_control_system
{

class SerialPort
{
public:
  SerialPort(string port);
  int connect();
  int read_from_serial(int bytes_to_read, char * output);
  int write_to_serial(const char * data);
  void disconenct();

private:
  string port_;
  int file_descriptor_;
};

} // namespace robot_control_system

#endif  // ROBOT_CONTROL_SYSTEM__SERIAL_PORT_HPP_
#ifndef ROBOT_CONTROL_SYSTEM__SERIAL_PORT_HPP_
#define ROBOT_CONTROL_SYSTEM__SERIAL_PORT_HPP_

#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>

namespace robot_control_system
{

class SerialPort
{
public:
  SerialPort();
  void connect();
  void read();
  void write();
  void disconenct();

private:

};

} // namespace robot_control_system

#endif  // ROBOT_CONTROL_SYSTEM__SERIAL_PORT_HPP_
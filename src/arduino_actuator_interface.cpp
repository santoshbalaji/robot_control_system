#include <robot_control_system/arduino_actuator_interface.hpp>

CallbackReturn ArduinoActuatorInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{

}

CallbackReturn ArduinoActuatorInterface::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{

}


std::vector<hardware_interface::StateInterface> ArduinoActuatorInterface::export_state_interfaces()
{

}

std::vector<hardware_interface::CommandInterface> ArduinoActuatorInterface::export_command_interfaces()
{

}

CallbackReturn ArduinoActuatorInterface::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{

}

CallbackReturn ArduinoActuatorInterface::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{

}

hardware_interface::return_type ArduinoActuatorInterface::read()
{

}

hardware_interface::return_type ArduinoActuatorInterface::write()
{

}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ArduinoActuatorInterface, hardware_interface::SystemInterface)

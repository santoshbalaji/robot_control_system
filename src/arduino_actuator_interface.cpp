#include <robot_control_system/arduino_actuator_interface.hpp>

CallbackReturn ArduinoActuatorInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  return CallbackReturn::SUCCESS;
}

CallbackReturn ArduinoActuatorInterface::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ArduinoActuatorInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ArduinoActuatorInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interafaces;
  return command_interafaces;
}

CallbackReturn ArduinoActuatorInterface::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  return CallbackReturn::SUCCESS;
}

CallbackReturn ArduinoActuatorInterface::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type ArduinoActuatorInterface::read()
{
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ArduinoActuatorInterface::write()
{
  return hardware_interface::return_type::OK;
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ArduinoActuatorInterface, hardware_interface::SystemInterface)

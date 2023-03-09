#include <robot_control_system/arduino_actuator_interface.hpp>

namespace robot_control_system
{

CallbackReturn ArduinoActuatorInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  hw_start_sec_ = stod(info_.hardware_parameters["hw_start_duration_sec"]);
  hw_stop_sec_ = stod(info_.hardware_parameters["hw_stop_duration_sec"]);
  hw_slowdown_ = stod(info_.hardware_parameters["hw_slowdown"]);

  hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("arduino_actuator_interface"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("arduino_actuator_interface"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("arduino_actuator_interface"),
        "Joint '%s' has %zu state interface. 1 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("arduino_actuator_interface"),
        "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn ArduinoActuatorInterface::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(
    rclcpp::get_logger("arduino_actuator_interface"), "configuring hardware .......");

  for (int i = 0; i < hw_start_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("arduino_actuator_interface"), "%.1f seconds left in configuring",
      hw_start_sec_ - i);
  }

  for (uint i = 0; i < hw_states_.size(); i++)
  {
    hw_states_[i] = 0;
    hw_commands_[i] = 0;
  }

  RCLCPP_INFO(rclcpp::get_logger("arduino_actuator_interface"), "successfully configured");

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ArduinoActuatorInterface::export_state_interfaces()
{
  RCLCPP_INFO(
    rclcpp::get_logger("arduino_actuator_interface"), "exporting state interfaces");

  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ArduinoActuatorInterface::export_command_interfaces()
{
  RCLCPP_INFO(
    rclcpp::get_logger("arduino_actuator_interface"), "exporting command interfaces");

  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }

  return command_interfaces;
}

CallbackReturn ArduinoActuatorInterface::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(
    rclcpp::get_logger("arduino_actuator_interface"), "activating now");

  for (int i = 0; i < hw_start_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("arduino_actuator_interface"), "%.1f seconds left in activating",
      hw_start_sec_ - i);
  }

  for (uint i = 0; i < hw_states_.size(); i++)
  {
    hw_commands_[i] = hw_states_[i];
  }

  this->serial_port = new SerialPort("/dev/ttyACM0");
  this->serial_port->connect();

  RCLCPP_INFO(rclcpp::get_logger("arduino_actuator_interface"), "successfully activated");

  return CallbackReturn::SUCCESS;
}

CallbackReturn ArduinoActuatorInterface::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(
    rclcpp::get_logger("arduino_actuator_interface"), "deactivating now");

  for (int i = 0; i < hw_stop_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("arduino_actuator_interface"), "%.1f seconds left in deactivating",
      hw_stop_sec_ - i);
  }

  this->serial_port->disconenct();

  RCLCPP_INFO(rclcpp::get_logger("arduino_actuator_interface"), "successfully deactivated");

  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type ArduinoActuatorInterface::read()
{
  RCLCPP_INFO(rclcpp::get_logger("arduino_actuator_interface"), "Reading...");

  for (uint i = 0; i < hw_states_.size(); i++)
  {
    hw_states_[i] = hw_states_[i] + (hw_commands_[i] - hw_states_[i]) / hw_slowdown_;
    RCLCPP_INFO(
      rclcpp::get_logger("arduino_actuator_interface"), "Got state %.5f for joint %d!",
      hw_states_[i], i);
  }
  RCLCPP_INFO(rclcpp::get_logger("arduino_actuator_interface"), "Joints successfully read!");

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ArduinoActuatorInterface::write()
{
  RCLCPP_INFO(rclcpp::get_logger("arduino_actuator_interface"), "Writing...");

  string data = "ON\n";
  this->serial_port->write_to_serial(data.c_str());

  for (uint i = 0; i < hw_commands_.size(); i++)
  {
    // Simulate sending commands to the hardware
    RCLCPP_INFO(
      rclcpp::get_logger("arduino_actuator_interface"), "Got command %.5f for joint %d!",
      hw_commands_[i], i);
  }
  RCLCPP_INFO(
    rclcpp::get_logger("arduino_actuator_interface"), "Joints successfully written %d", hw_commands_.size());

  return hardware_interface::return_type::OK;
}

} // namespace robot_control_system

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  robot_control_system::ArduinoActuatorInterface, hardware_interface::SystemInterface)

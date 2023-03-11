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

  int total_states = 0, total_commands = 0;

  for (uint i = 0; i < info_.joints.size(); i++)
  {
    for (uint j = 0; j < info_.joints[i].state_interfaces.size(); j++)
    {
      total_states = total_states + 1;
    }
  }

  for (uint i = 0; i < info_.joints.size(); i++)
  {
    for (uint j = 0; j < info_.joints[i].command_interfaces.size(); j++)
    {
      total_commands = total_commands + 1;
    }
  }
  
  hw_states_.resize(total_states, std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(total_commands, std::numeric_limits<double>::quiet_NaN());

  RCLCPP_INFO(rclcpp::get_logger("test"), "Me in %d %d", total_commands, total_states);

  // for (const hardware_interface::ComponentInfo & joint : info_.joints)
  // {
  //   if (joint.command_interfaces.size() != 2)
  //   {
  //     RCLCPP_FATAL(
  //       rclcpp::get_logger("arduino_actuator_interface"),
  //       "Joint '%s' has %zu command interfaces found. 2 expected.", joint.name.c_str(),
  //       joint.command_interfaces.size());
  //     return CallbackReturn::ERROR;
  //   }

  //   if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
  //   {
  //     RCLCPP_FATAL(
  //       rclcpp::get_logger("arduino_actuator_interface"),
  //       "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
  //       joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
  //     return CallbackReturn::ERROR;
  //   }

  //   if (joint.command_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
  //   {
  //     RCLCPP_FATAL(
  //       rclcpp::get_logger("arduino_actuator_interface"),
  //       "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
  //       joint.command_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
  //     return CallbackReturn::ERROR;
  //   }

  //   if (joint.state_interfaces.size() != 2)
  //   {
  //     RCLCPP_FATAL(
  //       rclcpp::get_logger("arduino_actuator_interface"),
  //       "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
  //       joint.state_interfaces.size());
  //     return CallbackReturn::ERROR;
  //   }

  //   if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
  //   {
  //     RCLCPP_FATAL(
  //       rclcpp::get_logger("arduino_actuator_interface"),
  //       "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
  //       joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
  //     return CallbackReturn::ERROR;
  //   }

  //   if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
  //   {
  //     RCLCPP_FATAL(
  //       rclcpp::get_logger("arduino_actuator_interface"),
  //       "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
  //       joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
  //     return CallbackReturn::ERROR;
  //   }
  // }

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
  }

  for (uint i = 0; i < hw_commands_.size(); i++)
  {
    hw_commands_[i] = 0;
  }

  RCLCPP_INFO(rclcpp::get_logger("arduino_actuator_interface"), "successfully configured");

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ArduinoActuatorInterface::export_state_interfaces()
{
  RCLCPP_INFO(
    rclcpp::get_logger("arduino_actuator_interface"), "exporting state interfaces");

  int current_state = 0;
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    for (uint j = 0; j < info_.joints[i].state_interfaces.size(); j++)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, info_.joints[i].state_interfaces[j].name, &hw_states_[current_state]));
      current_state = current_state + 1;
    }
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ArduinoActuatorInterface::export_command_interfaces()
{
  RCLCPP_INFO(
    rclcpp::get_logger("arduino_actuator_interface"), "exporting command interfaces");

  int current_command = 0;
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    for (uint j = 0; j < info_.joints[i].command_interfaces.size(); j++)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, info_.joints[i].command_interfaces[j].name, &hw_commands_[current_command]));
      current_command = current_command + 1;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("test"), "Me out %d", command_interfaces.size());
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

hardware_interface::return_type ArduinoActuatorInterface::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  RCLCPP_INFO(rclcpp::get_logger("arduino_actuator_interface"), "Reading...");

  for (uint i = 0; i < hw_states_.size(); i++)
  {
    hw_states_[i] = hw_states_[i] + (hw_commands_[i] - hw_states_[i]) / hw_slowdown_;
    RCLCPP_INFO(rclcpp::get_logger("test"), "Me casa %d %d %lf %lf", 
      hw_states_.size(),
      hw_commands_.size(),
      hw_states_[i],
      hw_commands_[i]);
    RCLCPP_INFO(
      rclcpp::get_logger("arduino_actuator_interface"), "Got state %.5f for joint %d!",
      hw_states_[i], i);
  }
  RCLCPP_INFO(rclcpp::get_logger("arduino_actuator_interface"), "Joints successfully read");

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ArduinoActuatorInterface::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  RCLCPP_INFO(rclcpp::get_logger("arduino_actuator_interface"), "Writing...");

  string data = "ON\n";
  this->serial_port->write_to_serial(data.c_str());

  for (uint i = 0; i < hw_commands_.size(); i++)
  {
    RCLCPP_INFO(
      rclcpp::get_logger("arduino_actuator_interface"), "Got command %.5f for joint %d!",
      hw_commands_[i], i);
  }
  RCLCPP_INFO(
    rclcpp::get_logger("arduino_actuator_interface"), "Joints successfully written");

  return hardware_interface::return_type::OK;
}

} // namespace robot_control_system

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  robot_control_system::ArduinoActuatorInterface, hardware_interface::SystemInterface)

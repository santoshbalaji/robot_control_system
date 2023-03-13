#include <robot_control_system/arduino_actuator_interface.hpp>

namespace robot_control_system
{

// Different states in interfaces
// Unconfigured, Inactive, Active, Finalized


/**
 * Function which gets invoked at starting of lifecycle in controllers
*/
CallbackReturn ArduinoActuatorInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  try
  {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
      return CallbackReturn::ERROR;
    }

    hw_start_sec_ = stod(info_.hardware_parameters["hw_start_duration_sec"]);
    hw_stop_sec_ = stod(info_.hardware_parameters["hw_stop_duration_sec"]);
    hw_slowdown_ = stod(info_.hardware_parameters["hw_slowdown"]);

    int total_states = 0, total_commands = 0;

    // get total required states and commands    
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
    encoder_readings_.resize(total_commands, std::numeric_limits<double>::quiet_NaN());
    
    for (const hardware_interface::ComponentInfo & joint : info_.joints)
    {
      // checking if required command interfaces is one
      if (joint.command_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("arduino_actuator_interface"),
          "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
          joint.command_interfaces.size());
        return CallbackReturn::ERROR;
      }

      // checking if required command interfaces is related to velocity
      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("arduino_actuator_interface"),
          "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
          joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return CallbackReturn::ERROR;
      }

      // checking if required states interfaces is two
      if (joint.state_interfaces.size() != 2)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("arduino_actuator_interface"),
          "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
          joint.state_interfaces.size());
        return CallbackReturn::ERROR;
      }

      // checking if required state interfaces is related to position
      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("arduino_actuator_interface"),
          "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
          joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return CallbackReturn::ERROR;
      }

      // checking if required state interfaces is related to velocity
      if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("arduino_actuator_interface"),
          "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
          joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return CallbackReturn::ERROR;
      }
    }
  }
  catch(std::exception &e)
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("arduino_actuator_interface"),
      "Error: %s", e.what()
    );
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

/**
 * Function to send reference of state variables to parent controller
 * These values can be read or modified by parent controller
 * Executed after on_init function
*/
std::vector<hardware_interface::StateInterface> ArduinoActuatorInterface::export_state_interfaces()
{
  RCLCPP_INFO(
    rclcpp::get_logger("arduino_actuator_interface"), "exporting state interfaces");
  std::vector<hardware_interface::StateInterface> state_interfaces;

  try
  {
    int current_state = 0;
    for (uint i = 0; i < info_.joints.size(); i++)
    {
      for (uint j = 0; j < info_.joints[i].state_interfaces.size(); j++)
      {
        // reference of states are added here for allowing of read and modification by parent controller
        state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, info_.joints[i].state_interfaces[j].name, &hw_states_[current_state]));
        current_state = current_state + 1;
      }
    }
  }
  catch(std::exception &e)
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("arduino_actuator_interface"),
      "Error: %s", e.what()
    );
  }

  return state_interfaces;
}

/**
 * Function to send reference of command variables to parent controller
 * These values can be read or modified by parent controller
 * Executed after on_init function
*/
std::vector<hardware_interface::CommandInterface> ArduinoActuatorInterface::export_command_interfaces()
{
  RCLCPP_INFO(
    rclcpp::get_logger("arduino_actuator_interface"), "exporting command interfaces");
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  try
  {
    int current_command = 0;
    for (uint i = 0; i < info_.joints.size(); i++)
    {
      for (uint j = 0; j < info_.joints[i].command_interfaces.size(); j++)
      {
        // reference of commands are added here for allowing of read and modification by parent controller
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.joints[i].name, info_.joints[i].command_interfaces[j].name, &hw_commands_[current_command]));
        current_command = current_command + 1;
      }
    }
  }
  catch(std::exception &e)
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("arduino_actuator_interface"),
      "Error: %s", e.what()
    );
  }
  return command_interfaces;
}

/**
 * Function which gets executed after export_state_interfaces and export_command_interfaces function
 * Intialize all state and command based variables here
*/
CallbackReturn ArduinoActuatorInterface::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(
    rclcpp::get_logger("arduino_actuator_interface"), "configuring hardware");
  try
  {
    for (int i = 0; i < hw_start_sec_; i++)
    {
      rclcpp::sleep_for(std::chrono::seconds(1));
      RCLCPP_INFO(
        rclcpp::get_logger("arduino_actuator_interface"), "%.1f seconds left in configuring",
        hw_start_sec_ - i);
    }

    // states are initialized
    for (uint i = 0; i < hw_states_.size(); i++)
    {
      hw_states_[i] = 0;
    }

    // commands are initialized
    for (uint i = 0; i < hw_commands_.size(); i++)
    {
      hw_commands_[i] = 0;
      encoder_readings_[i] = 0;
    }

    RCLCPP_INFO(rclcpp::get_logger("arduino_actuator_interface"), "successfully configured");
  }
  catch(std::exception &e)
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("arduino_actuator_interface"),
      "Error: %s", e.what()
    );
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

/**
 * Function which gets executed after on_configure function
 * This function gets invoked on activating the controller
*/
CallbackReturn ArduinoActuatorInterface::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(
    rclcpp::get_logger("arduino_actuator_interface"), "activating now");

  try
  {
    for (int i = 0; i < hw_start_sec_; i++)
    {
      rclcpp::sleep_for(std::chrono::seconds(1));
      RCLCPP_INFO(
        rclcpp::get_logger("arduino_actuator_interface"), "%.1f seconds left in activating",
        hw_start_sec_ - i);
    }

    // initialising and connecting to serial communication module
    this->serial_port = new SerialPort("/dev/ttyACM0");
    this->serial_port->connect();
  }
  catch(std::exception &e)
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("arduino_actuator_interface"),
      "Error: %s", e.what()
    );
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("arduino_actuator_interface"), "successfully activated");
  return CallbackReturn::SUCCESS;
}

/**
 * This function gets invoked on de-activating the controller
*/
CallbackReturn ArduinoActuatorInterface::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(
    rclcpp::get_logger("arduino_actuator_interface"), "deactivating now");

  try
  {
    for (int i = 0; i < hw_stop_sec_; i++)
    {
      rclcpp::sleep_for(std::chrono::seconds(1));
      RCLCPP_INFO(
        rclcpp::get_logger("arduino_actuator_interface"), "%.1f seconds left in deactivating",
        hw_stop_sec_ - i);
    }

    // disconnecting from serial communication module
    this->serial_port->disconenct();
  }
  catch(std::exception &e)
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("arduino_actuator_interface"),
      "Error: %s", e.what()
    );
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("arduino_actuator_interface"), "successfully deactivated");
  return CallbackReturn::SUCCESS;
}

/**
 * Function which gets invoked at every configured update frequency
 * Logic of converting sensor values to states happens here
*/
hardware_interface::return_type ArduinoActuatorInterface::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  RCLCPP_INFO(rclcpp::get_logger("arduino_actuator_interface"), "Reading");
  
  try
  {
    double radius = 0.038;
    double full_rotation_encoder_count = 873;

    char test_data[256];
    this->serial_port->read_from_serial(256, test_data);
    double current_encoder_reading = std::stod(test_data);

    double total_elapsed_count = current_encoder_reading - this->encoder_readings_[0];
    // calculating rate of change by getting the circumference of wheel and multiplying it with time interval
    // 2 * pi * radius of the wheel * (elapsed encoder count / total count needed by encoder for full rotation) * elapsed time period in hz
    double velocity = 2 * 3.14 * radius * (total_elapsed_count / full_rotation_encoder_count) * 10; //(frequency is 10 hz)

    this->encoder_readings_[0] = current_encoder_reading;

    // computing angular displacement (linear velocity / (radius * elasped time in hz))
    double radians = hw_states_[0] + (velocity / (radius * 10));
    hw_states_[0] = radians;
    hw_states_[1] = velocity;

    RCLCPP_INFO(rclcpp::get_logger("arduino_actuator_interface"), "%f %f %f %f", current_encoder_reading, total_elapsed_count, hw_states_[0], hw_states_[1]);
  }
  catch(std::exception &e)
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("arduino_actuator_interface"),
      "Error: %s", e.what()
    );
    return hardware_interface::return_type::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("arduino_actuator_interface"), "Joints successfully read");
  return hardware_interface::return_type::OK;
}

/**
 * Function which gets invoked at every configured update frequency
 * Based on commands obtained external actuators or other devices can be controlled from here
*/
hardware_interface::return_type ArduinoActuatorInterface::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  RCLCPP_INFO(rclcpp::get_logger("arduino_actuator_interface"), "Writing");

  try
  {
    // normalising threshold values (normalized_value = ((value - old_min) / (old_max - old_min)) * (new_max - new_min) + new_min)
    // 101, 0 - max, min command value to be obtained from control loop
    // 99, 15 - max, min command value to be sent for Arduino PWM control
    double threshold = ((hw_commands_[0]) / (101)) * (99 - 15) + 15;
    // Format for serial data (pwm_signal_value;forward;reverse)
    string data = std::to_string((int) threshold) + ";0;1;\n";
    this->serial_port->write_to_serial(data.c_str());
    RCLCPP_INFO(rclcpp::get_logger("arduino_actuator_interface"), "Writing %s", data.c_str());
  }
  catch(std::exception &e)
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("arduino_actuator_interface"),
      "Error: %s", e.what()
    );
    return hardware_interface::return_type::ERROR;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("arduino_actuator_interface"), "Joints successfully written %f", hw_commands_[0]);

  return hardware_interface::return_type::OK;
}

} // namespace robot_control_system

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  robot_control_system::ArduinoActuatorInterface, hardware_interface::SystemInterface)

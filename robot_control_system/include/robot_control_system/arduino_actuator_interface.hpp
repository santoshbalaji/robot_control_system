#ifndef ROBOT_CONTROL_SYSTEM__ARDUINO_ACTUATOR_INTERFACE_HPP_
#define ROBOT_CONTROL_SYSTEM__ARDUINO_ACTUATOR_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "robot_control_system/serial_port.hpp"
#include "robot_control_system/visibility_control.h"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace robot_control_system
{

class ArduinoActuatorInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ArduinoActuatorInterface)

  ROBOT_CONTROL_SYSTEM_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  ROBOT_CONTROL_SYSTEM_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  ROBOT_CONTROL_SYSTEM_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  ROBOT_CONTROL_SYSTEM_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  ROBOT_CONTROL_SYSTEM_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  ROBOT_CONTROL_SYSTEM_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  ROBOT_CONTROL_SYSTEM_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  ROBOT_CONTROL_SYSTEM_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  double hw_start_sec_;
  double hw_stop_sec_;
  double hw_slowdown_;

  std::vector<double> hw_commands_;
  std::vector<double> hw_states_;

  std::vector<double> encoder_readings_;
  std::vector<double> timings_;

  SerialPort * serial_port;
};

} // namespace robot_control_system

#endif  // ROBOT_CONTROL_SYSTEM__ARDUINO_ACTUATOR_INTERFACE_HPP_

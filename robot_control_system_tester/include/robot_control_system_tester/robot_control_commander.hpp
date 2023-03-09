#ifndef ROBOT_CONTROL_SYSTEM_TESTER__ROBOT_CONTROL_COMMANDER_HPP_
#define ROBOT_CONTROL_SYSTEM_TESTER__ROBOT_CONTROL_COMMANDER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <vector>

using std::chrono_literals::operator""ms;
using std::chrono::duration_cast;
using std::chrono::milliseconds;
using std::chrono::system_clock;

namespace robot_control_system_tester 
{
class RobotControlCommander : public rclcpp::Node
{
  public:
    RobotControlCommander(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    void timer_callback();

  private:
    rclcpp::Publisher<
      trajectory_msgs::msg::JointTrajectory>::SharedPtr robot_control_publisher_;
    rclcpp::TimerBase::SharedPtr background_process_timer_;
};
} // namespace robot_control_system_tester

#endif  // ROBOT_CONTROL_SYSTEM_TESTER__ROBOT_CONTROL_COMMANDER_HPP_
#include <robot_control_system_tester/robot_control_commander.hpp>

namespace robot_control_system_tester 
{
  RobotControlCommander::RobotControlCommander(
    const rclcpp::NodeOptions & options) : Node("robot_control_commander", options)
  {
    this->robot_control_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/position_trajectory_controller/joint_trajectory", 10);
    
    this->background_process_timer_ = this->create_wall_timer(
      10000ms, std::bind(&RobotControlCommander::timer_callback, this));
  }

  void RobotControlCommander::timer_callback()
  {
    std::vector<std::string> joint_names = {"joint2"};
    trajectory_msgs::msg::JointTrajectory joint_trajectory;
    joint_trajectory.joint_names = joint_names;
    std::vector<trajectory_msgs::msg::JointTrajectoryPoint> joint_trajectory_points;
    trajectory_msgs::msg::JointTrajectoryPoint joint_trajectory_point;
    std::vector<double> positions = {0.5};
    joint_trajectory_point.positions = positions;
    joint_trajectory_point.accelerations = positions;
    joint_trajectory_point.effort = positions;
    joint_trajectory_point.velocities = positions;
    joint_trajectory_points.push_back(joint_trajectory_point);
    joint_trajectory.points = joint_trajectory_points;

    this->robot_control_publisher_->publish(joint_trajectory);
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robot_control_system_tester::RobotControlCommander>());
  rclcpp::shutdown();
  return 0;
}
#include <robot_control_system_tester/robot_control_commander.hpp>

namespace robot_control_system_tester 
{
  RobotControlCommander::RobotControlCommander(
    const rclcpp::NodeOptions & options) : Node("robot_control_commander", options)
  {
    // topic to which joint trajectories to be passed
    this->robot_control_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/position_trajectory_controller/joint_trajectory", 10);
    
    this->background_process_timer_ = this->create_wall_timer(
      4000ms, std::bind(&RobotControlCommander::timer_callback, this));
  }

  void RobotControlCommander::timer_callback()
  {
    std::vector<std::string> joint_names = {"joint2"};
    trajectory_msgs::msg::JointTrajectory joint_trajectory;
    joint_trajectory.joint_names = joint_names;
    std::vector<trajectory_msgs::msg::JointTrajectoryPoint> joint_trajectory_points;
    trajectory_msgs::msg::JointTrajectoryPoint joint_trajectory_point;
    joint_trajectory_point.positions = {50}; // position is set for joint 2
    joint_trajectory_point.accelerations = {};
    joint_trajectory_point.effort = {};
    joint_trajectory_point.velocities = {0.2}; // velocity is set for joint 2
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
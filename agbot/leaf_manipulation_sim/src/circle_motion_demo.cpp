#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

using namespace std::chrono_literals;

namespace
{
constexpr double kRadius = 0.05;
constexpr int kSamples = 40;
constexpr double kJointPublishRate = 30.0;
constexpr double kCartesianStep = 0.01;
constexpr double kCircleFractionThreshold = 0.90;

std::vector<double> makeZeroPositions(std::size_t size)
{
  return std::vector<double>(size, 0.0);
}
}  // namespace

class CircleMotionDemo
{
public:
  explicit CircleMotionDemo(const rclcpp::Node::SharedPtr& node)
  : node_(node),
    system_clock_(RCL_SYSTEM_TIME),
    joint_state_pub_(node_->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10))
  {
  }

  bool run()
  {
    const std::vector<std::string> arm_joints = {
      "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"
    };

    publishStableJointState(arm_joints, makeZeroPositions(arm_joints.size()), 2s);

    moveit::planning_interface::MoveGroupInterface move_group(node_, "tmr_arm");
    move_group.setPoseReferenceFrame("base");
    move_group.setEndEffectorLink("gripper");
    move_group.setMaxVelocityScalingFactor(0.2);
    move_group.setMaxAccelerationScalingFactor(0.2);
    move_group.setPlanningTime(5.0);

    if (!planAndReplayNamedTarget(move_group, "ready1")) {
      return false;
    }

    rclcpp::sleep_for(500ms);
    geometry_msgs::msg::Pose start_pose = move_group.getCurrentPose("gripper").pose;
    auto trajectory = planCircle(move_group, start_pose);
    if (!trajectory) {
      return false;
    }

    RCLCPP_INFO(node_->get_logger(), "Replaying circular end-effector path");
    replayTrajectory(*trajectory);
    RCLCPP_INFO(node_->get_logger(), "Circle demo finished");
    return true;
  }

private:
  bool planAndReplayNamedTarget(
    moveit::planning_interface::MoveGroupInterface& move_group,
    const std::string& target_name)
  {
    move_group.setNamedTarget(target_name);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto result = move_group.plan(plan);
    if (result != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to plan to named target '%s'", target_name.c_str());
      return false;
    }

    RCLCPP_INFO(node_->get_logger(), "Replaying named target '%s'", target_name.c_str());
    replayTrajectory(plan.trajectory_);
    return true;
  }

  std::optional<moveit_msgs::msg::RobotTrajectory> planCircle(
    moveit::planning_interface::MoveGroupInterface& move_group,
    const geometry_msgs::msg::Pose& start_pose)
  {
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.reserve(kSamples + 1);

    for (int i = 0; i <= kSamples; ++i) {
      const double theta = (2.0 * M_PI * static_cast<double>(i)) / static_cast<double>(kSamples);
      geometry_msgs::msg::Pose waypoint = start_pose;
      waypoint.position.x = start_pose.position.x;
      waypoint.position.y = start_pose.position.y + kRadius * std::cos(theta) - kRadius;
      waypoint.position.z = start_pose.position.z + kRadius * std::sin(theta);
      waypoints.push_back(waypoint);
    }

    move_group.setStartStateToCurrentState();
    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = move_group.computeCartesianPath(waypoints, kCartesianStep, 0.0, trajectory);
    RCLCPP_INFO(
      node_->get_logger(),
      "Cartesian circle planning fraction: %.3f (%d waypoints)",
      fraction,
      static_cast<int>(waypoints.size()));

    if (fraction < kCircleFractionThreshold || trajectory.joint_trajectory.points.empty()) {
      RCLCPP_ERROR(node_->get_logger(), "Circle planning failed or incomplete");
      return std::nullopt;
    }

    return trajectory;
  }

  void replayTrajectory(const moveit_msgs::msg::RobotTrajectory& trajectory)
  {
    const auto& joint_names = trajectory.joint_trajectory.joint_names;
    const auto& points = trajectory.joint_trajectory.points;
    if (joint_names.empty() || points.empty()) {
      RCLCPP_WARN(node_->get_logger(), "Trajectory is empty, nothing to replay");
      return;
    }

    builtin_interfaces::msg::Duration last_stamp;
    for (const auto& point : points) {
      sensor_msgs::msg::JointState msg;
      msg.header.stamp = system_clock_.now();
      msg.name = joint_names;
      msg.position = point.positions;
      joint_state_pub_->publish(msg);

      const double wait_sec =
        toSeconds(point.time_from_start) - toSeconds(last_stamp);
      if (wait_sec > 0.0) {
        rclcpp::sleep_for(
          std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(wait_sec)));
      } else {
        rclcpp::sleep_for(
          std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / kJointPublishRate)));
      }
      last_stamp = point.time_from_start;
    }

    publishStableJointState(joint_names, points.back().positions, 1s);
  }

  void publishStableJointState(
    const std::vector<std::string>& names,
    const std::vector<double>& positions,
    std::chrono::milliseconds duration)
  {
    const auto end_time = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < end_time && rclcpp::ok()) {
      sensor_msgs::msg::JointState msg;
      msg.header.stamp = system_clock_.now();
      msg.name = names;
      msg.position = positions;
      joint_state_pub_->publish(msg);
      rclcpp::sleep_for(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(1.0 / kJointPublishRate)));
    }
  }

  static double toSeconds(const builtin_interfaces::msg::Duration& duration)
  {
    return static_cast<double>(duration.sec) +
      static_cast<double>(duration.nanosec) * 1e-9;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Clock system_clock_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("circle_motion_demo", options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  CircleMotionDemo demo(node);
  const bool ok = demo.run();

  rclcpp::shutdown();
  spinner.join();
  return ok ? 0 : 1;
}

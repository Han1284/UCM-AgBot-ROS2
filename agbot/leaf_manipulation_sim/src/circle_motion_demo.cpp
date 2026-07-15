#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

using namespace std::chrono_literals;

namespace
{
constexpr double kJointPublishRate = 30.0;
constexpr double kCartesianStep = 0.01;
constexpr double kCircleFractionThreshold = 0.90;
constexpr double kDefaultRadius = 0.10;
constexpr int kDefaultRepetitions = 3;
constexpr int kDefaultSamplesPerCircle = 60;

std::vector<double> makeZeroPositions(std::size_t size)
{
  return std::vector<double>(size, 0.0);
}

geometry_msgs::msg::Pose eigenToPose(const Eigen::Isometry3d& transform)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = transform.translation().x();
  pose.position.y = transform.translation().y();
  pose.position.z = transform.translation().z();

  const Eigen::Quaterniond q(transform.rotation());
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  return pose;
}

struct TrajectoryEndpoint
{
  std::vector<std::string> joint_names;
  std::vector<double> positions;
};
}  // namespace

class CircleMotionDemo
{
public:
  explicit CircleMotionDemo(const rclcpp::Node::SharedPtr& node)
  : node_(node),
    system_clock_(RCL_SYSTEM_TIME),
    joint_state_pub_(node_->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10)),
    path_pub_(node_->create_publisher<nav_msgs::msg::Path>("circle_demo_path", 10)),
    robot_model_loader_(std::make_shared<robot_model_loader::RobotModelLoader>(node_, "robot_description")),
    robot_model_(robot_model_loader_->getModel())
  {
    radius_ = getOrDeclareParameter("radius", kDefaultRadius);
    repetitions_ = getOrDeclareParameter("repetitions", kDefaultRepetitions);
    samples_per_circle_ = getOrDeclareParameter("samples_per_circle", kDefaultSamplesPerCircle);
    velocity_scale_ = getOrDeclareParameter("velocity_scale", 0.2);
    acceleration_scale_ = getOrDeclareParameter("acceleration_scale", 0.2);

    if (radius_ <= 0.0) {
      radius_ = kDefaultRadius;
    }
    if (repetitions_ < 1) {
      repetitions_ = 1;
    }
    if (samples_per_circle_ < 12) {
      samples_per_circle_ = 12;
    }
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
    move_group.setMaxVelocityScalingFactor(velocity_scale_);
    move_group.setMaxAccelerationScalingFactor(acceleration_scale_);
    move_group.setPlanningTime(5.0);

    auto start_state = planAndReplayNamedTarget(move_group, "ready1");
    if (!start_state) {
      return false;
    }

    publishStableJointState(start_state->joint_names, start_state->positions, 500ms);
    auto trajectory = planCircle(move_group, *start_state);
    if (!trajectory) {
      return false;
    }

    publishPath(*trajectory);
    RCLCPP_INFO(node_->get_logger(), "Replaying circular end-effector path");
    replayTrajectory(*trajectory);
    RCLCPP_INFO(node_->get_logger(), "Circle demo finished");
    return true;
  }

private:
  std::optional<TrajectoryEndpoint> planAndReplayNamedTarget(
    moveit::planning_interface::MoveGroupInterface& move_group,
    const std::string& target_name)
  {
    move_group.setNamedTarget(target_name);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto result = move_group.plan(plan);
    if (result != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to plan to named target '%s'", target_name.c_str());
      return std::nullopt;
    }

    RCLCPP_INFO(node_->get_logger(), "Replaying named target '%s'", target_name.c_str());
    replayTrajectory(plan.trajectory_);
    const auto& joint_names = plan.trajectory_.joint_trajectory.joint_names;
    const auto& points = plan.trajectory_.joint_trajectory.points;
    if (joint_names.empty() || points.empty()) {
      RCLCPP_ERROR(node_->get_logger(), "Named target plan '%s' has no trajectory points", target_name.c_str());
      return std::nullopt;
    }

    return TrajectoryEndpoint{joint_names, points.back().positions};
  }

  std::optional<moveit_msgs::msg::RobotTrajectory> planCircle(
    moveit::planning_interface::MoveGroupInterface& move_group,
    const TrajectoryEndpoint& start_state)
  {
    auto robot_state = makeRobotState(start_state);
    if (!robot_state) {
      return std::nullopt;
    }

    move_group.setStartState(*robot_state);
    const geometry_msgs::msg::Pose start_pose =
      eigenToPose(robot_state->getGlobalLinkTransform("gripper"));

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.reserve(repetitions_ * samples_per_circle_ + 1);

    const int total_samples = repetitions_ * samples_per_circle_;
    for (int i = 0; i <= total_samples; ++i) {
      const double theta = (2.0 * M_PI * static_cast<double>(i)) / static_cast<double>(samples_per_circle_);
      geometry_msgs::msg::Pose waypoint = start_pose;
      waypoint.position.x = start_pose.position.x;
      waypoint.position.y = start_pose.position.y + radius_ * std::cos(theta) - radius_;
      waypoint.position.z = start_pose.position.z + radius_ * std::sin(theta);
      waypoints.push_back(waypoint);
    }

    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = move_group.computeCartesianPath(waypoints, kCartesianStep, 0.0, trajectory);
    RCLCPP_INFO(
      node_->get_logger(),
      "Cartesian circle planning fraction: %.3f (%d waypoints, radius=%.3f, repetitions=%d)",
      fraction,
      static_cast<int>(waypoints.size()),
      radius_,
      repetitions_);

    if (fraction < kCircleFractionThreshold || trajectory.joint_trajectory.points.empty()) {
      RCLCPP_ERROR(node_->get_logger(), "Circle planning failed or incomplete");
      return std::nullopt;
    }

    return trajectory;
  }

  void publishPath(const moveit_msgs::msg::RobotTrajectory& trajectory)
  {
    if (!robot_model_ || trajectory.joint_trajectory.points.empty()) {
      return;
    }

    nav_msgs::msg::Path path;
    path.header.frame_id = "base";
    path.header.stamp = system_clock_.now();

    const auto& joint_names = trajectory.joint_trajectory.joint_names;
    for (const auto& point : trajectory.joint_trajectory.points) {
      TrajectoryEndpoint endpoint{joint_names, point.positions};
      auto robot_state = makeRobotState(endpoint);
      if (!robot_state) {
        continue;
      }

      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose = eigenToPose(robot_state->getGlobalLinkTransform("gripper"));
      path.poses.push_back(pose);
    }

    path_pub_->publish(path);
  }

  std::optional<moveit::core::RobotState> makeRobotState(const TrajectoryEndpoint& endpoint) const
  {
    if (!robot_model_) {
      RCLCPP_ERROR(node_->get_logger(), "Robot model is not available");
      return std::nullopt;
    }

    if (endpoint.joint_names.size() != endpoint.positions.size()) {
      RCLCPP_ERROR(node_->get_logger(), "Joint name/position size mismatch");
      return std::nullopt;
    }

    moveit::core::RobotState robot_state(robot_model_);
    robot_state.setToDefaultValues();

    for (std::size_t i = 0; i < endpoint.joint_names.size(); ++i) {
      robot_state.setVariablePosition(endpoint.joint_names[i], endpoint.positions[i]);
    }

    robot_state.update();
    return robot_state;
  }

  template<typename T>
  T getOrDeclareParameter(const std::string& name, const T& default_value)
  {
    if (node_->has_parameter(name)) {
      T value = default_value;
      node_->get_parameter(name, value);
      return value;
    }
    return node_->declare_parameter<T>(name, default_value);
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
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
  moveit::core::RobotModelPtr robot_model_;
  double radius_;
  int repetitions_;
  int samples_per_circle_;
  double velocity_scale_;
  double acceleration_scale_;
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

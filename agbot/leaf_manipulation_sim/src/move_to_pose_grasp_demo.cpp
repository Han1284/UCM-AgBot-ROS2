#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

using namespace std::chrono_literals;

namespace
{
constexpr double kJointPublishRate = 30.0;
constexpr double kDefaultFingerOpen = 0.10;
constexpr double kDefaultFingerClosed = 0.65;
constexpr double kDefaultMoveScale = 0.2;
constexpr double kDefaultTargetOffsetY = -0.10;
constexpr double kDefaultTargetOffsetZ = -0.05;
constexpr double kDefaultReplayDurationSec = 3.0;

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

Eigen::Isometry3d poseToEigen(const geometry_msgs::msg::Pose& pose)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  const Eigen::Quaterniond quaternion(
    pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  transform.linear() = quaternion.normalized().toRotationMatrix();
  return transform;
}

}  // namespace

class MoveToPoseGraspDemo
{
public:
  explicit MoveToPoseGraspDemo(const rclcpp::Node::SharedPtr& node)
  : node_(node),
    arm_command_pub_(node_->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/arm_position_controller/commands", 10)),
    gripper_command_pub_(node_->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/gripper_position_controller/commands", 10)),
    path_pub_(node_->create_publisher<nav_msgs::msg::Path>("pose_grasp_path", 10)),
    robot_model_loader_(std::make_shared<robot_model_loader::RobotModelLoader>(node_, "robot_description")),
    robot_model_(robot_model_loader_->getModel())
  {
    move_scale_ = getOrDeclareParameter("velocity_scale", kDefaultMoveScale);
    accel_scale_ = getOrDeclareParameter("acceleration_scale", kDefaultMoveScale);
    finger_open_ = getOrDeclareParameter("finger_open", kDefaultFingerOpen);
    finger_closed_ = getOrDeclareParameter("finger_closed", kDefaultFingerClosed);
    target_x_ = getOrDeclareParameter("target_x", std::numeric_limits<double>::quiet_NaN());
    target_y_ = getOrDeclareParameter("target_y", std::numeric_limits<double>::quiet_NaN());
    target_z_ = getOrDeclareParameter("target_z", std::numeric_limits<double>::quiet_NaN());
    target_roll_ = getOrDeclareParameter("target_roll", std::numeric_limits<double>::quiet_NaN());
    target_pitch_ = getOrDeclareParameter("target_pitch", std::numeric_limits<double>::quiet_NaN());
    target_yaw_ = getOrDeclareParameter("target_yaw", std::numeric_limits<double>::quiet_NaN());
    target_offset_x_ = getOrDeclareParameter("target_offset_x", 0.0);
    target_offset_y_ = getOrDeclareParameter("target_offset_y", kDefaultTargetOffsetY);
    target_offset_z_ = getOrDeclareParameter("target_offset_z", kDefaultTargetOffsetZ);
    replay_duration_sec_ = getOrDeclareParameter("replay_duration_sec", kDefaultReplayDurationSec);

    state_names_ = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "finger_joint"};
    state_positions_ = {0.0, 0.0, 1.5708, 0.0, 1.5708, 0.0, finger_open_};
    ready1_arm_positions_ = {0.0, 0.0, 1.5708, 0.0, 1.5708, 0.0};
    state_timer_ = node_->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / kJointPublishRate)),
      [this]() { publishState(); });
  }

  bool run()
  {
    if (!robot_model_) {
      RCLCPP_ERROR(node_->get_logger(), "机器人模型不可用");
      return false;
    }
    if (!waitForGazeboControllers(15s)) {
      return false;
    }

    resetPath();
    publishStableState(2s);

    auto initial_state = makeRobotState(state_names_, state_positions_);
    if (!initial_state) {
      return false;
    }

    const Eigen::Isometry3d world_to_base = initial_state->getGlobalLinkTransform("base");
    const auto start_pose = eigenToPose(
      world_to_base.inverse() * initial_state->getGlobalLinkTransform("gripper"));
    const auto target_pose = makeTargetPose(start_pose);
    const auto target_pose_world = eigenToPose(world_to_base * poseToEigen(target_pose));
    auto target_arm_positions = solveIkTarget(*initial_state, target_pose_world);
    if (!target_arm_positions) {
      return false;
    }
    RCLCPP_INFO(
      node_->get_logger(),
      "开始执行目标位姿 [%.3f, %.3f, %.3f]，运动时长 %.3f 秒",
      target_pose.position.x,
      target_pose.position.y,
      target_pose.position.z,
      replay_duration_sec_);
    replayArmMotion(ready1_arm_positions_, *target_arm_positions, replay_duration_sec_);

    if (!animateGripper(finger_closed_, 800ms, "夹爪闭合")) {
      return false;
    }
    if (!animateGripper(finger_open_, 800ms, "夹爪张开")) {
      return false;
    }

    RCLCPP_INFO(node_->get_logger(), "开始返回初始姿态 ready1");
    replayArmMotion(*target_arm_positions, ready1_arm_positions_, replay_duration_sec_);
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      state_positions_ = {
        ready1_arm_positions_[0],
        ready1_arm_positions_[1],
        ready1_arm_positions_[2],
        ready1_arm_positions_[3],
        ready1_arm_positions_[4],
        ready1_arm_positions_[5],
        finger_open_,
      };
    }
    publishStableState(1s);
    RCLCPP_INFO(node_->get_logger(), "位姿移动与夹爪开合流程执行完成");
    return true;
  }

private:
  std::optional<std::vector<double>> solveIkTarget(
    const moveit::core::RobotState& start_state,
    const geometry_msgs::msg::Pose& target_pose)
  {
    auto ik_state = start_state;
    const auto* joint_model_group = robot_model_->getJointModelGroup("tmr_arm");
    if (joint_model_group == nullptr) {
      RCLCPP_ERROR(node_->get_logger(), "未找到关节组 'tmr_arm'");
      return std::nullopt;
    }
    if (!ik_state.setFromIK(joint_model_group, target_pose, "gripper", 0.05)) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "目标位姿不可达，IK 求解失败。请缩小位移范围或调整姿态角后重试。");
      return std::nullopt;
    }
    ik_state.update();
    std::vector<double> arm_positions;
    ik_state.copyJointGroupPositions(joint_model_group, arm_positions);
    return arm_positions;
  }

  geometry_msgs::msg::Pose makeTargetPose(const geometry_msgs::msg::Pose& start_pose) const
  {
    geometry_msgs::msg::Pose pose = start_pose;
    pose.position.x = std::isnan(target_x_) ? start_pose.position.x + target_offset_x_ : target_x_;
    pose.position.y = std::isnan(target_y_) ? start_pose.position.y + target_offset_y_ : target_y_;
    pose.position.z = std::isnan(target_z_) ? start_pose.position.z + target_offset_z_ : target_z_;

    if (!std::isnan(target_roll_) && !std::isnan(target_pitch_) && !std::isnan(target_yaw_)) {
      const Eigen::AngleAxisd roll_angle(target_roll_, Eigen::Vector3d::UnitX());
      const Eigen::AngleAxisd pitch_angle(target_pitch_, Eigen::Vector3d::UnitY());
      const Eigen::AngleAxisd yaw_angle(target_yaw_, Eigen::Vector3d::UnitZ());
      const Eigen::Quaterniond q(yaw_angle * pitch_angle * roll_angle);
      pose.orientation.x = q.x();
      pose.orientation.y = q.y();
      pose.orientation.z = q.z();
      pose.orientation.w = q.w();
    }
    return pose;
  }

  std::optional<moveit::core::RobotState> makeRobotState(
    const std::vector<std::string>& names,
    const std::vector<double>& positions) const
  {
    if (names.size() != positions.size()) {
      return std::nullopt;
    }
    moveit::core::RobotState robot_state(robot_model_);
    robot_state.setToDefaultValues();
    for (std::size_t i = 0; i < names.size(); ++i) {
      robot_state.setVariablePosition(names[i], positions[i]);
    }
    robot_state.update();
    return robot_state;
  }

  bool animateGripper(double target, std::chrono::milliseconds duration, const std::string& label)
  {
    RCLCPP_INFO(node_->get_logger(), "%s", label.c_str());
    const std::size_t finger_index = state_names_.size() - 1;
    double start = 0.0;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      start = state_positions_[finger_index];
    }
    const auto start_time = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start_time < duration && rclcpp::ok()) {
      const double alpha = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start_time).count() /
        std::chrono::duration<double>(duration).count();
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        state_positions_[finger_index] = start + (target - start) * std::min(alpha, 1.0);
      }
      rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / kJointPublishRate)));
    }
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      state_positions_[finger_index] = target;
    }
    publishStableState(500ms);
    return true;
  }

  void replayArmMotion(
    const std::vector<double>& start_positions,
    const std::vector<double>& target_positions,
    double duration_sec)
  {
    if (start_positions.size() != ready1_arm_positions_.size() ||
      target_positions.size() != ready1_arm_positions_.size())
    {
      return;
    }

    const int steps = std::max(2, static_cast<int>(std::ceil(duration_sec * kJointPublishRate)));
    for (int step = 1; step <= steps; ++step) {
      const double alpha = static_cast<double>(step) / static_cast<double>(steps);
      std::vector<double> interpolated = start_positions;
      for (std::size_t joint = 0; joint < interpolated.size(); ++joint) {
        interpolated[joint] =
          start_positions[joint] + (target_positions[joint] - start_positions[joint]) * alpha;
      }
      updateArmState(interpolated);
      rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / kJointPublishRate)));
    }

    publishStableState(500ms);
  }

  void updateArmState(const std::vector<double>& positions)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    for (std::size_t i = 0; i < positions.size(); ++i) {
      state_positions_[i] = positions[i];
    }
  }

  void publishState()
  {
    std::vector<double> positions;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      positions = state_positions_;
    }
    std_msgs::msg::Float64MultiArray arm_command;
    arm_command.data.assign(positions.begin(), positions.begin() + 6);
    std_msgs::msg::Float64MultiArray gripper_command;
    gripper_command.data = {positions.back()};
    arm_command_pub_->publish(arm_command);
    gripper_command_pub_->publish(gripper_command);
    publishPathPoint(positions);
  }

  bool waitForGazeboControllers(std::chrono::seconds timeout)
  {
    RCLCPP_INFO(node_->get_logger(), "等待 Gazebo 机械臂与夹爪控制器");
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      if (arm_command_pub_->get_subscription_count() > 0 &&
        gripper_command_pub_->get_subscription_count() > 0)
      {
        return true;
      }
      rclcpp::sleep_for(100ms);
    }
    RCLCPP_ERROR(node_->get_logger(), "Gazebo 控制器在等待期限内未就绪");
    return false;
  }

  void publishStableState(std::chrono::milliseconds duration)
  {
    rclcpp::sleep_for(duration);
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

  void resetPath()
  {
    current_path_.header.frame_id = "base";
    current_path_.header.stamp = node_->get_clock()->now();
    current_path_.poses.clear();
    last_path_pose_.reset();
  }

  void publishPathPoint(const std::vector<double>& positions)
  {
    auto state = makeRobotState(state_names_, positions);
    if (!state) {
      return;
    }

    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = "base";
    pose_stamped.header.stamp = node_->get_clock()->now();
    pose_stamped.pose = eigenToPose(
      state->getGlobalLinkTransform("base").inverse() *
      state->getGlobalLinkTransform("gripper"));

    if (last_path_pose_) {
      const double dx = pose_stamped.pose.position.x - last_path_pose_->pose.position.x;
      const double dy = pose_stamped.pose.position.y - last_path_pose_->pose.position.y;
      const double dz = pose_stamped.pose.position.z - last_path_pose_->pose.position.z;
      if ((dx * dx + dy * dy + dz * dz) < 1e-8) {
        return;
      }
    }

    current_path_.header.stamp = pose_stamped.header.stamp;
    current_path_.poses.push_back(pose_stamped);
    last_path_pose_ = pose_stamped;
    path_pub_->publish(current_path_);
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr arm_command_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gripper_command_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
  moveit::core::RobotModelPtr robot_model_;
  std::vector<std::string> state_names_;
  std::vector<double> state_positions_;
  std::vector<double> ready1_arm_positions_;
  std::mutex state_mutex_;
  rclcpp::TimerBase::SharedPtr state_timer_;
  double move_scale_;
  double accel_scale_;
  double finger_open_;
  double finger_closed_;
  double target_x_;
  double target_y_;
  double target_z_;
  double target_roll_;
  double target_pitch_;
  double target_yaw_;
  double target_offset_x_;
  double target_offset_y_;
  double target_offset_z_;
  double replay_duration_sec_;
  nav_msgs::msg::Path current_path_;
  std::optional<geometry_msgs::msg::PoseStamped> last_path_pose_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("move_to_pose_grasp_demo", options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  MoveToPoseGraspDemo demo(node);
  const bool ok = demo.run();
  if (ok) {
    RCLCPP_INFO(
      node->get_logger(),
      "保持最终机械臂与夹爪关节状态；按 Ctrl+C 结束");
    while (rclcpp::ok()) {
      rclcpp::sleep_for(100ms);
    }
  }

  rclcpp::shutdown();
  spinner.join();
  return ok ? 0 : 1;
}

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <map>
#include <memory>
#include <mutex>
#include <numeric>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>

using namespace std::chrono_literals;

namespace
{
constexpr double kCommandRate = 30.0;

Eigen::Isometry3d poseToEigen(const geometry_msgs::msg::Pose& pose)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(
    pose.position.x, pose.position.y, pose.position.z);
  Eigen::Quaterniond quaternion(
    pose.orientation.w,
    pose.orientation.x,
    pose.orientation.y,
    pose.orientation.z);
  transform.linear() = quaternion.normalized().toRotationMatrix();
  return transform;
}

geometry_msgs::msg::Pose eigenToPose(const Eigen::Isometry3d& transform)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = transform.translation().x();
  pose.position.y = transform.translation().y();
  pose.position.z = transform.translation().z();
  const Eigen::Quaterniond quaternion(transform.rotation());
  pose.orientation.x = quaternion.x();
  pose.orientation.y = quaternion.y();
  pose.orientation.z = quaternion.z();
  pose.orientation.w = quaternion.w();
  return pose;
}

double toSeconds(const builtin_interfaces::msg::Duration& duration)
{
  return static_cast<double>(duration.sec) +
    static_cast<double>(duration.nanosec) * 1e-9;
}
}  // namespace

class MultiViewObservationDemo
{
public:
  explicit MultiViewObservationDemo(const rclcpp::Node::SharedPtr& node)
  : node_(node),
    robot_model_loader_(
      std::make_shared<robot_model_loader::RobotModelLoader>(
        node_, "robot_description")),
    robot_model_(robot_model_loader_->getModel())
  {
    initial_joints_ = getOrDeclareParameter(
      "initial_observation_joint_positions",
      std::vector<double>{0.213, 0.464, 0.548, 0.559, 1.571, 0.213});
    arm_group_ = getOrDeclareParameter(
      "arm_group", std::string("tmr_arm"));
    base_frame_ = getOrDeclareParameter(
      "base_frame", std::string("base"));
    end_effector_link_ = getOrDeclareParameter(
      "end_effector_link", std::string("gripper"));
    arm_joint_names_ = getOrDeclareParameter(
      "arm_joint_names",
      std::vector<std::string>{
        "joint_1", "joint_2", "joint_3",
        "joint_4", "joint_5", "joint_6"});
    arm_command_topic_ = getOrDeclareParameter(
      "arm_command_topic",
      std::string("/arm_position_controller/commands"));
    camera_link_ = getOrDeclareParameter(
      "camera_optical_frame", std::string("camera_depth_optical_frame"));
    execute_via_move_group_ = getOrDeclareParameter(
      "execute_via_move_group", false);
    velocity_scale_ = getOrDeclareParameter("velocity_scale", 0.18);
    acceleration_scale_ = getOrDeclareParameter("acceleration_scale", 0.18);
    maximum_views_ = getOrDeclareParameter("maximum_views", 5);
    settle_seconds_ = getOrDeclareParameter("settle_seconds", 0.45);

    if (!execute_via_move_group_) {
      arm_command_publisher_ =
        node_->create_publisher<std_msgs::msg::Float64MultiArray>(
        arm_command_topic_, 10);
    }
    capture_client_ = node_->create_client<std_srvs::srv::Trigger>(
      "/leaf_perception/capture_view");
    overview_client_ = node_->create_client<std_srvs::srv::Trigger>(
      "/leaf_perception/prepare_overview");
    finalize_client_ = node_->create_client<std_srvs::srv::Trigger>(
      "/leaf_perception/finalize");
    auto durable = rclcpp::QoS(1).transient_local();
    views_subscription_ =
      node_->create_subscription<geometry_msgs::msg::PoseArray>(
      "/leaf_perception/observation_views",
      durable,
      [this](const geometry_msgs::msg::PoseArray::SharedPtr message) {
        {
          std::lock_guard<std::mutex> lock(views_mutex_);
          observation_views_ = *message;
          ++views_revision_;
        }
        views_condition_.notify_all();
      });
  }

  bool run()
  {
    if (!robot_model_) {
      RCLCPP_ERROR(node_->get_logger(), "Robot model is unavailable");
      return false;
    }
    if (!capture_client_->wait_for_service(15s)) {
      RCLCPP_ERROR(
        node_->get_logger(), "Leaf capture service is unavailable");
      return false;
    }

    moveit::planning_interface::MoveGroupInterface move_group(
      node_, arm_group_);
    move_group.setPoseReferenceFrame(base_frame_);
    move_group.setEndEffectorLink(end_effector_link_);
    move_group.setMaxVelocityScalingFactor(velocity_scale_);
    move_group.setMaxAccelerationScalingFactor(acceleration_scale_);
    move_group.setPlanningTime(3.0);

    if (!moveToInitialObservation(move_group)) {
      return false;
    }
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(settle_seconds_)));

    auto gripper_to_camera = currentGripperToCamera(move_group);
    if (!gripper_to_camera) {
      return false;
    }

    std::size_t consumed_views_revision = 0;
    bool overview_moved = false;
    if (prepareOverview()) {
      auto overview_views =
        waitForObservationViews(consumed_views_revision, 5s);
      if (overview_views && !overview_views->second.poses.empty()) {
        consumed_views_revision = overview_views->first;
        int pose_index = 0;
        for (const auto& pose : overview_views->second.poses) {
          ++pose_index;
          if (moveToCameraPose(
              move_group,
              poseToEigen(pose),
              *gripper_to_camera,
              "full-plant overview " + std::to_string(pose_index)))
          {
            overview_moved = true;
            break;
          }
        }
      } else {
        RCLCPP_WARN(
          node_->get_logger(),
          "No full-plant overview candidates received; using safe view");
      }
    }
    if (!overview_moved) {
      RCLCPP_WARN(
        node_->get_logger(),
        "No full-plant overview pose was reachable; "
        "capturing from the safe initial view without blocking");
    } else {
      rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(settle_seconds_)));
    }

    auto initial_capture = captureView();
    if (!initial_capture) {
      return false;
    }
    RCLCPP_INFO(
      node_->get_logger(), "First fused canopy capture: %s",
      initial_capture->c_str());
    RCLCPP_INFO(
      node_->get_logger(),
      "Selecting one geometry-diverse validation NBV by default; "
      "additional NBVs are requested only when candidates are insufficient "
      "or the fused surface has not converged");

    auto views = waitForObservationViews(consumed_views_revision, 5s);
    if (!views || views->second.poses.empty()) {
      RCLCPP_ERROR(
        node_->get_logger(), "No canopy-relative observation views received");
      return false;
    }
    consumed_views_revision = views->first;

    int captured_views = 1;
    const int extra_limit = std::max(0, maximum_views_ - 1);
    bool candidates_sufficient = false;
    for (int observation_index = 0;
      observation_index < extra_limit && rclcpp::ok();
      ++observation_index)
    {
      if (observation_index > 0) {
        views = waitForObservationViews(consumed_views_revision, 5s);
        if (!views || views->second.poses.empty()) {
          // Candidate shortage is supposed to buy more NBV coverage.  Give
          // perception one longer wait before abandoning the loop, instead of
          // finalizing a thin Trex-compliant set early.
          RCLCPP_WARN(
            node_->get_logger(),
            "No updated peripheral observation views yet; waiting longer "
            "because candidates are still insufficient");
          views = waitForObservationViews(consumed_views_revision, 10s);
        }
        if (!views || views->second.poses.empty()) {
          RCLCPP_WARN(
            node_->get_logger(),
            "No updated peripheral observation views received");
          break;
        }
        consumed_views_revision = views->first;
      }

      bool moved = false;
      std::optional<std::string> capture;
      int pose_index = 0;
      auto ordered_poses =
        orderByMotionFromCurrentCamera(move_group, views->second);
      for (const auto& pose : ordered_poses) {
        ++pose_index;
        if (moveToCameraPose(
              move_group,
              poseToEigen(pose),
              *gripper_to_camera,
              "completion NBV " +
              std::to_string(observation_index + 2) + "." +
              std::to_string(pose_index)))
        {
          moved = true;
          rclcpp::sleep_for(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
              std::chrono::duration<double>(settle_seconds_)));
          capture = captureView();
          if (capture) {
            break;
          }
          RCLCPP_WARN(
            node_->get_logger(),
            "Reachable completion NBV %d.%d produced no valid leaf "
            "observation; trying the next candidate",
            observation_index + 2, pose_index);
        }
      }
      if (!moved) {
        RCLCPP_WARN(
          node_->get_logger(),
          "Skipping unreachable adaptive observation %d",
          observation_index + 2);
        break;
      }
      if (!capture) {
        RCLCPP_WARN(
          node_->get_logger(),
          "No reachable NBV candidate produced a valid observation for "
          "round %d; stopping acquisition and finalizing existing views",
          observation_index + 2);
        break;
      }
      ++captured_views;
      RCLCPP_INFO(
        node_->get_logger(), "Observation %d: %s",
        captured_views, capture->c_str());
      const bool ready =
        capture->find("ready=True") != std::string::npos ||
        capture->find("ready=true") != std::string::npos;
      const bool has_candidates =
        capture->find("projected_candidates=0;") == std::string::npos;
      candidates_sufficient =
        capture->find("sufficient=True") != std::string::npos ||
        capture->find("sufficient=true") != std::string::npos;
      // Stop only when perception itself says the Trex-compliant set is
      // ready.  A thin candidate list must continue requesting NBV.
      if (ready && has_candidates && candidates_sufficient)
      {
        RCLCPP_INFO(
          node_->get_logger(),
          "Adaptive stopping condition reached after %d views",
          captured_views);
        return true;
      }
      if (ready && !has_candidates) {
        RCLCPP_ERROR(
          node_->get_logger(),
          "Perception reported ready without any grasp candidates; "
          "continuing to finalization instead of reporting false success");
      }
      if (!candidates_sufficient) {
        RCLCPP_INFO(
          node_->get_logger(),
          "Candidate set still insufficient after %d views; "
          "requesting another NBV for more canopy coverage",
          captured_views);
      }
    }
    const bool finalized = finalizeCandidates();
    RCLCPP_INFO(
      node_->get_logger(),
      "Observation sequence finished after %d captured views; finalized=%s",
      captured_views, finalized ? "true" : "false");
    return captured_views >= 2 && finalized;
  }

private:
  bool moveToInitialObservation(
    moveit::planning_interface::MoveGroupInterface& move_group)
  {
    if (initial_joints_.size() != arm_joint_names_.size()) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "initial_observation_joint_positions and arm_joint_names "
        "must have equal length");
      return false;
    }
    std::map<std::string, double> target;
    for (std::size_t index = 0; index < arm_joint_names_.size(); ++index) {
      target[arm_joint_names_[index]] = initial_joints_[index];
    }
    move_group.setJointValueTarget(target);
    return planAndReplay(move_group, "safe initial observation");
  }

  bool moveToPose(
    moveit::planning_interface::MoveGroupInterface& move_group,
    const geometry_msgs::msg::Pose& pose,
    const std::string& label)
  {
    move_group.setStartStateToCurrentState();
    const bool has_ik = move_group.setJointValueTarget(
      pose, end_effector_link_);
    if (!has_ik) {
      RCLCPP_WARN(
        node_->get_logger(), "No IK solution for %s", label.c_str());
      return false;
    }
    const bool success = planAndReplay(move_group, label);
    return success;
  }

  bool moveToCameraPose(
    moveit::planning_interface::MoveGroupInterface& move_group,
    const Eigen::Isometry3d& base_to_camera,
    const Eigen::Isometry3d& gripper_to_camera,
    const std::string& label)
  {
    constexpr std::array<double, 4> optical_rolls = {
      0.0, -M_PI_2, M_PI_2, M_PI};
    for (const double roll : optical_rolls) {
      Eigen::Isometry3d rolled_camera = base_to_camera;
      rolled_camera.rotate(Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitZ()));
      const Eigen::Isometry3d base_to_gripper =
        rolled_camera * gripper_to_camera.inverse();
      if (moveToPose(move_group, eigenToPose(base_to_gripper), label)) {
        return true;
      }
    }
    return false;
  }

  std::optional<Eigen::Isometry3d> currentCameraPose(
    moveit::planning_interface::MoveGroupInterface& move_group)
  {
    auto state = move_group.getCurrentState(2.0);
    if (!state || !robot_model_->hasLinkModel(camera_link_)) {
      return std::nullopt;
    }
    const Eigen::Isometry3d base_to_global =
      state->getGlobalLinkTransform(base_frame_).inverse();
    return base_to_global * state->getGlobalLinkTransform(camera_link_);
  }

  std::vector<geometry_msgs::msg::Pose>
  localParallaxViewsFromCurrentCamera(
    moveit::planning_interface::MoveGroupInterface& move_group)
  {
    const auto current = currentCameraPose(move_group);
    if (!current) {
      RCLCPP_WARN(
        node_->get_logger(),
        "Cannot derive local parallax views from the current camera pose");
      return {};
    }

    // The successful overview is a measured reachable pose. Small offsets in
    // its optical frame provide genuine parallax without asking the compact
    // Pro450 to jump to a distant global shell. The perception node still
    // decides whether the captured view contributes enough surface evidence.
    const std::array<Eigen::Vector3d, 8> optical_offsets = {
      Eigen::Vector3d(0.010, 0.000, 0.000),
      Eigen::Vector3d(-0.010, 0.000, 0.000),
      Eigen::Vector3d(0.020, 0.000, 0.000),
      Eigen::Vector3d(-0.020, 0.000, 0.000),
      Eigen::Vector3d(0.000, 0.010, 0.000),
      Eigen::Vector3d(0.000, -0.010, 0.000),
      Eigen::Vector3d(0.010, 0.000, -0.010),
      Eigen::Vector3d(-0.010, 0.000, -0.010),
    };
    std::vector<geometry_msgs::msg::Pose> result;
    result.reserve(optical_offsets.size());
    for (const auto& optical_offset : optical_offsets) {
      Eigen::Isometry3d candidate = *current;
      candidate.translation() += current->rotation() * optical_offset;
      result.push_back(eigenToPose(candidate));
    }
    RCLCPP_INFO(
      node_->get_logger(),
      "Prepending %zu local parallax views around current camera "
      "(%.3f, %.3f, %.3f)",
      result.size(),
      current->translation().x(),
      current->translation().y(),
      current->translation().z());
    return result;
  }

  std::vector<geometry_msgs::msg::Pose> orderByMotionFromCurrentCamera(
    moveit::planning_interface::MoveGroupInterface& move_group,
    const geometry_msgs::msg::PoseArray& views)
  {
    const auto current = currentCameraPose(move_group);
    if (!current) {
      RCLCPP_WARN(
        node_->get_logger(),
        "Cannot read the current camera pose; retaining NBV quality order");
      return views.poses;
    }

    const Eigen::Vector3d current_position = current->translation();
    const Eigen::Quaterniond current_orientation(current->rotation());

    std::vector<std::size_t> indices(views.poses.size());
    std::iota(indices.begin(), indices.end(), 0);
    auto motion_cost = [&](const std::size_t index) {
        const Eigen::Isometry3d candidate = poseToEigen(views.poses[index]);
        const double translation =
          (candidate.translation() - current_position).norm();
        const Eigen::Quaterniond candidate_orientation(candidate.rotation());
        const double rotation = current_orientation.angularDistance(
          candidate_orientation);
        // Ten degrees of optical-axis motion has roughly the same priority
        // cost as 1 cm of camera translation. Candidate quality is already
        // screened upstream; this ordering only improves execution
        // reachability around the last successful observation.
        return translation + 0.01 * rotation / (10.0 * M_PI / 180.0);
      };
    std::stable_sort(
      indices.begin(), indices.end(),
      [&](const std::size_t left, const std::size_t right) {
        return motion_cost(left) < motion_cost(right);
      });

    std::vector<geometry_msgs::msg::Pose> ordered;
    ordered.reserve(indices.size());
    for (const auto index : indices) {
      ordered.push_back(views.poses[index]);
    }
    if (!indices.empty()) {
      const auto nearest = indices.front();
      const Eigen::Isometry3d candidate = poseToEigen(views.poses[nearest]);
      RCLCPP_INFO(
        node_->get_logger(),
        "Trying %zu quality-filtered NBVs in local-motion order; "
        "nearest original rank=%zu, translation=%.3f m",
        ordered.size(), nearest + 1,
        (candidate.translation() - current_position).norm());
    }
    return ordered;
  }

  bool planAndReplay(
    moveit::planning_interface::MoveGroupInterface& move_group,
    const std::string& label)
  {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto result = move_group.plan(plan);
    if (result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(
        node_->get_logger(), "Failed to plan %s", label.c_str());
      return false;
    }
    RCLCPP_INFO(
      node_->get_logger(), "Replaying collision-aware %s", label.c_str());
    if (execute_via_move_group_) {
      const auto execution = move_group.execute(plan);
      if (execution != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(
          node_->get_logger(), "Failed to execute %s", label.c_str());
        return false;
      }
      // The action result means the controller reached its terminal point;
      // wait briefly for the RGB-D stream to settle before capture.
      rclcpp::sleep_for(300ms);
      return true;
    }
    replayTrajectory(plan.trajectory_);
    return true;
  }

  void replayTrajectory(const moveit_msgs::msg::RobotTrajectory& trajectory)
  {
    if (!arm_command_publisher_) {
      return;
    }
    const auto& names = trajectory.joint_trajectory.joint_names;
    const auto& points = trajectory.joint_trajectory.points;
    if (names.empty() || points.empty()) {
      return;
    }
    const auto discovery_deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (
      arm_command_publisher_->get_subscription_count() == 0 &&
      std::chrono::steady_clock::now() < discovery_deadline &&
      rclcpp::ok())
    {
      rclcpp::sleep_for(50ms);
    }
    if (arm_command_publisher_->get_subscription_count() == 0) {
      RCLCPP_WARN(
        node_->get_logger(),
        "No subscriber discovered on %s before trajectory replay",
        arm_command_topic_.c_str());
    }
    builtin_interfaces::msg::Duration previous_stamp;
    for (const auto& point : points) {
      std_msgs::msg::Float64MultiArray command;
      command.data.resize(arm_joint_names_.size(), 0.0);
      for (std::size_t index = 0; index < names.size(); ++index) {
        const auto configured = std::find(
          arm_joint_names_.begin(), arm_joint_names_.end(), names[index]);
        if (configured != arm_joint_names_.end()) {
          const auto joint_index = static_cast<std::size_t>(
            std::distance(arm_joint_names_.begin(), configured));
          command.data[joint_index] = point.positions[index];
        }
      }
      arm_command_publisher_->publish(command);
      const double wait_seconds =
        toSeconds(point.time_from_start) - toSeconds(previous_stamp);
      if (wait_seconds > 0.0) {
        rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(wait_seconds)));
      } else {
        rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(1.0 / kCommandRate)));
      }
      previous_stamp = point.time_from_start;
    }
    // Keep publishing the terminal command long enough for a fresh Gazebo
    // controller and joint-state feedback loop to converge before capture.
    const auto end_time = std::chrono::steady_clock::now() + 1500ms;
    while (std::chrono::steady_clock::now() < end_time && rclcpp::ok()) {
      std_msgs::msg::Float64MultiArray command;
      command.data.resize(arm_joint_names_.size(), 0.0);
      for (std::size_t index = 0; index < names.size(); ++index) {
        const auto configured = std::find(
          arm_joint_names_.begin(), arm_joint_names_.end(), names[index]);
        if (configured != arm_joint_names_.end()) {
          const auto joint_index = static_cast<std::size_t>(
            std::distance(arm_joint_names_.begin(), configured));
          command.data[joint_index] = points.back().positions[index];
        }
      }
      arm_command_publisher_->publish(command);
      rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / kCommandRate)));
    }
  }

  std::optional<Eigen::Isometry3d> currentGripperToCamera(
    moveit::planning_interface::MoveGroupInterface& move_group)
  {
    auto state = move_group.getCurrentState(2.0);
    if (!state || !robot_model_->hasLinkModel(camera_link_)) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Cannot resolve fixed transform from gripper to %s",
        camera_link_.c_str());
      return std::nullopt;
    }
    const Eigen::Isometry3d base_to_global =
      state->getGlobalLinkTransform(base_frame_).inverse();
    const Eigen::Isometry3d base_to_gripper =
      base_to_global * state->getGlobalLinkTransform(end_effector_link_);
    const Eigen::Isometry3d base_to_camera =
      base_to_global * state->getGlobalLinkTransform(camera_link_);
    return base_to_gripper.inverse() * base_to_camera;
  }

  std::optional<std::string> captureView()
  {
    // The RGB-D bridge may become available a little before its first
    // organized cloud.  Treat that particular rejection as a bounded startup
    // wait, rather than incorrectly terminating a valid multi-view run.
    for (int attempt = 0; attempt < 50 && rclcpp::ok(); ++attempt) {
      auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
      auto future = capture_client_->async_send_request(request);
      if (future.wait_for(2s) != std::future_status::ready) {
        RCLCPP_ERROR(node_->get_logger(), "Timed out capturing RGB-D view");
        return std::nullopt;
      }
      const auto response = future.get();
      if (response->success) {
        return response->message;
      }
      if (!isTransientCaptureRejection(response->message)) {
        RCLCPP_WARN(
          node_->get_logger(), "Capture rejected: %s",
          response->message.c_str());
        return std::nullopt;
      }
      if (attempt == 0) {
        RCLCPP_INFO(
          node_->get_logger(),
          "Waiting for a transformable organized RGB-D cloud: %s",
          response->message.c_str());
      }
      rclcpp::sleep_for(200ms);
    }
    RCLCPP_ERROR(node_->get_logger(), "No organized RGB-D cloud arrived");
    return std::nullopt;
  }

  bool prepareOverview()
  {
    if (!overview_client_->wait_for_service(2s)) {
      RCLCPP_WARN(
        node_->get_logger(),
        "Overview preparation service is unavailable; using safe view");
      return false;
    }
    for (int attempt = 0; attempt < 50 && rclcpp::ok(); ++attempt) {
      auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
      auto future = overview_client_->async_send_request(request);
      if (future.wait_for(2s) != std::future_status::ready) {
        RCLCPP_WARN(
          node_->get_logger(),
          "Timed out preparing full-plant overview; using safe view");
        return false;
      }
      const auto response = future.get();
      if (response->success) {
        RCLCPP_INFO(
          node_->get_logger(), "Overview preparation: %s",
          response->message.c_str());
        return true;
      }
      if (!isTransientCaptureRejection(response->message)) {
        RCLCPP_WARN(
          node_->get_logger(), "Overview preparation rejected: %s",
          response->message.c_str());
        return false;
      }
      if (attempt == 0) {
        RCLCPP_INFO(
          node_->get_logger(),
          "Waiting for an overview cloud with matching TF: %s",
          response->message.c_str());
      }
      rclcpp::sleep_for(200ms);
    }
    RCLCPP_WARN(
      node_->get_logger(),
      "No organized RGB-D cloud arrived for full-plant overview");
    return false;
  }

  static bool isTransientCaptureRejection(const std::string& message)
  {
    constexpr std::array<const char*, 4> transient_fragments = {
      "No organized RGB point cloud",
      "Lookup would require extrapolation into the past",
      "Lookup would require extrapolation into the future",
      "Could not transform",
    };
    return std::any_of(
      transient_fragments.begin(), transient_fragments.end(),
      [&message](const char* fragment) {
        return message.find(fragment) != std::string::npos;
      });
  }

  bool finalizeCandidates()
  {
    if (!finalize_client_->wait_for_service(2s)) {
      RCLCPP_ERROR(node_->get_logger(), "Finalize service is unavailable");
      return false;
    }
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = finalize_client_->async_send_request(request);
    if (future.wait_for(10s) != std::future_status::ready) {
      RCLCPP_ERROR(node_->get_logger(), "Timed out finalizing candidates");
      return false;
    }
    const auto response = future.get();
    if (!response->success) {
      RCLCPP_ERROR(
        node_->get_logger(), "Candidate finalization failed: %s",
        response->message.c_str());
      return false;
    }
    RCLCPP_INFO(
      node_->get_logger(), "Candidate finalization: %s",
      response->message.c_str());
    return true;
  }

  std::optional<std::pair<std::size_t, geometry_msgs::msg::PoseArray>>
  waitForObservationViews(
    std::size_t after_revision,
    std::chrono::seconds timeout)
  {
    std::unique_lock<std::mutex> lock(views_mutex_);
    if (!views_condition_.wait_for(
        lock, timeout, [this, after_revision]() {
          return views_revision_ > after_revision &&
                 !observation_views_.poses.empty();
        }))
    {
      return std::nullopt;
    }
    return std::make_pair(views_revision_, observation_views_);
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

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
  moveit::core::RobotModelPtr robot_model_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
    arm_command_publisher_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr capture_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr overview_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr finalize_client_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr
    views_subscription_;
  std::vector<double> initial_joints_;
  std::string arm_group_;
  std::string base_frame_;
  std::string end_effector_link_;
  std::vector<std::string> arm_joint_names_;
  std::string arm_command_topic_;
  std::string camera_link_;
  bool execute_via_move_group_{false};
  double velocity_scale_;
  double acceleration_scale_;
  int maximum_views_;
  double settle_seconds_;
  std::mutex views_mutex_;
  std::condition_variable views_condition_;
  geometry_msgs::msg::PoseArray observation_views_;
  std::size_t views_revision_{0};
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared(
    "multi_view_observation_demo", options);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  MultiViewObservationDemo demo(node);
  const bool success = demo.run();
  executor.cancel();
  if (spinner.joinable()) {
    spinner.join();
  }
  rclcpp::shutdown();
  return success ? 0 : 1;
}

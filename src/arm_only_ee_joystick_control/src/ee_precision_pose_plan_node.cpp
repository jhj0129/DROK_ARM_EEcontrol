#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/msg/display_trajectory.hpp>

using namespace std::chrono_literals;

class EEPrecisionPosePlanNode : public rclcpp::Node
{
public:
  EEPrecisionPosePlanNode()
  : Node("ee_precision_pose_plan_node")
  {
    declare_parameter<std::string>("planning_group", "arm");
    declare_parameter<std::string>("ee_link", "gripper_tcp");
    declare_parameter<std::string>("planner_id", "RRTConnectkConfigDefault");

    declare_parameter<double>("planning_time", 4.0);
    declare_parameter<int>("planning_attempts", 5);
    declare_parameter<double>("velocity_scaling", 0.20);
    declare_parameter<double>("accel_scaling", 0.20);

    declare_parameter<double>("workspace_x_min", -0.70);
    declare_parameter<double>("workspace_x_max", 0.90);
    declare_parameter<double>("workspace_y_min", -0.70);
    declare_parameter<double>("workspace_y_max", 0.70);
    declare_parameter<double>("workspace_z_min", -0.20);
    declare_parameter<double>("workspace_z_max", 0.90);

    declare_parameter<std::vector<double>>(
      "preferred_joints",
      std::vector<double>{-0.0318, -1.20, -0.95, 0.0106, 0.0404, 3.2295}
    );

    declare_parameter<std::vector<double>>(
      "posture_weights",
      std::vector<double>{0.3, 1.2, 1.2, 0.8, 0.4, 0.4}
    );

    declare_parameter<std::vector<double>>(
      "continuity_weights",
      std::vector<double>{0.3, 1.5, 1.5, 0.8, 0.5, 0.5}
    );

    declare_parameter<double>("ik_position_tolerance", 0.003);
    declare_parameter<int>("ik_max_iterations", 180);
    declare_parameter<double>("ik_damping", 0.06);
    declare_parameter<double>("ik_step_scale", 0.30);

    declare_parameter<double>("nullspace_gain", 0.005);
    declare_parameter<double>("continuity_gain", 0.04);
    declare_parameter<double>("joint_limit_gain", 0.01);

    declare_parameter<double>("max_joint_step", 0.015);
    declare_parameter<double>("joint_limit_margin", 0.12);
    declare_parameter<double>("max_goal_joint_delta", 0.06);

    declare_parameter<double>("max_waypoint_joint_jump", 0.08);
    declare_parameter<double>("max_total_joint_delta", 0.70);

    declare_parameter<bool>("execute_plan", false);
    declare_parameter<bool>("save_last_goal_on_plan_only", true);

    planning_group_ = get_parameter("planning_group").as_string();
    ee_link_ = get_parameter("ee_link").as_string();
    planner_id_ = get_parameter("planner_id").as_string();

    planning_time_ = get_parameter("planning_time").as_double();
    planning_attempts_ = get_parameter("planning_attempts").as_int();
    velocity_scaling_ = get_parameter("velocity_scaling").as_double();
    accel_scaling_ = get_parameter("accel_scaling").as_double();

    workspace_x_min_ = get_parameter("workspace_x_min").as_double();
    workspace_x_max_ = get_parameter("workspace_x_max").as_double();
    workspace_y_min_ = get_parameter("workspace_y_min").as_double();
    workspace_y_max_ = get_parameter("workspace_y_max").as_double();
    workspace_z_min_ = get_parameter("workspace_z_min").as_double();
    workspace_z_max_ = get_parameter("workspace_z_max").as_double();

    preferred_joints_ = get_parameter("preferred_joints").as_double_array();
    posture_weights_ = get_parameter("posture_weights").as_double_array();
    continuity_weights_ = get_parameter("continuity_weights").as_double_array();

    ik_position_tolerance_ = get_parameter("ik_position_tolerance").as_double();
    ik_max_iterations_ = get_parameter("ik_max_iterations").as_int();
    ik_damping_ = get_parameter("ik_damping").as_double();
    ik_step_scale_ = get_parameter("ik_step_scale").as_double();

    nullspace_gain_ = get_parameter("nullspace_gain").as_double();
    continuity_gain_ = get_parameter("continuity_gain").as_double();
    joint_limit_gain_ = get_parameter("joint_limit_gain").as_double();

    max_joint_step_ = get_parameter("max_joint_step").as_double();
    joint_limit_margin_ = get_parameter("joint_limit_margin").as_double();
    max_goal_joint_delta_ = get_parameter("max_goal_joint_delta").as_double();

    max_waypoint_joint_jump_ = get_parameter("max_waypoint_joint_jump").as_double();
    max_total_joint_delta_ = get_parameter("max_total_joint_delta").as_double();

    execute_plan_ = get_parameter("execute_plan").as_bool();
    save_last_goal_on_plan_only_ = get_parameter("save_last_goal_on_plan_only").as_bool();

    target_point_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "/ee_precision/target_point",
      10,
      std::bind(&EEPrecisionPosePlanNode::targetPointCallback, this, std::placeholders::_1)
    );

    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "/ee_precision/target_marker",
      10
    );

    display_pub_ = create_publisher<moveit_msgs::msg::DisplayTrajectory>(
      "/display_planned_path",
      10
    );

    RCLCPP_WARN(
      get_logger(),
      "SAFE DEFAULT: execute_plan=false. This node plans/displays only unless explicitly enabled."
    );

    RCLCPP_INFO(
      get_logger(),
      "EE precision pose planning node created. Publish geometry_msgs/PointStamped to /ee_precision/target_point."
    );
  }

  void initializeMoveIt()
  {
    RCLCPP_INFO(get_logger(), "Initializing MoveGroupInterface...");

    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(),
      planning_group_
    );

    move_group_->setEndEffectorLink(ee_link_);
    move_group_->setPlanningTime(planning_time_);
    move_group_->setNumPlanningAttempts(planning_attempts_);
    move_group_->setMaxVelocityScalingFactor(velocity_scaling_);
    move_group_->setMaxAccelerationScalingFactor(accel_scaling_);
    move_group_->setPlannerId(planner_id_);
    move_group_->startStateMonitor(5.0);

    planning_frame_ = move_group_->getPlanningFrame();

    bool pose_ok = false;
    for (int i = 0; i < 30; ++i) {
      try {
        auto current_pose_stamped = move_group_->getCurrentPose(ee_link_);
        target_pose_ = current_pose_stamped.pose;
        pose_ok = true;
        break;
      } catch (...) {
        rclcpp::sleep_for(200ms);
      }
    }

    if (!pose_ok) {
      RCLCPP_WARN(get_logger(), "Failed to get current EE pose. Using default pose.");
      target_pose_.orientation.w = 1.0;
      target_pose_.position.x = 0.20;
      target_pose_.position.y = 0.00;
      target_pose_.position.z = 0.30;
    }

    moveit_ready_ = true;
    publishMarker();

    RCLCPP_INFO(get_logger(), "MoveIt initialized.");
    RCLCPP_INFO(get_logger(), "  planning_group = %s", planning_group_.c_str());
    RCLCPP_INFO(get_logger(), "  ee_link        = %s", ee_link_.c_str());
    RCLCPP_INFO(get_logger(), "  planning_frame = %s", planning_frame_.c_str());
    RCLCPP_INFO(get_logger(), "  execute_plan   = %s", execute_plan_ ? "true" : "false");

    RCLCPP_INFO(get_logger(), "Mode 1 high-performance params:");
    RCLCPP_INFO(get_logger(), "  ik_position_tolerance = %.4f m", ik_position_tolerance_);
    RCLCPP_INFO(get_logger(), "  ik_max_iterations     = %d", ik_max_iterations_);
    RCLCPP_INFO(get_logger(), "  ik_damping            = %.4f", ik_damping_);
    RCLCPP_INFO(get_logger(), "  ik_step_scale         = %.4f", ik_step_scale_);
    RCLCPP_INFO(get_logger(), "  nullspace_gain        = %.4f", nullspace_gain_);
    RCLCPP_INFO(get_logger(), "  continuity_gain       = %.4f", continuity_gain_);
    RCLCPP_INFO(get_logger(), "  joint_limit_gain      = %.4f", joint_limit_gain_);
    RCLCPP_INFO(get_logger(), "  max_joint_step        = %.4f rad", max_joint_step_);
    RCLCPP_INFO(get_logger(), "  max_goal_joint_delta  = %.4f rad", max_goal_joint_delta_);
    RCLCPP_INFO(get_logger(), "  max_waypoint_jump     = %.4f rad", max_waypoint_joint_jump_);
  }

private:
  static double clamp(double v, double lo, double hi)
  {
    return std::max(lo, std::min(hi, v));
  }

  static double angleDiff(double target, double current)
  {
    double d = target - current;
    while (d > M_PI) {
      d -= 2.0 * M_PI;
    }
    while (d < -M_PI) {
      d += 2.0 * M_PI;
    }
    return d;
  }

  void targetPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(plan_mutex_);

    if (!moveit_ready_ || !move_group_) {
      RCLCPP_WARN(get_logger(), "MoveIt is not ready yet. Ignoring target point.");
      return;
    }

    target_pose_.position.x = clamp(msg->point.x, workspace_x_min_, workspace_x_max_);
    target_pose_.position.y = clamp(msg->point.y, workspace_y_min_, workspace_y_max_);
    target_pose_.position.z = clamp(msg->point.z, workspace_z_min_, workspace_z_max_);

    try {
      auto current_pose = move_group_->getCurrentPose(ee_link_);
      target_pose_.orientation = current_pose.pose.orientation;
    } catch (...) {
      target_pose_.orientation.w = 1.0;
    }

    RCLCPP_INFO(
      get_logger(),
      "Received target point in frame [%s]: x=%.3f y=%.3f z=%.3f",
      msg->header.frame_id.c_str(),
      target_pose_.position.x,
      target_pose_.position.y,
      target_pose_.position.z
    );

    publishMarker();
    planTargetHighPerformance();
  }

  void publishMarker()
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = planning_frame_.empty() ? "ARM_BASE_LINK" : planning_frame_;
    marker.header.stamp = now();
    marker.ns = "ee_precision";
    marker.id = 1;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose = target_pose_;

    marker.scale.x = 0.045;
    marker.scale.y = 0.045;
    marker.scale.z = 0.045;

    marker.color.r = 1.0;
    marker.color.g = 0.05;
    marker.color.b = 0.05;
    marker.color.a = 0.9;

    marker_pub_->publish(marker);
  }

  bool computePrecisionIK(std::map<std::string, double> & q_goal_map)
  {
    auto state = move_group_->getCurrentState(2.0);
    if (!state) {
      RCLCPP_ERROR(get_logger(), "Failed to get current robot state.");
      return false;
    }

    const moveit::core::JointModelGroup * jmg = state->getJointModelGroup(planning_group_);
    if (!jmg) {
      RCLCPP_ERROR(get_logger(), "JointModelGroup [%s] not found.", planning_group_.c_str());
      return false;
    }

    const moveit::core::LinkModel * ee_link_model = state->getLinkModel(ee_link_);
    if (!ee_link_model) {
      RCLCPP_ERROR(get_logger(), "EE link [%s] not found.", ee_link_.c_str());
      return false;
    }

    std::vector<double> q_start;
    state->copyJointGroupPositions(jmg, q_start);

    const int n = static_cast<int>(q_start.size());
    if (n <= 0) {
      RCLCPP_ERROR(get_logger(), "No active joints in planning group.");
      return false;
    }

    if (static_cast<int>(preferred_joints_.size()) != n ||
        static_cast<int>(posture_weights_.size()) != n ||
        static_cast<int>(continuity_weights_.size()) != n)
    {
      RCLCPP_ERROR(
        get_logger(),
        "Parameter size mismatch. joints=%d preferred=%zu posture=%zu continuity=%zu",
        n,
        preferred_joints_.size(),
        posture_weights_.size(),
        continuity_weights_.size()
      );
      return false;
    }

    Eigen::VectorXd q(n);
    Eigen::VectorXd q0(n);
    Eigen::VectorXd q_pref(n);
    Eigen::VectorXd w_post(n);
    Eigen::VectorXd w_cont(n);

    for (int i = 0; i < n; ++i) {
      q(i) = q_start[i];
      q0(i) = q_start[i];
      q_pref(i) = preferred_joints_[i];
      w_post(i) = posture_weights_[i];
      w_cont(i) = continuity_weights_[i];
    }

    Eigen::VectorXd q_cont_ref = q0;
    if (has_last_q_goal_ && static_cast<int>(last_q_goal_.size()) == n) {
      for (int i = 0; i < n; ++i) {
        q_cont_ref(i) = last_q_goal_[i];
      }
    }

    Eigen::Vector3d target(
      target_pose_.position.x,
      target_pose_.position.y,
      target_pose_.position.z
    );

    double final_error = 999.0;
    bool solved = false;

    for (int iter = 0; iter < ik_max_iterations_; ++iter) {
      std::vector<double> q_vec(n);
      for (int i = 0; i < n; ++i) {
        q_vec[i] = q(i);
      }

      state->setJointGroupPositions(jmg, q_vec);
      state->update();

      Eigen::Vector3d current =
        state->getGlobalLinkTransform(ee_link_model).translation();

      Eigen::Vector3d error = target - current;
      final_error = error.norm();

      if (final_error <= ik_position_tolerance_) {
        solved = true;
        break;
      }

      Eigen::MatrixXd jacobian6;
      state->getJacobian(
        jmg,
        ee_link_model,
        Eigen::Vector3d::Zero(),
        jacobian6
      );

      if (jacobian6.rows() < 3 || jacobian6.cols() != n) {
        RCLCPP_ERROR(
          get_logger(),
          "Invalid Jacobian size. rows=%ld cols=%ld expected_cols=%d",
          jacobian6.rows(),
          jacobian6.cols(),
          n
        );
        return false;
      }

      Eigen::MatrixXd J = jacobian6.topRows(3);

      Eigen::MatrixXd JJt = J * J.transpose();
      Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();
      Eigen::MatrixXd J_pinv =
        J.transpose() * (JJt + ik_damping_ * ik_damping_ * I3).inverse();

      Eigen::MatrixXd N =
        Eigen::MatrixXd::Identity(n, n) - J_pinv * J;

      Eigen::VectorXd q_pref_error(n);
      Eigen::VectorXd q_cont_error(n);
      Eigen::VectorXd q_limit_error(n);

      for (int i = 0; i < n; ++i) {
        q_pref_error(i) = angleDiff(q_pref(i), q(i));
        q_cont_error(i) = angleDiff(q_cont_ref(i), q(i));
        q_limit_error(i) = jointLimitError(*state, jmg, i, q(i));
      }

      Eigen::VectorXd weighted_posture_error =
        w_post.asDiagonal() * q_pref_error;

      Eigen::VectorXd weighted_continuity_error =
        w_cont.asDiagonal() * q_cont_error;

      Eigen::VectorXd dq_task =
        ik_step_scale_ * J_pinv * error;

      Eigen::VectorXd dq_null =
          nullspace_gain_ * N * weighted_posture_error
        + continuity_gain_ * N * weighted_continuity_error
        + joint_limit_gain_ * N * q_limit_error;

      Eigen::VectorXd dq = dq_task + dq_null;

      for (int i = 0; i < n; ++i) {
        dq(i) = clamp(dq(i), -max_joint_step_, max_joint_step_);
        q(i) += dq(i);
        q(i) = clampToJointBounds(*state, jmg, i, q(i));
      }
    }

    for (int i = 0; i < n; ++i) {
      double d = angleDiff(q(i), q0(i));
      if (std::abs(d) > max_goal_joint_delta_) {
        q(i) = q0(i) + clamp(d, -max_goal_joint_delta_, max_goal_joint_delta_);
      }
    }

    std::vector<double> q_final(n);
    for (int i = 0; i < n; ++i) {
      q_final[i] = q(i);
    }

    state->setJointGroupPositions(jmg, q_final);
    state->update();

    Eigen::Vector3d current =
      state->getGlobalLinkTransform(ee_link_model).translation();

    final_error = (target - current).norm();

    // Important:
    // solved may have become true inside the IK loop,
    // but q can be changed later by max_goal_joint_delta limiting.
    // Therefore the final reached pose must be checked again here.
    solved = (final_error <= ik_position_tolerance_);

    RCLCPP_INFO(
      get_logger(),
      "Final reached EE position: x=%.4f y=%.4f z=%.4f / target: x=%.4f y=%.4f z=%.4f",
      current.x(),
      current.y(),
      current.z(),
      target.x(),
      target.y(),
      target.z()
    );

    auto variable_names = jmg->getVariableNames();
    if (static_cast<int>(variable_names.size()) != n) {
      RCLCPP_ERROR(
        get_logger(),
        "Variable name size mismatch. names=%zu joints=%d",
        variable_names.size(),
        n
      );
      return false;
    }

    q_goal_map.clear();
    for (int i = 0; i < n; ++i) {
      q_goal_map[variable_names[i]] = q(i);
    }

    RCLCPP_INFO(
      get_logger(),
      "Precision IK result: error=%.4f m, solved=%s",
      final_error,
      solved ? "true" : "false"
    );

    if (!solved) {
      RCLCPP_WARN(
        get_logger(),
        "IK did not reach tolerance %.4f m. Keeping result, but plan may be rejected.",
        ik_position_tolerance_
      );
    }

    return solved;
  }

  double jointLimitError(
    const moveit::core::RobotState & state,
    const moveit::core::JointModelGroup * jmg,
    int index,
    double q_value)
  {
    auto variable_names = jmg->getVariableNames();
    if (index < 0 || index >= static_cast<int>(variable_names.size())) {
      return 0.0;
    }

    auto robot_model = state.getRobotModel();
    if (!robot_model) {
      return 0.0;
    }

    const auto & bounds = robot_model->getVariableBounds(variable_names[index]);
    if (!bounds.position_bounded_) {
      return 0.0;
    }

    const double lo = bounds.min_position_ + joint_limit_margin_;
    const double hi = bounds.max_position_ - joint_limit_margin_;

    if (lo >= hi) {
      return 0.0;
    }

    if (q_value < lo) {
      return lo - q_value;
    }

    if (q_value > hi) {
      return hi - q_value;
    }

    return 0.0;
  }

  double clampToJointBounds(
    const moveit::core::RobotState & state,
    const moveit::core::JointModelGroup * jmg,
    int index,
    double q_value)
  {
    auto variable_names = jmg->getVariableNames();
    if (index < 0 || index >= static_cast<int>(variable_names.size())) {
      return q_value;
    }

    auto robot_model = state.getRobotModel();
    if (!robot_model) {
      return q_value;
    }

    const auto & bounds = robot_model->getVariableBounds(variable_names[index]);
    if (!bounds.position_bounded_) {
      return q_value;
    }

    return clamp(q_value, bounds.min_position_ + 1e-4, bounds.max_position_ - 1e-4);
  }

  void planTargetHighPerformance()
  {
    std::map<std::string, double> q_goal_map;

    if (!computePrecisionIK(q_goal_map)) {
      RCLCPP_ERROR(get_logger(), "High-performance IK failed. No planning will be executed.");
      return;
    }

    move_group_->clearPoseTargets();
    move_group_->setStartStateToCurrentState();

    if (!move_group_->setJointValueTarget(q_goal_map)) {
      RCLCPP_ERROR(get_logger(), "setJointValueTarget failed.");
      return;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = static_cast<bool>(move_group_->plan(plan));

    if (!success) {
      RCLCPP_ERROR(get_logger(), "MoveIt planning failed.");
      return;
    }

    if (!validateTrajectory(plan)) {
      RCLCPP_ERROR(get_logger(), "Trajectory rejected by precision safety validator.");
      return;
    }

    publishDisplayTrajectory(plan);

    if (!execute_plan_) {
      RCLCPP_WARN(
        get_logger(),
        "execute_plan=false. Plan displayed only. Robot/controller will not be commanded by this node."
      );

      if (save_last_goal_on_plan_only_) {
        saveLastGoal(q_goal_map);
      }
      return;
    }

    RCLCPP_WARN(get_logger(), "execute_plan=true. Sending trajectory to controller.");

    auto result = move_group_->execute(plan);
    if (result == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(get_logger(), "Execution succeeded.");
      saveLastGoal(q_goal_map);
    } else {
      RCLCPP_ERROR(get_logger(), "Execution failed.");
    }
  }

  bool validateTrajectory(
    const moveit::planning_interface::MoveGroupInterface::Plan & plan)
  {
    const auto & traj = plan.trajectory_.joint_trajectory;

    if (traj.points.empty()) {
      RCLCPP_ERROR(get_logger(), "Trajectory has no points.");
      return false;
    }

    if (traj.points.front().positions.empty()) {
      RCLCPP_ERROR(get_logger(), "Trajectory point positions are empty.");
      return false;
    }

    for (size_t p = 1; p < traj.points.size(); ++p) {
      const auto & prev = traj.points[p - 1].positions;
      const auto & curr = traj.points[p].positions;

      if (prev.size() != curr.size()) {
        RCLCPP_ERROR(get_logger(), "Trajectory point size mismatch.");
        return false;
      }

      for (size_t j = 0; j < curr.size(); ++j) {
        double jump = std::abs(curr[j] - prev[j]);
        if (jump > max_waypoint_joint_jump_) {
          RCLCPP_ERROR(
            get_logger(),
            "Trajectory rejected: waypoint joint jump too large. point=%zu joint=%zu jump=%.4f rad",
            p,
            j,
            jump
          );
          return false;
        }
      }
    }

    const auto & first = traj.points.front().positions;
    const auto & last = traj.points.back().positions;

    if (first.size() == last.size()) {
      for (size_t j = 0; j < last.size(); ++j) {
        double total_delta = std::abs(last[j] - first[j]);
        if (total_delta > max_total_joint_delta_) {
          RCLCPP_ERROR(
            get_logger(),
            "Trajectory rejected: total joint delta too large. joint=%zu delta=%.4f rad",
            j,
            total_delta
          );
          return false;
        }
      }
    }

    RCLCPP_INFO(
      get_logger(),
      "Trajectory validation passed. points=%zu",
      traj.points.size()
    );

    return true;
  }

  void publishDisplayTrajectory(
    const moveit::planning_interface::MoveGroupInterface::Plan & plan)
  {
    moveit_msgs::msg::DisplayTrajectory display_msg;

    auto current_state = move_group_->getCurrentState(1.0);
    if (current_state) {
      moveit::core::robotStateToRobotStateMsg(
        *current_state,
        display_msg.trajectory_start
      );
    }

    display_msg.trajectory.push_back(plan.trajectory_);
    display_pub_->publish(display_msg);

    RCLCPP_INFO(get_logger(), "Published planned trajectory to /display_planned_path.");
  }

  void saveLastGoal(const std::map<std::string, double> & q_goal_map)
  {
    auto state = move_group_->getCurrentState(1.0);
    if (!state) {
      return;
    }

    const auto * jmg = state->getJointModelGroup(planning_group_);
    if (!jmg) {
      return;
    }

    auto variable_names = jmg->getVariableNames();

    last_q_goal_.clear();
    for (const auto & name : variable_names) {
      auto it = q_goal_map.find(name);
      if (it == q_goal_map.end()) {
        last_q_goal_.clear();
        has_last_q_goal_ = false;
        return;
      }
      last_q_goal_.push_back(it->second);
    }

    has_last_q_goal_ = true;
    RCLCPP_INFO(get_logger(), "Saved q_goal as continuity reference for next target.");
  }

  std::mutex plan_mutex_;

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr target_point_sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_pub_;

  std::string planning_group_;
  std::string ee_link_;
  std::string planner_id_;
  std::string planning_frame_;

  geometry_msgs::msg::Pose target_pose_;

  bool moveit_ready_{false};
  bool execute_plan_{false};
  bool save_last_goal_on_plan_only_{true};

  double planning_time_{4.0};
  int planning_attempts_{5};
  double velocity_scaling_{0.20};
  double accel_scaling_{0.20};

  double workspace_x_min_{-0.70};
  double workspace_x_max_{0.90};
  double workspace_y_min_{-0.70};
  double workspace_y_max_{0.70};
  double workspace_z_min_{-0.20};
  double workspace_z_max_{0.90};

  std::vector<double> preferred_joints_;
  std::vector<double> posture_weights_;
  std::vector<double> continuity_weights_;

  double ik_position_tolerance_{0.003};
  int ik_max_iterations_{180};
  double ik_damping_{0.06};
  double ik_step_scale_{0.30};

  double nullspace_gain_{0.005};
  double continuity_gain_{0.04};
  double joint_limit_gain_{0.01};

  double max_joint_step_{0.015};
  double joint_limit_margin_{0.12};
  double max_goal_joint_delta_{0.06};

  double max_waypoint_joint_jump_{0.08};
  double max_total_joint_delta_{0.70};

  std::vector<double> last_q_goal_;
  bool has_last_q_goal_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<EEPrecisionPosePlanNode>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  std::thread spin_thread([&executor]() {
    executor.spin();
  });

  rclcpp::sleep_for(500ms);
  node->initializeMoveIt();

  spin_thread.join();

  rclcpp::shutdown();
  return 0;
}

#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/robot_model.h>

using namespace std::chrono_literals;

class EEJoyPlanExecuteNode : public rclcpp::Node
{
public:
  EEJoyPlanExecuteNode()
  : Node("ee_joy_plan_execute_node")
  {
    declare_parameter<std::string>("planning_group", "arm");
    declare_parameter<std::string>("ee_link", "gripper_tcp");

    declare_parameter<double>("linear_speed", 0.05);
    declare_parameter<double>("linear_speed_min", 0.01);
    declare_parameter<double>("linear_speed_max", 0.20);
    declare_parameter<double>("linear_speed_step", 0.01);
    declare_parameter<double>("deadzone", 0.15);

    declare_parameter<double>("planning_time", 3.0);
    declare_parameter<int>("planning_attempts", 1);
    declare_parameter<double>("velocity_scaling", 0.70);
    declare_parameter<double>("accel_scaling", 0.70);

    declare_parameter<double>("workspace_x_min", -0.60);
    declare_parameter<double>("workspace_x_max", 0.60);
    declare_parameter<double>("workspace_y_min", -0.60);
    declare_parameter<double>("workspace_y_max", 0.60);
    declare_parameter<double>("workspace_z_min", -0.20);
    declare_parameter<double>("workspace_z_max", 0.80);

    declare_parameter<std::vector<double>>(
      "preferred_joints",
      std::vector<double>{-3.146745, 0.085489, -1.332956, -1.313918, -0.079086, 4.995042}
    );

    // Posture cost weight matrix W diagonal.
    // Larger weight means the joint is more strongly pulled toward preferred_joints.
    declare_parameter<std::vector<double>>(
      "posture_weights",
      std::vector<double>{0.5, 3.0, 3.0, 2.0, 0.5, 0.5}
    );

    declare_parameter<double>("ik_position_tolerance", 0.05);  // 5 cm
    declare_parameter<int>("ik_max_iterations", 25);
    declare_parameter<double>("ik_damping", 0.08);
    declare_parameter<double>("ik_step_scale", 0.70);
    declare_parameter<double>("nullspace_gain", 0.08);
    declare_parameter<double>("max_joint_step", 0.12);

    // 안정화 옵션
    declare_parameter<double>("joint_limit_margin", 0.08);       // limit에서 0.08 rad 안쪽만 사용
    declare_parameter<double>("max_goal_joint_delta", 0.35);     // 한 번 실행당 목표 관절 변화량 제한
    declare_parameter<bool>("enable_position_fallback", true);   // null-space plan 실패 시 기존 position-only로 재시도

    planning_group_ = get_parameter("planning_group").as_string();
    ee_link_ = get_parameter("ee_link").as_string();

    linear_speed_ = get_parameter("linear_speed").as_double();
    linear_speed_min_ = get_parameter("linear_speed_min").as_double();
    linear_speed_max_ = get_parameter("linear_speed_max").as_double();
    linear_speed_step_ = get_parameter("linear_speed_step").as_double();
    deadzone_ = get_parameter("deadzone").as_double();

    workspace_x_min_ = get_parameter("workspace_x_min").as_double();
    workspace_x_max_ = get_parameter("workspace_x_max").as_double();
    workspace_y_min_ = get_parameter("workspace_y_min").as_double();
    workspace_y_max_ = get_parameter("workspace_y_max").as_double();
    workspace_z_min_ = get_parameter("workspace_z_min").as_double();
    workspace_z_max_ = get_parameter("workspace_z_max").as_double();

    preferred_joints_ = get_parameter("preferred_joints").as_double_array();
    posture_weights_ = get_parameter("posture_weights").as_double_array();

    ik_position_tolerance_ = get_parameter("ik_position_tolerance").as_double();
    ik_max_iterations_ = get_parameter("ik_max_iterations").as_int();
    ik_damping_ = get_parameter("ik_damping").as_double();
    ik_step_scale_ = get_parameter("ik_step_scale").as_double();
    nullspace_gain_ = get_parameter("nullspace_gain").as_double();
    max_joint_step_ = get_parameter("max_joint_step").as_double();
    joint_limit_margin_ = get_parameter("joint_limit_margin").as_double();
    max_goal_joint_delta_ = get_parameter("max_goal_joint_delta").as_double();
    enable_position_fallback_ = get_parameter("enable_position_fallback").as_bool();

    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("/ee_target_marker", 10);

    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "/joy",
      10,
      std::bind(&EEJoyPlanExecuteNode::joyCallback, this, std::placeholders::_1)
    );

    timer_ = create_wall_timer(
      33ms,
      std::bind(&EEJoyPlanExecuteNode::timerCallback, this)
    );

    RCLCPP_INFO(get_logger(), "EE joystick null-space IK node created.");
  }

  void initializeMoveIt()
  {
    RCLCPP_INFO(get_logger(), "Initializing MoveGroupInterface...");

    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(),
      planning_group_
    );

    move_group_->setEndEffectorLink(ee_link_);
    move_group_->setPlanningTime(get_parameter("planning_time").as_double());
    move_group_->setNumPlanningAttempts(get_parameter("planning_attempts").as_int());
    move_group_->setMaxVelocityScalingFactor(get_parameter("velocity_scaling").as_double());
    move_group_->setMaxAccelerationScalingFactor(get_parameter("accel_scaling").as_double());
    move_group_->setPlannerId("RRTConnectkConfigDefault");

    planning_frame_ = move_group_->getPlanningFrame();

    move_group_->startStateMonitor(5.0);

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
      rclcpp::sleep_for(200ms);
    }

    if (!pose_ok) {
      RCLCPP_WARN(get_logger(), "Failed to get current EE pose. Using default target pose.");
      target_pose_.orientation.w = 1.0;
      target_pose_.position.x = 0.20;
      target_pose_.position.y = 0.00;
      target_pose_.position.z = 0.30;
    }

    moveit_ready_ = true;

    RCLCPP_INFO(get_logger(), "MoveIt initialized.");
    RCLCPP_INFO(get_logger(), "planning_group : %s", planning_group_.c_str());
    RCLCPP_INFO(get_logger(), "ee_link        : %s", ee_link_.c_str());
    RCLCPP_INFO(get_logger(), "planning_frame : %s", planning_frame_.c_str());

    RCLCPP_INFO(get_logger(), "preferred_joints:");
    for (size_t i = 0; i < preferred_joints_.size(); ++i) {
      RCLCPP_INFO(get_logger(), "  q_pref[%zu] = %.6f", i, preferred_joints_[i]);
    }

    RCLCPP_INFO(get_logger(), "posture_weights:");
    for (size_t i = 0; i < posture_weights_.size(); ++i) {
      RCLCPP_INFO(get_logger(), "  W[%zu] = %.3f", i, posture_weights_[i]);
    }

    RCLCPP_INFO(get_logger(), "Null-space IK params:");
    RCLCPP_INFO(get_logger(), "  ik_position_tolerance = %.3f m", ik_position_tolerance_);
    RCLCPP_INFO(get_logger(), "  ik_max_iterations     = %d", ik_max_iterations_);
    RCLCPP_INFO(get_logger(), "  ik_damping            = %.3f", ik_damping_);
    RCLCPP_INFO(get_logger(), "  ik_step_scale         = %.3f", ik_step_scale_);
    RCLCPP_INFO(get_logger(), "  nullspace_gain        = %.3f", nullspace_gain_);
    RCLCPP_INFO(get_logger(), "  max_joint_step        = %.3f rad", max_joint_step_);
    RCLCPP_INFO(get_logger(), "  joint_limit_margin    = %.3f rad", joint_limit_margin_);
    RCLCPP_INFO(get_logger(), "  max_goal_joint_delta  = %.3f rad", max_goal_joint_delta_);
    RCLCPP_INFO(get_logger(), "  position_fallback     = %s", enable_position_fallback_ ? "true" : "false");
  }

private:
  static double clamp(double v, double lo, double hi)
  {
    return std::max(lo, std::min(hi, v));
  }

  static double angleDiff(double target, double current)
  {
    double d = target - current;
    while (d > M_PI) d -= 2.0 * M_PI;
    while (d < -M_PI) d += 2.0 * M_PI;
    return d;
  }

  double applyDeadzone(double v) const
  {
    if (std::abs(v) < deadzone_) {
      return 0.0;
    }
    return v;
  }

  double getAxis(const std::vector<double> & axes, size_t idx) const
  {
    if (idx >= axes.size()) {
      return 0.0;
    }
    return applyDeadzone(axes[idx]);
  }

  bool getButton(const std::vector<int> & buttons, size_t idx) const
  {
    if (idx >= buttons.size()) {
      return false;
    }
    return buttons[idx] != 0;
  }

  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    last_axes_.assign(msg->axes.begin(), msg->axes.end());

    std::vector<int> new_buttons = msg->buttons;

    auto rising = [&](size_t idx) -> bool {
      bool now = getButton(new_buttons, idx);
      bool prev = getButton(last_buttons_, idx);
      return now && !prev;
    };

    if (rising(0)) request_execute_ = true;      // A
    if (rising(2)) request_reset_ = true;        // X
    if (rising(3)) request_home_ = true;         // Y
    if (rising(4)) request_speed_down_ = true;   // LB
    if (rising(5)) request_speed_up_ = true;     // RB

    last_buttons_ = new_buttons;
    joy_received_ = true;
  }

  void timerCallback()
  {
    if (!moveit_ready_) {
      return;
    }

    std::vector<double> axes;
    bool do_execute = false;
    bool do_reset = false;
    bool do_home = false;
    bool do_speed_up = false;
    bool do_speed_down = false;

    {
      std::lock_guard<std::mutex> lock(mutex_);

      axes = last_axes_;

      do_execute = request_execute_;
      do_reset = request_reset_;
      do_home = request_home_;
      do_speed_up = request_speed_up_;
      do_speed_down = request_speed_down_;

      request_execute_ = false;
      request_reset_ = false;
      request_home_ = false;
      request_speed_up_ = false;
      request_speed_down_ = false;
    }

    if (joy_received_) {
      const double dt = 0.033;

      const double x_cmd = getAxis(axes, 1);
      const double y_cmd = getAxis(axes, 0);
      const double z_cmd = getAxis(axes, 7);

      target_pose_.position.x += x_cmd * linear_speed_ * dt;
      target_pose_.position.y += y_cmd * linear_speed_ * dt;
      target_pose_.position.z += z_cmd * linear_speed_ * dt;

      target_pose_.position.x = clamp(target_pose_.position.x, workspace_x_min_, workspace_x_max_);
      target_pose_.position.y = clamp(target_pose_.position.y, workspace_y_min_, workspace_y_max_);
      target_pose_.position.z = clamp(target_pose_.position.z, workspace_z_min_, workspace_z_max_);
    }

    publishMarker();

    if (do_speed_down) {
      linear_speed_ = clamp(linear_speed_ - linear_speed_step_, linear_speed_min_, linear_speed_max_);
      RCLCPP_INFO(get_logger(), "linear_speed = %.3f m/s", linear_speed_);
    }

    if (do_speed_up) {
      linear_speed_ = clamp(linear_speed_ + linear_speed_step_, linear_speed_min_, linear_speed_max_);
      RCLCPP_INFO(get_logger(), "linear_speed = %.3f m/s", linear_speed_);
    }

    if (do_reset) {
      resetTargetToCurrent();
      return;
    }

    if (do_home) {
      planAndExecuteHome();
      return;
    }

    if (do_execute) {
      planAndExecuteTarget();
      return;
    }
  }

  void publishMarker()
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = planning_frame_.empty() ? "ARM_BASE_LINK" : planning_frame_;
    marker.header.stamp = now();

    marker.ns = "ee_target";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose = target_pose_;

    marker.scale.x = 0.04;
    marker.scale.y = 0.04;
    marker.scale.z = 0.04;

    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 0.85;

    marker_pub_->publish(marker);
  }

  void resetTargetToCurrent()
  {
    if (!move_group_) {
      return;
    }

    try {
      auto current_pose_stamped = move_group_->getCurrentPose(ee_link_);
      target_pose_ = current_pose_stamped.pose;

      RCLCPP_INFO(get_logger(), "Target reset: x=%.3f y=%.3f z=%.3f",
                  target_pose_.position.x,
                  target_pose_.position.y,
                  target_pose_.position.z);
    } catch (...) {
      RCLCPP_ERROR(get_logger(), "Failed to reset target to current EE pose.");
    }
  }

  bool computeNullspaceIK(std::map<std::string, double> & q_goal_map)
  {
    auto state = move_group_->getCurrentState(1.0);
    if (!state) {
      RCLCPP_ERROR(get_logger(), "Failed to get current robot state for null-space IK.");
      return false;
    }

    const moveit::core::JointModelGroup * jmg = state->getJointModelGroup(planning_group_);
    if (!jmg) {
      RCLCPP_ERROR(get_logger(), "JointModelGroup not found: %s", planning_group_.c_str());
      return false;
    }

    const moveit::core::LinkModel * link_model = state->getRobotModel()->getLinkModel(ee_link_);
    if (!link_model) {
      RCLCPP_ERROR(get_logger(), "EE link model not found: %s", ee_link_.c_str());
      return false;
    }

    std::vector<std::string> variable_names = jmg->getVariableNames();
    const int n = static_cast<int>(variable_names.size());

    if (n <= 0) {
      RCLCPP_ERROR(get_logger(), "No joint variables in planning group.");
      return false;
    }

    if (static_cast<int>(preferred_joints_.size()) != n) {
      RCLCPP_ERROR(get_logger(), "preferred_joints size mismatch. expected=%d actual=%zu",
                   n, preferred_joints_.size());
      return false;
    }

    if (static_cast<int>(posture_weights_.size()) != n) {
      RCLCPP_ERROR(get_logger(), "posture_weights size mismatch. expected=%d actual=%zu",
                   n, posture_weights_.size());
      return false;
    }

    std::vector<double> q_vec;
    state->copyJointGroupPositions(jmg, q_vec);

    Eigen::VectorXd q(n);
    Eigen::VectorXd q_pref(n);
    Eigen::VectorXd q_weight(n);

    for (int i = 0; i < n; ++i) {
      q(i) = q_vec[i];
      q_pref(i) = preferred_joints_[i];
      q_weight(i) = posture_weights_[i];
    }

    Eigen::VectorXd q_start = q;

    Eigen::Vector3d target(
      target_pose_.position.x,
      target_pose_.position.y,
      target_pose_.position.z
    );

    bool solved = false;
    double final_error = 999.0;

    for (int iter = 0; iter < ik_max_iterations_; ++iter) {
      std::vector<double> q_tmp(n);
      for (int i = 0; i < n; ++i) {
        q_tmp[i] = q(i);
      }

      state->setJointGroupPositions(jmg, q_tmp);
      state->enforceBounds(jmg);
      state->update();

      Eigen::Vector3d current = state->getGlobalLinkTransform(link_model).translation();
      Eigen::Vector3d error = target - current;
      final_error = error.norm();

      if (final_error <= ik_position_tolerance_) {
        solved = true;
        break;
      }

      Eigen::MatrixXd jacobian6;
      bool jac_ok = state->getJacobian(
        jmg,
        link_model,
        Eigen::Vector3d::Zero(),
        jacobian6
      );

      if (!jac_ok || jacobian6.rows() < 3) {
        RCLCPP_ERROR(get_logger(), "Failed to compute Jacobian.");
        return false;
      }

      Eigen::MatrixXd J = jacobian6.topRows(3);

      Eigen::MatrixXd I_n = Eigen::MatrixXd::Identity(n, n);
      Eigen::MatrixXd I_3 = Eigen::MatrixXd::Identity(3, 3);

      Eigen::MatrixXd JJt = J * J.transpose();
      Eigen::MatrixXd inv = (JJt + ik_damping_ * ik_damping_ * I_3).inverse();
      Eigen::MatrixXd J_pinv = J.transpose() * inv;

      Eigen::VectorXd q_pref_error(n);
      for (int i = 0; i < n; ++i) {
        q_pref_error(i) = angleDiff(q_pref(i), q(i));
      }

      Eigen::MatrixXd N = I_n - J_pinv * J;

      Eigen::VectorXd dq_task = ik_step_scale_ * J_pinv * error;

      // Weighted posture cost:
      // H(q) = 0.5 * (q - q_ref)^T * W * (q - q_ref)
      // -grad(H) direction is W * (q_ref - q)
      Eigen::VectorXd weighted_posture_error = q_weight.asDiagonal() * q_pref_error;
      Eigen::VectorXd dq_null = nullspace_gain_ * N * weighted_posture_error;

      Eigen::VectorXd dq = dq_task + dq_null;

      for (int i = 0; i < n; ++i) {
        dq(i) = clamp(dq(i), -max_joint_step_, max_joint_step_);
        q(i) += dq(i);
      }

      // Joint limit clamp with margin
      const moveit::core::RobotModelConstPtr & model = state->getRobotModel();
      for (int i = 0; i < n; ++i) {
        const auto & bounds = model->getVariableBounds(variable_names[i]);
        if (bounds.position_bounded_) {
          double lo = bounds.min_position_ + joint_limit_margin_;
          double hi = bounds.max_position_ - joint_limit_margin_;

          if (lo < hi) {
            q(i) = clamp(q(i), lo, hi);
          } else {
            q(i) = clamp(q(i), bounds.min_position_, bounds.max_position_);
          }
        }
      }
    }

    // Final q_goal safety clamp:
    // 1) 현재 관절에서 너무 멀리 가지 않게 제한
    // 2) joint limit 끝단에 붙지 않게 margin 적용
    const moveit::core::RobotModelConstPtr & model = state->getRobotModel();

    for (int i = 0; i < n; ++i) {
      q(i) = clamp(
        q(i),
        q_start(i) - max_goal_joint_delta_,
        q_start(i) + max_goal_joint_delta_
      );

      const auto & bounds = model->getVariableBounds(variable_names[i]);
      if (bounds.position_bounded_) {
        double lo = bounds.min_position_ + joint_limit_margin_;
        double hi = bounds.max_position_ - joint_limit_margin_;

        if (lo < hi) {
          q(i) = clamp(q(i), lo, hi);
        } else {
          q(i) = clamp(q(i), bounds.min_position_, bounds.max_position_);
        }
      }
    }

    std::vector<double> q_result(n);
    for (int i = 0; i < n; ++i) {
      q_result[i] = q(i);
    }

    state->setJointGroupPositions(jmg, q_result);
    state->enforceBounds(jmg);
    state->update();

    Eigen::Vector3d current = state->getGlobalLinkTransform(link_model).translation();
    final_error = (target - current).norm();

    if (final_error <= ik_position_tolerance_) {
      solved = true;
    }

    RCLCPP_INFO(get_logger(), "Null-space IK result: error=%.3f m, solved=%s",
                final_error,
                solved ? "true" : "false");

    if (!solved) {
      RCLCPP_WARN(get_logger(), "IK did not reach tolerance, but using closest q_goal.");
    }

    std::vector<double> final_q;
    state->copyJointGroupPositions(jmg, final_q);

    q_goal_map.clear();
    for (size_t i = 0; i < variable_names.size(); ++i) {
      q_goal_map[variable_names[i]] = final_q[i];
    }

    RCLCPP_INFO(get_logger(), "q_goal:");
    for (const auto & name : variable_names) {
      RCLCPP_INFO(get_logger(), "  %s = %.6f", name.c_str(), q_goal_map[name]);
    }

    return true;
  }

  bool planAndExecutePositionFallback()
  {
    if (!move_group_) {
      return false;
    }

    RCLCPP_WARN(get_logger(), "Trying position-only fallback...");

    move_group_->clearPoseTargets();
    move_group_->setStartStateToCurrentState();

    double fallback_tol = ik_position_tolerance_;
    if (fallback_tol < 0.07) {
      fallback_tol = 0.07;
    }

    move_group_->setGoalPositionTolerance(fallback_tol);
    move_group_->setGoalOrientationTolerance(3.14);

    move_group_->setPositionTarget(
      target_pose_.position.x,
      target_pose_.position.y,
      target_pose_.position.z,
      ee_link_
    );

    moveit::planning_interface::MoveGroupInterface::Plan fallback_plan;
    bool success = static_cast<bool>(move_group_->plan(fallback_plan));

    if (!success) {
      RCLCPP_ERROR(get_logger(), "Position-only fallback planning failed.");
      move_group_->clearPoseTargets();
      return false;
    }

    RCLCPP_INFO(get_logger(), "Position-only fallback planning succeeded. Executing...");

    auto result = move_group_->execute(fallback_plan);
    move_group_->clearPoseTargets();

    if (result == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(get_logger(), "Position-only fallback execution succeeded.");
      resetTargetToCurrent();
      return true;
    }

    RCLCPP_ERROR(get_logger(), "Position-only fallback execution failed.");
    return false;
  }

  void planAndExecuteTarget()
  {
    if (!move_group_) {
      return;
    }

    RCLCPP_INFO(get_logger(), "Plan+Execute target marker: x=%.3f y=%.3f z=%.3f",
                target_pose_.position.x,
                target_pose_.position.y,
                target_pose_.position.z);

    std::map<std::string, double> q_goal_map;

    if (!computeNullspaceIK(q_goal_map)) {
      RCLCPP_ERROR(get_logger(), "Null-space IK failed.");

      if (enable_position_fallback_) {
        planAndExecutePositionFallback();
      }

      return;
    }

    move_group_->clearPoseTargets();
    move_group_->setStartStateToCurrentState();

    bool target_ok = move_group_->setJointValueTarget(q_goal_map);

    if (!target_ok) {
      RCLCPP_ERROR(get_logger(), "setJointValueTarget failed.");

      if (enable_position_fallback_) {
        planAndExecutePositionFallback();
      }

      return;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;

    bool success = static_cast<bool>(move_group_->plan(plan));

    if (!success) {
      RCLCPP_WARN(get_logger(), "Weighted null-space joint planning failed.");

      if (enable_position_fallback_) {
        planAndExecutePositionFallback();
      } else {
        RCLCPP_ERROR(get_logger(), "Planning failed.");
      }

      return;
    }

    RCLCPP_INFO(get_logger(), "Weighted null-space planning succeeded. Executing...");

    auto result = move_group_->execute(plan);

    if (result == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(get_logger(), "Execution succeeded.");
      resetTargetToCurrent();
    } else {
      RCLCPP_ERROR(get_logger(), "Execution failed.");
    }
  }

  void planAndExecuteHome()
  {
    if (!move_group_) {
      return;
    }

    RCLCPP_INFO(get_logger(), "Plan+Execute named target: home");

    move_group_->setStartStateToCurrentState();
    move_group_->setNamedTarget("home");

    moveit::planning_interface::MoveGroupInterface::Plan plan;

    bool success = static_cast<bool>(move_group_->plan(plan));

    if (!success) {
      RCLCPP_ERROR(get_logger(), "Home planning failed.");
      return;
    }

    auto result = move_group_->execute(plan);

    if (result == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(get_logger(), "Home execution succeeded.");
      resetTargetToCurrent();
    } else {
      RCLCPP_ERROR(get_logger(), "Home execution failed.");
    }
  }

private:
  std::string planning_group_;
  std::string ee_link_;
  std::string planning_frame_;

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  bool moveit_ready_ = false;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex mutex_;
  std::vector<double> last_axes_;
  std::vector<int> last_buttons_;
  bool joy_received_ = false;

  bool request_execute_ = false;
  bool request_reset_ = false;
  bool request_home_ = false;
  bool request_speed_up_ = false;
  bool request_speed_down_ = false;

  geometry_msgs::msg::Pose target_pose_;

  std::vector<double> preferred_joints_;
  std::vector<double> posture_weights_;

  double linear_speed_;
  double linear_speed_min_;
  double linear_speed_max_;
  double linear_speed_step_;
  double deadzone_;

  double workspace_x_min_;
  double workspace_x_max_;
  double workspace_y_min_;
  double workspace_y_max_;
  double workspace_z_min_;
  double workspace_z_max_;

  double ik_position_tolerance_;
  int ik_max_iterations_;
  double ik_damping_;
  double ik_step_scale_;
  double nullspace_gain_;
  double max_joint_step_;
  double joint_limit_margin_;
  double max_goal_joint_delta_;
  bool enable_position_fallback_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<EEJoyPlanExecuteNode>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  std::thread spin_thread([&executor]() {
    executor.spin();
  });

  rclcpp::sleep_for(2s);
  node->initializeMoveIt();

  spin_thread.join();

  rclcpp::shutdown();
  return 0;
}

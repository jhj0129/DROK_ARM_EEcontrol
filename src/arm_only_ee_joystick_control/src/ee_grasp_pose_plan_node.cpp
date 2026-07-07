#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/msg/display_trajectory.hpp>

using namespace std::chrono_literals;

class EEGraspPosePlanNode : public rclcpp::Node
{
public:
  EEGraspPosePlanNode()
  : Node("ee_grasp_pose_plan_node")
  {
    planning_group_ = declare_parameter<std::string>("planning_group", "arm");
    ee_link_ = declare_parameter<std::string>("ee_link", "gripper_tcp");
    planner_id_ = declare_parameter<std::string>("planner_id", "RRTConnectkConfigDefault");

    execute_plan_ = declare_parameter<bool>("execute_plan", false);

    ik_position_tolerance_ = declare_parameter<double>("ik_position_tolerance", 0.010);
    ik_orientation_tolerance_ = declare_parameter<double>("ik_orientation_tolerance", 0.120);
    ik_max_iterations_ = declare_parameter<int>("ik_max_iterations", 1400);
    ik_damping_ = declare_parameter<double>("ik_damping", 0.035);
    ik_step_scale_ = declare_parameter<double>("ik_step_scale", 0.35);
    max_joint_step_ = declare_parameter<double>("max_joint_step", 0.060);

    position_weight_ = declare_parameter<double>("position_weight", 1.0);
    orientation_weight_ = declare_parameter<double>("orientation_weight", 0.55);

    stage2_base_joint_scale_ = declare_parameter<double>("stage2_base_joint_scale", 0.12);
    stage2_wrist_joint_scale_ = declare_parameter<double>("stage2_wrist_joint_scale", 1.00);

    planning_time_ = declare_parameter<double>("planning_time", 12.0);
    planning_attempts_ = declare_parameter<int>("planning_attempts", 20);
    velocity_scaling_ = declare_parameter<double>("velocity_scaling", 0.12);
    accel_scaling_ = declare_parameter<double>("accel_scaling", 0.12);

    max_waypoint_joint_jump_ = declare_parameter<double>("max_waypoint_joint_jump", 1.20);

    workspace_x_min_ = declare_parameter<double>("workspace_x_min", -1.50);
    workspace_x_max_ = declare_parameter<double>("workspace_x_max",  1.50);
    workspace_y_min_ = declare_parameter<double>("workspace_y_min", -1.50);
    workspace_y_max_ = declare_parameter<double>("workspace_y_max",  1.50);
    workspace_z_min_ = declare_parameter<double>("workspace_z_min", -1.00);
    workspace_z_max_ = declare_parameter<double>("workspace_z_max",  1.50);

    tool_axis_name_ = declare_parameter<std::string>("tool_axis", "x");
    approach_direction_world_vec_ =
      declare_parameter<std::vector<double>>("approach_direction_world", std::vector<double>{0.0, 0.0, -1.0});

    point_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "/ee_grasp/target_point",
      10,
      std::bind(&EEGraspPosePlanNode::onTargetPoint, this, std::placeholders::_1)
    );

    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/ee_grasp/target_pose",
      10,
      std::bind(&EEGraspPosePlanNode::onTargetPose, this, std::placeholders::_1)
    );

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states",
      50,
      std::bind(&EEGraspPosePlanNode::onJointState, this, std::placeholders::_1)
    );

    display_pub_ = create_publisher<moveit_msgs::msg::DisplayTrajectory>("/display_planned_path", 1);
    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("/ee_grasp/target_marker", 1);

    RCLCPP_WARN(
      get_logger(),
      "SAFE DEFAULT: execute_plan=false. This node plans/displays only unless explicitly enabled."
    );

    init_timer_ = create_wall_timer(
      500ms,
      std::bind(&EEGraspPosePlanNode::initializeMoveIt, this)
    );
  }

private:
  void initializeMoveIt()
  {
    if (move_group_) {
      return;
    }

    RCLCPP_INFO(get_logger(), "Initializing MoveGroupInterface...");

    move_group_ =
      std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(),
        planning_group_
      );

    move_group_->setEndEffectorLink(ee_link_);
    move_group_->setPlannerId(planner_id_);
    move_group_->setPlanningTime(planning_time_);
    move_group_->setNumPlanningAttempts(planning_attempts_);
    move_group_->setMaxVelocityScalingFactor(velocity_scaling_);
    move_group_->setMaxAccelerationScalingFactor(accel_scaling_);

    // Start MoveIt's current state monitor early.
    // Without this warm-up, the first grasp target can arrive before MoveIt has
    // received a fresh /joint_states sample, causing:
    // "latest received state has time 0.000000" and "Current robot state unavailable".
    RCLCPP_INFO(get_logger(), "Starting MoveIt current state monitor...");
    move_group_->startStateMonitor(1.0);
    RCLCPP_WARN(
      get_logger(),
      "This node will use its own /joint_states cache for grasp IK start state."
    );

    robot_model_ = move_group_->getRobotModel();
    jmg_ = robot_model_->getJointModelGroup(planning_group_);
    planning_frame_ = move_group_->getPlanningFrame();

    if (!jmg_) {
      RCLCPP_ERROR(get_logger(), "JointModelGroup [%s] not found.", planning_group_.c_str());
      return;
    }

    tool_axis_local_ = toolAxisFromName(tool_axis_name_);
    approach_direction_world_ = vectorFromParam(approach_direction_world_vec_, Eigen::Vector3d(0, 0, -1));

    RCLCPP_INFO(get_logger(), "MoveIt initialized.");
    RCLCPP_INFO(get_logger(), "  planning_group = %s", planning_group_.c_str());
    RCLCPP_INFO(get_logger(), "  ee_link        = %s", ee_link_.c_str());
    RCLCPP_INFO(get_logger(), "  planning_frame = %s", planning_frame_.c_str());
    RCLCPP_INFO(get_logger(), "  execute_plan   = %s", execute_plan_ ? "true" : "false");
    RCLCPP_INFO(get_logger(), "  tool_axis      = %s", tool_axis_name_.c_str());
    RCLCPP_INFO(
      get_logger(),
      "  approach_world = [%.3f %.3f %.3f]",
      approach_direction_world_.x(),
      approach_direction_world_.y(),
      approach_direction_world_.z()
    );
    RCLCPP_INFO(get_logger(), "  ik_position_tolerance    = %.4f m", ik_position_tolerance_);
    RCLCPP_INFO(get_logger(), "  ik_orientation_tolerance = %.4f rad", ik_orientation_tolerance_);

    init_timer_->cancel();
  }

  void onJointState(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    latest_joint_state_ = *msg;
    have_joint_state_ = true;
  }

  std::shared_ptr<moveit::core::RobotState> getCachedRobotState(double max_age_sec)
  {
    if (!robot_model_) {
      RCLCPP_ERROR(get_logger(), "Robot model is not initialized.");
      return nullptr;
    }

    sensor_msgs::msg::JointState js;
    {
      std::lock_guard<std::mutex> lock(joint_state_mutex_);
      if (!have_joint_state_) {
        RCLCPP_ERROR(get_logger(), "No cached /joint_states received yet.");
        return nullptr;
      }
      js = latest_joint_state_;
    }

    const rclcpp::Time stamp(js.header.stamp);
    const rclcpp::Time now_time = now();

    if (stamp.nanoseconds() > 0) {
      const double age = (now_time - stamp).seconds();
      if (age > max_age_sec) {
        RCLCPP_ERROR(
          get_logger(),
          "Cached /joint_states is too old: age=%.3f sec, limit=%.3f sec",
          age,
          max_age_sec
        );
        return nullptr;
      }
    } else {
      RCLCPP_WARN(get_logger(), "Cached /joint_states stamp is zero. Using it anyway.");
    }

    auto state = std::make_shared<moveit::core::RobotState>(robot_model_);
    state->setToDefaultValues();

    std::size_t used = 0;
    for (std::size_t i = 0; i < js.name.size() && i < js.position.size(); ++i) {
      const std::string& name = js.name[i];
      const auto& vars = robot_model_->getVariableNames();
      if (std::find(vars.begin(), vars.end(), name) != vars.end()) {
        state->setVariablePosition(name, js.position[i]);
        ++used;
      }
    }

    state->update();

    RCLCPP_INFO(
      get_logger(),
      "Using cached /joint_states for start state. joints_used=%zu stamp=%.3f",
      used,
      stamp.seconds()
    );

    return state;
  }

  Eigen::Vector3d vectorFromParam(
    const std::vector<double>& v,
    const Eigen::Vector3d& fallback)
  {
    if (v.size() != 3) {
      return fallback.normalized();
    }
    Eigen::Vector3d out(v[0], v[1], v[2]);
    if (out.norm() < 1e-9) {
      return fallback.normalized();
    }
    return out.normalized();
  }

  Eigen::Vector3d toolAxisFromName(const std::string& name)
  {
    if (name == "x")  return Eigen::Vector3d( 1,  0,  0);
    if (name == "-x") return Eigen::Vector3d(-1,  0,  0);
    if (name == "y")  return Eigen::Vector3d( 0,  1,  0);
    if (name == "-y") return Eigen::Vector3d( 0, -1,  0);
    if (name == "z")  return Eigen::Vector3d( 0,  0,  1);
    if (name == "-z") return Eigen::Vector3d( 0,  0, -1);

    RCLCPP_WARN(get_logger(), "Unknown tool_axis [%s]. Falling back to x.", name.c_str());
    return Eigen::Vector3d(1, 0, 0);
  }

  Eigen::Quaterniond makeAlignmentQuaternion(
    const Eigen::Vector3d& local_axis,
    const Eigen::Vector3d& desired_world_axis)
  {
    Eigen::Vector3d a = local_axis.normalized();
    Eigen::Vector3d b = desired_world_axis.normalized();

    Eigen::Quaterniond q;
    q.setFromTwoVectors(a, b);
    q.normalize();
    return q;
  }

  Eigen::Vector3d clampWorkspace(const Eigen::Vector3d& p)
  {
    Eigen::Vector3d out = p;
    out.x() = std::clamp(out.x(), workspace_x_min_, workspace_x_max_);
    out.y() = std::clamp(out.y(), workspace_y_min_, workspace_y_max_);
    out.z() = std::clamp(out.z(), workspace_z_min_, workspace_z_max_);
    return out;
  }

  void onTargetPoint(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    if (!move_group_) {
      RCLCPP_WARN(get_logger(), "MoveIt is not initialized yet.");
      return;
    }

    Eigen::Vector3d target(
      msg->point.x,
      msg->point.y,
      msg->point.z
    );
    target = clampWorkspace(target);

    Eigen::Quaterniond target_q =
      makeAlignmentQuaternion(tool_axis_local_, approach_direction_world_);

    RCLCPP_INFO(
      get_logger(),
      "Received grasp target point frame [%s]: x=%.3f y=%.3f z=%.3f",
      msg->header.frame_id.c_str(),
      target.x(),
      target.y(),
      target.z()
    );

    planThreeStageGrasp(target, target_q);
  }

  void onTargetPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!move_group_) {
      RCLCPP_WARN(get_logger(), "MoveIt is not initialized yet.");
      return;
    }

    Eigen::Vector3d target(
      msg->pose.position.x,
      msg->pose.position.y,
      msg->pose.position.z
    );
    target = clampWorkspace(target);

    Eigen::Quaterniond target_q(
      msg->pose.orientation.w,
      msg->pose.orientation.x,
      msg->pose.orientation.y,
      msg->pose.orientation.z
    );

    if (target_q.norm() < 1e-9) {
      RCLCPP_WARN(get_logger(), "Invalid quaternion. Falling back to point-mode orientation.");
      target_q = makeAlignmentQuaternion(tool_axis_local_, approach_direction_world_);
    }
    target_q.normalize();

    RCLCPP_INFO(
      get_logger(),
      "Received grasp target pose frame [%s]: x=%.3f y=%.3f z=%.3f",
      msg->header.frame_id.c_str(),
      target.x(),
      target.y(),
      target.z()
    );

    planThreeStageGrasp(target, target_q);
  }

  Eigen::Vector3d orientationError(
    const Eigen::Matrix3d& current_R,
    const Eigen::Matrix3d& desired_R)
  {
    // IMPORTANT:
    // We do NOT constrain the full TCP quaternion here.
    // For grasping, we only need the gripper approach axis to align
    // with the desired approach direction.
    //
    // current_axis: current gripper approach axis in world frame
    // desired_axis: desired gripper approach axis in world frame
    //
    // This keeps roll around the approach axis free, making the IK
    // much less over-constrained and more human-wrist-like.
    Eigen::Vector3d current_axis = (current_R * tool_axis_local_).normalized();
    Eigen::Vector3d desired_axis = (desired_R * tool_axis_local_).normalized();

    if (!current_axis.allFinite() || !desired_axis.allFinite()) {
      return Eigen::Vector3d::Zero();
    }

    Eigen::Vector3d err = current_axis.cross(desired_axis);

    if (!err.allFinite()) {
      return Eigen::Vector3d::Zero();
    }

    return err;
  }

  bool runIKStage(
    moveit::core::RobotState& state,
    const Eigen::Vector3d& target_pos,
    const Eigen::Quaterniond& target_q,
    bool use_position,
    bool use_orientation,
    double pos_weight,
    double ori_weight,
    const std::vector<double>& joint_column_scales,
    int max_iterations,
    const std::string& stage_name,
    double& final_pos_error,
    double& final_ori_error)
  {
    const auto variable_names = jmg_->getVariableNames();
    const std::size_t n = variable_names.size();

    if (n == 0) {
      RCLCPP_ERROR(get_logger(), "No variables in JointModelGroup.");
      return false;
    }

    std::vector<double> q;
    state.copyJointGroupPositions(jmg_, q);

    Eigen::Matrix3d target_R = target_q.normalized().toRotationMatrix();

    bool solved = false;

    for (int iter = 0; iter < max_iterations; ++iter) {
      state.setJointGroupPositions(jmg_, q);
      state.update();

      const Eigen::Isometry3d& ee_tf = state.getGlobalLinkTransform(ee_link_);
      Eigen::Vector3d current_pos = ee_tf.translation();
      Eigen::Matrix3d current_R = ee_tf.rotation();

      Eigen::Vector3d e_pos = target_pos - current_pos;
      Eigen::Vector3d e_ori = orientationError(current_R, target_R);

      final_pos_error = e_pos.norm();
      final_ori_error = e_ori.norm();

      bool pos_ok = (!use_position) || (final_pos_error <= ik_position_tolerance_);
      bool ori_ok = (!use_orientation) || (final_ori_error <= ik_orientation_tolerance_);

      if (pos_ok && ori_ok) {
        solved = true;
        break;
      }

      Eigen::MatrixXd jacobian;
      state.getJacobian(
        jmg_,
        state.getLinkModel(ee_link_),
        Eigen::Vector3d::Zero(),
        jacobian
      );

      int rows = 0;
      if (use_position) {
        rows += 3;
      }
      if (use_orientation) {
        rows += 3;
      }

      Eigen::MatrixXd J(rows, n);
      Eigen::VectorXd e(rows);

      int r = 0;
      if (use_position) {
        for (int i = 0; i < 3; ++i) {
          J.row(r) = pos_weight * jacobian.row(i);
          e(r) = pos_weight * e_pos(i);
          ++r;
        }
      }

      if (use_orientation) {
        for (int i = 0; i < 3; ++i) {
          J.row(r) = ori_weight * jacobian.row(i + 3);
          e(r) = ori_weight * e_ori(i);
          ++r;
        }
      }

      Eigen::MatrixXd A =
        J * J.transpose()
        + ik_damping_ * ik_damping_ * Eigen::MatrixXd::Identity(rows, rows);

      Eigen::VectorXd dq = J.transpose() * A.ldlt().solve(e);
      dq *= ik_step_scale_;

      for (std::size_t i = 0; i < n; ++i) {
        double scale = 1.0;
        if (i < joint_column_scales.size()) {
          scale = joint_column_scales[i];
        }

        dq(static_cast<int>(i)) =
          std::clamp(dq(static_cast<int>(i)), -max_joint_step_, max_joint_step_);

        dq(static_cast<int>(i)) *= scale;

        q[i] += dq(static_cast<int>(i));

        const auto& bounds = robot_model_->getVariableBounds(variable_names[i]);
        if (bounds.position_bounded_) {
          q[i] = std::clamp(q[i], bounds.min_position_, bounds.max_position_);
        }
      }
    }

    state.setJointGroupPositions(jmg_, q);
    state.update();

    const Eigen::Isometry3d& ee_tf = state.getGlobalLinkTransform(ee_link_);
    Eigen::Vector3d current_pos = ee_tf.translation();
    Eigen::Matrix3d current_R = ee_tf.rotation();

    final_pos_error = (target_pos - current_pos).norm();
    final_ori_error = orientationError(current_R, target_R).norm();

    bool pos_ok = (!use_position) || (final_pos_error <= ik_position_tolerance_);
    bool ori_ok = (!use_orientation) || (final_ori_error <= ik_orientation_tolerance_);
    solved = pos_ok && ori_ok;

    RCLCPP_INFO(
      get_logger(),
      "[%s] final pos_error=%.4f m, ori_error=%.4f rad, solved=%s",
      stage_name.c_str(),
      final_pos_error,
      final_ori_error,
      solved ? "true" : "false"
    );

    return solved;
  }

  std::vector<double> makeStage2JointScales(std::size_t n)
  {
    std::vector<double> scales(n, 1.0);

    // 관절 순서가 6축 arm이라고 가정:
    // 0,1,2 = 팔 / 3,4,5 = 손목
    // 4,5,6번 관절 중심 orientation refine을 만들기 위해 앞쪽 관절 변화는 줄이고,
    // 뒤쪽 손목 관절 변화는 크게 허용한다.
    for (std::size_t i = 0; i < n; ++i) {
      if (i < 3) {
        scales[i] = stage2_base_joint_scale_;
      } else {
        scales[i] = stage2_wrist_joint_scale_;
      }
    }

    return scales;
  }

  bool validateTrajectory(const moveit::planning_interface::MoveGroupInterface::Plan& plan)
  {
    const auto& traj = plan.trajectory_.joint_trajectory;

    if (traj.points.empty()) {
      RCLCPP_ERROR(get_logger(), "Trajectory rejected: empty trajectory.");
      return false;
    }

    for (std::size_t i = 1; i < traj.points.size(); ++i) {
      const auto& prev = traj.points[i - 1].positions;
      const auto& curr = traj.points[i].positions;

      if (prev.size() != curr.size()) {
        RCLCPP_ERROR(get_logger(), "Trajectory rejected: joint size mismatch.");
        return false;
      }

      for (std::size_t j = 0; j < curr.size(); ++j) {
        double jump = std::abs(curr[j] - prev[j]);
        if (jump > max_waypoint_joint_jump_) {
          RCLCPP_ERROR(
            get_logger(),
            "Trajectory rejected: waypoint jump too large. point=%zu joint=%zu jump=%.4f rad limit=%.4f",
            i,
            j,
            jump,
            max_waypoint_joint_jump_
          );
          return false;
        }
      }
    }

    RCLCPP_INFO(get_logger(), "Trajectory validation passed. points=%zu", traj.points.size());
    return true;
  }

  void publishTargetMarker(
    const Eigen::Vector3d& target,
    const Eigen::Quaterniond& target_q)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = planning_frame_;
    marker.header.stamp = now();
    marker.ns = "ee_grasp_target";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = target.x();
    marker.pose.position.y = target.y();
    marker.pose.position.z = target.z();
    marker.pose.orientation.x = target_q.x();
    marker.pose.orientation.y = target_q.y();
    marker.pose.orientation.z = target_q.z();
    marker.pose.orientation.w = target_q.w();

    marker.scale.x = 0.18;
    marker.scale.y = 0.025;
    marker.scale.z = 0.025;

    marker.color.r = 0.1;
    marker.color.g = 0.6;
    marker.color.b = 1.0;
    marker.color.a = 0.9;

    marker_pub_->publish(marker);
  }

  void publishDisplayTrajectory(
    const moveit::planning_interface::MoveGroupInterface::Plan& plan)
  {
    auto current_state = getCachedRobotState(5.0);
    if (!current_state) {
      RCLCPP_WARN(get_logger(), "Cannot publish DisplayTrajectory: cached joint state unavailable.");
      return;
    }

    moveit_msgs::msg::DisplayTrajectory display;
    display.model_id = robot_model_->getName();
    moveit::core::robotStateToRobotStateMsg(*current_state, display.trajectory_start);
    display.trajectory.push_back(plan.trajectory_);

    display_pub_->publish(display);
    RCLCPP_INFO(get_logger(), "Published planned trajectory to /display_planned_path.");
  }

  void planThreeStageGrasp(
    const Eigen::Vector3d& target_pos,
    const Eigen::Quaterniond& target_q)
  {
    auto current_state = getCachedRobotState(5.0);
    if (!current_state) {
      RCLCPP_ERROR(get_logger(), "Cached robot state unavailable.");
      return;
    }

    moveit::core::RobotState ik_state(*current_state);

    const std::size_t n = jmg_->getVariableNames().size();
    std::vector<double> all_joint_scales(n, 1.0);
    std::vector<double> wrist_scales = makeStage2JointScales(n);

    double pos_err = 0.0;
    double ori_err = 0.0;

    RCLCPP_INFO(get_logger(), "Starting 3-stage grasp IK.");

    // 1단계: 전체 팔 position IK
    bool s1 = runIKStage(
      ik_state,
      target_pos,
      target_q,
      true,
      false,
      1.0,
      0.0,
      all_joint_scales,
      ik_max_iterations_ / 3,
      "Stage1 full position IK",
      pos_err,
      ori_err
    );

    if (!s1) {
      RCLCPP_WARN(get_logger(), "Stage1 did not fully solve. Continuing to orientation refine anyway.");
    }

    // 2단계: 손목 중심 orientation refine
    bool s2 = runIKStage(
      ik_state,
      target_pos,
      target_q,
      true,
      true,
      0.85,
      0.55,
      wrist_scales,
      ik_max_iterations_ / 3,
      "Stage2 wrist orientation refine with position hold",
      pos_err,
      ori_err
    );

    if (!s2) {
      RCLCPP_WARN(get_logger(), "Stage2 did not fully solve. Continuing to final correction anyway.");
    }

    // 3단계: 전체 팔 pose IK 최종 보정
    bool s3 = runIKStage(
      ik_state,
      target_pos,
      target_q,
      true,
      true,
      position_weight_,
      orientation_weight_,
      all_joint_scales,
      ik_max_iterations_ / 3,
      "Stage3 full pose correction",
      pos_err,
      ori_err
    );

    RCLCPP_INFO(
      get_logger(),
      "3-stage IK result: pos_error=%.4f m, ori_error=%.4f rad, solved=%s",
      pos_err,
      ori_err,
      s3 ? "true" : "false"
    );

    publishTargetMarker(target_pos, target_q);

    if (!s3) {
      RCLCPP_ERROR(get_logger(), "3-stage grasp IK failed. No planning will be executed.");
      return;
    }

    std::vector<double> q_goal;
    ik_state.copyJointGroupPositions(jmg_, q_goal);

    std::map<std::string, double> q_goal_map;
    const auto variable_names = jmg_->getVariableNames();

    for (std::size_t i = 0; i < variable_names.size(); ++i) {
      q_goal_map[variable_names[i]] = q_goal[i];
    }

    move_group_->setStartState(*current_state);
    bool target_ok = move_group_->setJointValueTarget(q_goal_map);

    if (!target_ok) {
      RCLCPP_ERROR(get_logger(), "setJointValueTarget failed.");
      return;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto result = move_group_->plan(plan);

    bool success =
      (result == moveit::core::MoveItErrorCode::SUCCESS);

    if (!success) {
      RCLCPP_ERROR(get_logger(), "MoveIt planning failed.");
      return;
    }

    if (!validateTrajectory(plan)) {
      return;
    }

    publishDisplayTrajectory(plan);

    if (execute_plan_) {
      RCLCPP_WARN(get_logger(), "execute_plan=true. Executing planned trajectory.");
      auto exec_result = move_group_->execute(plan);
      if (exec_result == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(get_logger(), "Execution success.");
      } else {
        RCLCPP_ERROR(get_logger(), "Execution failed.");
      }
    } else {
      RCLCPP_WARN(
        get_logger(),
        "execute_plan=false. Plan displayed only. Robot/controller will not be commanded by this node."
      );
    }
  }

private:
  std::string planning_group_;
  std::string ee_link_;
  std::string planner_id_;
  std::string planning_frame_;

  bool execute_plan_{false};

  double ik_position_tolerance_{0.010};
  double ik_orientation_tolerance_{0.120};
  int ik_max_iterations_{1400};
  double ik_damping_{0.035};
  double ik_step_scale_{0.35};
  double max_joint_step_{0.060};

  double position_weight_{1.0};
  double orientation_weight_{0.55};

  double stage2_base_joint_scale_{0.12};
  double stage2_wrist_joint_scale_{1.00};

  double planning_time_{12.0};
  int planning_attempts_{20};
  double velocity_scaling_{0.12};
  double accel_scaling_{0.12};

  double max_waypoint_joint_jump_{1.20};

  double workspace_x_min_{-1.50};
  double workspace_x_max_{ 1.50};
  double workspace_y_min_{-1.50};
  double workspace_y_max_{ 1.50};
  double workspace_z_min_{-1.00};
  double workspace_z_max_{ 1.50};

  std::string tool_axis_name_{"x"};
  std::vector<double> approach_direction_world_vec_{0.0, 0.0, -1.0};

  Eigen::Vector3d tool_axis_local_{1, 0, 0};
  Eigen::Vector3d approach_direction_world_{0, 0, -1};

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  moveit::core::RobotModelConstPtr robot_model_;
  const moveit::core::JointModelGroup* jmg_{nullptr};

  rclcpp::TimerBase::SharedPtr init_timer_;

  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  std::mutex joint_state_mutex_;
  sensor_msgs::msg::JointState latest_joint_state_;
  bool have_joint_state_{false};

  rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<EEGraspPosePlanNode>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

using namespace std::chrono_literals;

class EEObstacleWaypointPlanNode : public rclcpp::Node
{
public:
  EEObstacleWaypointPlanNode()
  : Node("ee_obstacle_waypoint_plan_node")
  {
    planning_group_ = declare_parameter<std::string>("planning_group", "arm");
    ee_link_ = declare_parameter<std::string>("ee_link", "gripper_tcp");
    planner_id_ = declare_parameter<std::string>("planner_id", "RRTConnectkConfigDefault");

    execute_plan_ = declare_parameter<bool>("execute_plan", false);

    ik_position_tolerance_ = declare_parameter<double>("ik_position_tolerance", 0.010);
    q6_plan_error_tolerance_ = declare_parameter<double>("q6_plan_error_tolerance", 0.025);
    ik_max_iterations_ = declare_parameter<int>("ik_max_iterations", 1600);
    ik_damping_ = declare_parameter<double>("ik_damping", 0.035);
    ik_step_scale_ = declare_parameter<double>("ik_step_scale", 0.35);
    max_joint_step_ = declare_parameter<double>("max_joint_step", 0.060);

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

    obstacle_enabled_ = declare_parameter<bool>("obstacle_enabled", true);
    obstacle_id_ = declare_parameter<std::string>("obstacle_id", "front_box_obstacle");
    obstacle_center_vec_ =
      declare_parameter<std::vector<double>>("obstacle_center", std::vector<double>{0.55, 0.0, 0.25});
    obstacle_size_vec_ =
      declare_parameter<std::vector<double>>("obstacle_size", std::vector<double>{0.22, 0.60, 0.50});

    obstacle_margin_ = declare_parameter<double>("obstacle_margin", 0.04);
    obstacle_clearance_ = declare_parameter<double>("obstacle_clearance", 0.10);
    enable_q6_candidates_ = declare_parameter<bool>("enable_q6_candidates", true);
    q6_joint_name_ = declare_parameter<std::string>("q6_joint_name", "JOINT6");
    q6_candidates_ = declare_parameter<std::vector<double>>(
      "q6_candidates",
      std::vector<double>{
        -1.5708, 1.5708,
        0.0,
        -2.0944, 2.0944,
        -1.0472, 1.0472
      }
    );

    always_use_waypoints_ = declare_parameter<bool>("always_use_waypoints", true);

    target_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "/ee_obstacle/target_point",
      10,
      std::bind(&EEObstacleWaypointPlanNode::onTargetPoint, this, std::placeholders::_1)
    );

    display_pub_ =
      create_publisher<moveit_msgs::msg::DisplayTrajectory>("/display_planned_path", 1);

    marker_pub_ =
      create_publisher<visualization_msgs::msg::Marker>("/ee_obstacle/waypoint_markers", 1);
    RCLCPP_WARN(
      get_logger(),
      "SAFE DEFAULT: execute_plan=false. This node plans/displays only unless explicitly enabled."
    );

    init_timer_ = create_wall_timer(
      500ms,
      std::bind(&EEObstacleWaypointPlanNode::initializeMoveIt, this)
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

    robot_model_ = move_group_->getRobotModel();
    jmg_ = robot_model_->getJointModelGroup(planning_group_);
    planning_frame_ = move_group_->getPlanningFrame();

    if (!jmg_) {
      RCLCPP_ERROR(get_logger(), "JointModelGroup [%s] not found.", planning_group_.c_str());
      return;
    }

    obstacle_center_ = vectorFromParam(obstacle_center_vec_, Eigen::Vector3d(0.55, 0.0, 0.25));
    obstacle_size_ = vectorFromParam(obstacle_size_vec_, Eigen::Vector3d(0.22, 0.60, 0.50));

    RCLCPP_INFO(get_logger(), "MoveIt initialized.");
    RCLCPP_INFO(get_logger(), "  planning_group = %s", planning_group_.c_str());
    RCLCPP_INFO(get_logger(), "  ee_link        = %s", ee_link_.c_str());
    RCLCPP_INFO(get_logger(), "  planning_frame = %s", planning_frame_.c_str());
    RCLCPP_INFO(get_logger(), "  execute_plan   = %s", execute_plan_ ? "true" : "false");
    RCLCPP_INFO(get_logger(), "  ik_position_tolerance = %.4f m", ik_position_tolerance_);
    RCLCPP_INFO(get_logger(), "  q6_plan_error_tolerance = %.4f m", q6_plan_error_tolerance_);

    RCLCPP_INFO(
      get_logger(),
      "  obstacle center = [%.3f %.3f %.3f], size = [%.3f %.3f %.3f], margin=%.3f, clearance=%.3f",
      obstacle_center_.x(), obstacle_center_.y(), obstacle_center_.z(),
      obstacle_size_.x(), obstacle_size_.y(), obstacle_size_.z(),
      obstacle_margin_, obstacle_clearance_
    );

    if (obstacle_enabled_) {
      applyObstacleBox();
    }

    init_timer_->cancel();
  }

  Eigen::Vector3d vectorFromParam(
    const std::vector<double>& v,
    const Eigen::Vector3d& fallback)
  {
    if (v.size() != 3) {
      return fallback;
    }
    return Eigen::Vector3d(v[0], v[1], v[2]);
  }

  Eigen::Vector3d clampWorkspace(const Eigen::Vector3d& p)
  {
    Eigen::Vector3d out = p;
    out.x() = std::clamp(out.x(), workspace_x_min_, workspace_x_max_);
    out.y() = std::clamp(out.y(), workspace_y_min_, workspace_y_max_);
    out.z() = std::clamp(out.z(), workspace_z_min_, workspace_z_max_);
    return out;
  }

  void applyObstacleBox()
  {
    moveit_msgs::msg::CollisionObject obj;
    obj.header.frame_id = planning_frame_;
    obj.id = obstacle_id_;

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    primitive.dimensions.resize(3);

    // 실제 장애물보다 margin만큼 크게 넣어서 안전 여유 확보
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] =
      std::max(0.001, obstacle_size_.x() + 2.0 * obstacle_margin_);
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] =
      std::max(0.001, obstacle_size_.y() + 2.0 * obstacle_margin_);
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] =
      std::max(0.001, obstacle_size_.z() + 2.0 * obstacle_margin_);

    geometry_msgs::msg::Pose pose;
    pose.orientation.w = 1.0;
    pose.position.x = obstacle_center_.x();
    pose.position.y = obstacle_center_.y();
    pose.position.z = obstacle_center_.z();

    obj.primitives.push_back(primitive);
    obj.primitive_poses.push_back(pose);
    obj.operation = moveit_msgs::msg::CollisionObject::ADD;

    planning_scene_interface_.applyCollisionObjects({obj});

    RCLCPP_INFO(
      get_logger(),
      "Applied collision box [%s] to PlanningScene.",
      obstacle_id_.c_str()
    );
  }

  bool lineIntersectsInflatedObstacle(
    const Eigen::Vector3d& p0,
    const Eigen::Vector3d& p1)
  {
    Eigen::Vector3d half = 0.5 * obstacle_size_
      + Eigen::Vector3d::Constant(obstacle_margin_);

    Eigen::Vector3d bmin = obstacle_center_ - half;
    Eigen::Vector3d bmax = obstacle_center_ + half;

    Eigen::Vector3d d = p1 - p0;
    double tmin = 0.0;
    double tmax = 1.0;

    for (int i = 0; i < 3; ++i) {
      if (std::abs(d[i]) < 1e-9) {
        if (p0[i] < bmin[i] || p0[i] > bmax[i]) {
          return false;
        }
      } else {
        double inv = 1.0 / d[i];
        double t1 = (bmin[i] - p0[i]) * inv;
        double t2 = (bmax[i] - p0[i]) * inv;

        if (t1 > t2) {
          std::swap(t1, t2);
        }

        tmin = std::max(tmin, t1);
        tmax = std::min(tmax, t2);

        if (tmin > tmax) {
          return false;
        }
      }
    }

    return true;
  }

  std::vector<Eigen::Vector3d> makeWaypoints(
    const Eigen::Vector3d& current_pos,
    const Eigen::Vector3d& target_pos)
  {
    std::vector<Eigen::Vector3d> waypoints;

    // Correct strategy for this project:
    //
    // We are NOT trying to go over the obstacle and then descend behind it.
    // We are NOT trying to go around the side.
    //
    // Desired human-like motion:
    //   1) lift TCP vertically from home/current pose,
    //   2) keep that height,
    //   3) lean/reach forward to the target point on/above the box top.
    //
    // Therefore only two task-space waypoints are generated:
    //   WP1 = current x,y + target height
    //   WP2 = final target
    //
    // MoveIt still checks the rectangular collision object during each segment.

    double lift_z = target_pos.z();

    // If the target is lower than current z, do not force a downward intermediate.
    // But in the obstacle-top case target_z should usually be higher than current_z.
    lift_z = std::max(lift_z, current_pos.z());
    lift_z = std::clamp(lift_z, workspace_z_min_, workspace_z_max_);

    Eigen::Vector3d wp_lift(current_pos.x(), current_pos.y(), lift_z);
    Eigen::Vector3d wp_target(target_pos.x(), target_pos.y(), target_pos.z());

    if ((wp_lift - current_pos).norm() > 0.025) {
      waypoints.push_back(clampWorkspace(wp_lift));
    }

    if ((wp_target - wp_lift).norm() > 0.025) {
      waypoints.push_back(clampWorkspace(wp_target));
    } else {
      // If target is nearly identical to lift point, still publish target once.
      waypoints.push_back(clampWorkspace(wp_target));
    }

    return waypoints;
  }

  bool runPositionIK(
    moveit::core::RobotState& state,
    const Eigen::Vector3d& target_pos,
    double& final_error)
  {
    const auto variable_names = jmg_->getVariableNames();
    const std::size_t n = variable_names.size();

    if (n == 0) {
      RCLCPP_ERROR(get_logger(), "No variables in JointModelGroup.");
      return false;
    }

    std::vector<double> q;
    state.copyJointGroupPositions(jmg_, q);

    bool solved = false;

    for (int iter = 0; iter < ik_max_iterations_; ++iter) {
      state.setJointGroupPositions(jmg_, q);
      state.update();

      const Eigen::Isometry3d& ee_tf = state.getGlobalLinkTransform(ee_link_);
      Eigen::Vector3d current_pos = ee_tf.translation();

      Eigen::Vector3d e = target_pos - current_pos;
      final_error = e.norm();

      if (final_error <= ik_position_tolerance_) {
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

      Eigen::MatrixXd J = jacobian.topRows(3);

      Eigen::MatrixXd A =
        J * J.transpose()
        + ik_damping_ * ik_damping_ * Eigen::MatrixXd::Identity(3, 3);

      Eigen::VectorXd dq = J.transpose() * A.ldlt().solve(e);
      dq *= ik_step_scale_;

      for (std::size_t i = 0; i < n; ++i) {
        dq(static_cast<int>(i)) =
          std::clamp(dq(static_cast<int>(i)), -max_joint_step_, max_joint_step_);

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
    final_error = (target_pos - ee_tf.translation()).norm();

    solved = final_error <= ik_position_tolerance_;

    return solved;
  }

  bool validateTrajectory(
    const moveit::planning_interface::MoveGroupInterface::Plan& plan,
    const std::string& segment_name)
  {
    const auto& traj = plan.trajectory_.joint_trajectory;

    if (traj.points.empty()) {
      RCLCPP_ERROR(
        get_logger(),
        "[%s] Trajectory rejected: empty trajectory.",
        segment_name.c_str()
      );
      return false;
    }

    for (std::size_t i = 1; i < traj.points.size(); ++i) {
      const auto& prev = traj.points[i - 1].positions;
      const auto& curr = traj.points[i].positions;

      if (prev.size() != curr.size()) {
        RCLCPP_ERROR(
          get_logger(),
          "[%s] Trajectory rejected: joint size mismatch.",
          segment_name.c_str()
        );
        return false;
      }

      for (std::size_t j = 0; j < curr.size(); ++j) {
        double jump = std::abs(curr[j] - prev[j]);

        if (jump > max_waypoint_joint_jump_) {
          RCLCPP_ERROR(
            get_logger(),
            "[%s] Trajectory rejected: waypoint jump too large. point=%zu joint=%zu jump=%.4f rad limit=%.4f",
            segment_name.c_str(),
            i,
            j,
            jump,
            max_waypoint_joint_jump_
          );
          return false;
        }
      }
    }

    RCLCPP_INFO(
      get_logger(),
      "[%s] Trajectory validation passed. points=%zu",
      segment_name.c_str(),
      traj.points.size()
    );

    return true;
  }

  bool getGroupJointValue(
    const moveit::core::RobotState& state,
    const std::string& joint_name,
    double& value)
  {
    const auto variable_names = jmg_->getVariableNames();
    std::vector<double> q;
    state.copyJointGroupPositions(jmg_, q);

    for (std::size_t i = 0; i < variable_names.size(); ++i) {
      if (variable_names[i] == joint_name) {
        value = q[i];
        return true;
      }
    }

    return false;
  }

  bool setGroupJointValue(
    moveit::core::RobotState& state,
    const std::string& joint_name,
    double value)
  {
    const auto variable_names = jmg_->getVariableNames();
    std::vector<double> q;
    state.copyJointGroupPositions(jmg_, q);

    for (std::size_t i = 0; i < variable_names.size(); ++i) {
      if (variable_names[i] == joint_name) {
        q[i] = value;

        const auto& bounds = robot_model_->getVariableBounds(variable_names[i]);
        if (bounds.position_bounded_) {
          q[i] = std::clamp(q[i], bounds.min_position_, bounds.max_position_);
        }

        state.setJointGroupPositions(jmg_, q);
        state.update();
        return true;
      }
    }

    return false;
  }

  std::vector<double> makeQ6CandidateList(
    const moveit::core::RobotState& start_state)
  {
    std::vector<double> candidates;

    double current_q6 = 0.0;
    if (getGroupJointValue(start_state, q6_joint_name_, current_q6)) {
      candidates.push_back(current_q6);
    }

    for (double c : q6_candidates_) {
      bool duplicate = false;

      for (double existing : candidates) {
        if (std::abs(existing - c) < 1e-4) {
          duplicate = true;
          break;
        }
      }

      if (!duplicate) {
        candidates.push_back(c);
      }
    }

    return candidates;
  }

  bool planSegment(
    const moveit::core::RobotState& start_state,
    const moveit::core::RobotState& goal_state,
    const std::string& segment_name,
    moveit_msgs::msg::RobotTrajectory& out_traj)
  {
    std::vector<double> q_goal;
    goal_state.copyJointGroupPositions(jmg_, q_goal);

    const auto variable_names = jmg_->getVariableNames();

    std::map<std::string, double> q_goal_map;
    for (std::size_t i = 0; i < variable_names.size(); ++i) {
      q_goal_map[variable_names[i]] = q_goal[i];
    }
    move_group_->setStartState(start_state);
    move_group_->clearPoseTargets();

    bool target_ok = move_group_->setJointValueTarget(q_goal_map);
    if (!target_ok) {
      RCLCPP_ERROR(
        get_logger(),
        "[%s] setJointValueTarget failed.",
        segment_name.c_str()
      );
      return false;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto result = move_group_->plan(plan);

    bool success = (result == moveit::core::MoveItErrorCode::SUCCESS);

    if (!success) {
      RCLCPP_ERROR(
        get_logger(),
        "[%s] MoveIt planning failed.",
        segment_name.c_str()
      );
      return false;
    }

    if (!validateTrajectory(plan, segment_name)) {
      return false;
    }

    out_traj = plan.trajectory_;
    return true;
  }

  void publishWaypointMarkers(
    const std::vector<Eigen::Vector3d>& waypoints)
  {
    for (std::size_t i = 0; i < waypoints.size(); ++i) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = planning_frame_;
      marker.header.stamp = now();
      marker.ns = "ee_obstacle_waypoints";
      marker.id = static_cast<int>(i);
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;

      marker.pose.position.x = waypoints[i].x();
      marker.pose.position.y = waypoints[i].y();
      marker.pose.position.z = waypoints[i].z();
      marker.pose.orientation.w = 1.0;

      marker.scale.x = 0.04;
      marker.scale.y = 0.04;
      marker.scale.z = 0.04;

      marker.color.r = 1.0;
      marker.color.g = 0.3;
      marker.color.b = 0.0;
      marker.color.a = 0.9;

      marker_pub_->publish(marker);
    }
  }

  void publishDisplayTrajectory(
    const moveit::core::RobotState& start_state,
    const std::vector<moveit_msgs::msg::RobotTrajectory>& trajectories)
  {
    moveit_msgs::msg::DisplayTrajectory display;
    display.model_id = robot_model_->getName();

    moveit::core::robotStateToRobotStateMsg(start_state, display.trajectory_start);

    for (const auto& traj : trajectories) {
      display.trajectory.push_back(traj);
    }

    display_pub_->publish(display);

    RCLCPP_INFO(
      get_logger(),
      "Published %zu planned segment(s) to /display_planned_path.",
      trajectories.size()
    );
  }

  void onTargetPoint(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    if (!move_group_) {
      RCLCPP_WARN(get_logger(), "MoveIt is not initialized yet.");
      return;
    }

    if (obstacle_enabled_) {
      applyObstacleBox();
    }

    auto current_state = move_group_->getCurrentState(2.0);

    if (!current_state) {
      RCLCPP_ERROR(get_logger(), "Current robot state unavailable.");
      return;
    }

    current_state->update();

    const Eigen::Isometry3d& ee_tf =
      current_state->getGlobalLinkTransform(ee_link_);

    Eigen::Vector3d current_pos = ee_tf.translation();

    Eigen::Vector3d target_pos(
      msg->point.x,
      msg->point.y,
      msg->point.z
    );

    target_pos = clampWorkspace(target_pos);

    RCLCPP_INFO(
      get_logger(),
      "Received obstacle-aware target frame [%s]: x=%.3f y=%.3f z=%.3f",
      msg->header.frame_id.c_str(),
      target_pos.x(),
      target_pos.y(),
      target_pos.z()
    );

    RCLCPP_INFO(
      get_logger(),
      "Current EE position: x=%.3f y=%.3f z=%.3f",
      current_pos.x(),
      current_pos.y(),
      current_pos.z()
    );

    std::vector<Eigen::Vector3d> waypoints =
      makeWaypoints(current_pos, target_pos);

    RCLCPP_INFO(get_logger(), "Generated %zu waypoint target(s).", waypoints.size());

    for (std::size_t i = 0; i < waypoints.size(); ++i) {
      RCLCPP_INFO(
        get_logger(),
        "  WP%zu: x=%.3f y=%.3f z=%.3f",
        i + 1,
        waypoints[i].x(),
        waypoints[i].y(),
        waypoints[i].z()
      );
    }

    publishWaypointMarkers(waypoints);

    moveit::core::RobotState segment_start(*current_state);
    moveit::core::RobotState initial_start(*current_state);

    std::vector<moveit_msgs::msg::RobotTrajectory> segment_trajectories;

    for (std::size_t i = 0; i < waypoints.size(); ++i) {
      moveit_msgs::msg::RobotTrajectory traj;
      std::string segment_name = "Segment " + std::to_string(i + 1);

      bool use_q6_candidates =
        enable_q6_candidates_
        && (i >= 1);  // Segment 2부터 q6 회피 후보 사용

      std::vector<double> q6_try_list;

      if (use_q6_candidates) {
        q6_try_list = makeQ6CandidateList(segment_start);
      } else {
        double current_q6 = 0.0;
        if (!getGroupJointValue(segment_start, q6_joint_name_, current_q6)) {
          current_q6 = 0.0;
        }
        q6_try_list.push_back(current_q6);
      }

      bool segment_ok = false;
      moveit::core::RobotState chosen_goal(segment_start);

      for (std::size_t attempt = 0; attempt < q6_try_list.size(); ++attempt) {
        moveit::core::RobotState candidate_goal(segment_start);

        if (use_q6_candidates) {
          bool set_ok = setGroupJointValue(candidate_goal, q6_joint_name_, q6_try_list[attempt]);

          if (!set_ok) {
            RCLCPP_WARN(
              get_logger(),
              "[%s] q6 candidate skipped: joint [%s] not found.",
              segment_name.c_str(),
              q6_joint_name_.c_str()
            );
            continue;
          }

          RCLCPP_INFO(
            get_logger(),
            "[%s] q6 candidate attempt %zu/%zu: %s = %.4f rad",
            segment_name.c_str(),
            attempt + 1,
            q6_try_list.size(),
            q6_joint_name_.c_str(),
            q6_try_list[attempt]
          );
        }

        double ik_error = 0.0;
        bool ik_ok = runPositionIK(candidate_goal, waypoints[i], ik_error);

        RCLCPP_INFO(
          get_logger(),
          "[%s] IK target error=%.4f m, solved=%s",
          segment_name.c_str(),
          ik_error,
          ik_ok ? "true" : "false"
        );

        const bool ik_close_enough_for_q6_plan =
          use_q6_candidates && (ik_error <= q6_plan_error_tolerance_);

        if (!ik_ok && !ik_close_enough_for_q6_plan) {
          continue;
        }

        if (!ik_ok && ik_close_enough_for_q6_plan) {
          RCLCPP_WARN(
            get_logger(),
            "[%s] IK not strict-solved, but error %.4f m <= q6_plan_error_tolerance %.4f m. Trying MoveIt planning with this %s candidate.",
            segment_name.c_str(), ik_error, q6_plan_error_tolerance_, q6_joint_name_.c_str()
          );
        }

        moveit_msgs::msg::RobotTrajectory candidate_traj;
        bool plan_ok = planSegment(segment_start, candidate_goal, segment_name, candidate_traj);

        if (plan_ok) {
          RCLCPP_INFO(
            get_logger(),
            "[%s] SUCCESS with %s = %.4f rad",
            segment_name.c_str(),
            q6_joint_name_.c_str(),
            q6_try_list[attempt]
          );

          traj = candidate_traj;
          chosen_goal = candidate_goal;
          segment_ok = true;
          break;
        }

        RCLCPP_WARN(
          get_logger(),
          "[%s] planning failed with %s = %.4f rad. Trying next candidate.",
          segment_name.c_str(),
          q6_joint_name_.c_str(),
          q6_try_list[attempt]
        );
      }

      if (!segment_ok) {
        RCLCPP_ERROR(
          get_logger(),
          "[Segment %zu] All q6 candidates failed. Stop waypoint planning.",
          i + 1
        );
        return;
      }

      segment_trajectories.push_back(traj);
      segment_start = chosen_goal;
    }

    publishDisplayTrajectory(initial_start, segment_trajectories);

    if (execute_plan_) {
      RCLCPP_ERROR(
        get_logger(),
        "execute_plan=true is not implemented for chained obstacle waypoint mode yet. No execution performed."
      );
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
  double q6_plan_error_tolerance_{0.025};
  int ik_max_iterations_{1600};
  double ik_damping_{0.035};
  double ik_step_scale_{0.35};
  double max_joint_step_{0.060};

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

  bool obstacle_enabled_{true};
  std::string obstacle_id_{"front_box_obstacle"};
  std::vector<double> obstacle_center_vec_{0.55, 0.0, 0.25};
  std::vector<double> obstacle_size_vec_{0.22, 0.60, 0.50};

  Eigen::Vector3d obstacle_center_{0.55, 0.0, 0.25};
  Eigen::Vector3d obstacle_size_{0.22, 0.60, 0.50};

  double obstacle_margin_{0.04};
  double obstacle_clearance_{0.10};

  bool enable_q6_candidates_{true};
  std::string q6_joint_name_{"JOINT6"};
  std::vector<double> q6_candidates_{
    -1.5708, 1.5708,
    0.0,
    -2.0944, 2.0944,
    -1.0472, 1.0472
  };

  bool always_use_waypoints_{true};

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

  moveit::core::RobotModelConstPtr robot_model_;
  const moveit::core::JointModelGroup* jmg_{nullptr};

  rclcpp::TimerBase::SharedPtr init_timer_;

  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr target_sub_;

  rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<EEObstacleWaypointPlanNode>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}

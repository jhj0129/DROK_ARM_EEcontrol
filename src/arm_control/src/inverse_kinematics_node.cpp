#include "rclcpp/rclcpp.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include <string>
#include <cmath>
#include <memory>
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <map>
#include <array>
#include <chrono>
#include <mutex>
#include <atomic> // is_running_trajectory_ 사용을 위해 추가

// ROS 2 메시지 타입들
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "custom_msgs/msg/target.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp" // 오일러 각도 발행용 헤더
#include "std_msgs/msg/bool.hpp" // Bool 메시지 헤더 추가

using namespace Eigen;
using namespace std::chrono_literals;

// ===================================================================================
// 1. 전역 설정 및 상수
// ===================================================================================
const char* C_RED = "\033[1;31m";
const char* C_GREEN = "\033[1;32m";
const char* C_BLUE = "\033[1;34m";
const char* C_RESET = "\033[0m";

// IK 제어 모드 정의
constexpr int8_t MODE_POSITION_ONLY = 0;
constexpr int8_t MODE_FULL_POSE = 1;
constexpr int8_t MODE_PLANE_CONSTRAINT = 2;

// 평면 구속 타입 정의
constexpr int8_t PLANE_XY = 0;
constexpr int8_t PLANE_YZ = 1;
constexpr int8_t PLANE_XZ = 2;

// ===================================================================================
// 2. 유틸리티 및 기구학/자코비안 함수
// ===================================================================================

class QuinticPolynomial {
public:
    VectorXd c = VectorXd::Zero(6);
    void plan(double p_start, double p_end, double duration) {
        if(duration <= 1e-6){ c(0) = p_start; return; }
        double T=duration, T2=T*T, T3=T2*T, T4=T3*T, T5=T4*T;
        c(0)=p_start; c(1)=0; c(2)=0;
        c(3)=10*(p_end-p_start)/T3; c(4)=-15*(p_end-p_start)/T4; c(5)=6*(p_end-p_start)/T5;
    }
    double getPosition(double t){ return c(0)+c(1)*t+c(2)*pow(t,2)+c(3)*pow(t,3)+c(4)*pow(t,4)+c(5)*pow(t,5); }
};

MatrixXd jointToTransform01(const VectorXd& q){ MatrixXd tmp_m(4,4); Vector3d r = {0,0,-0.01525}; double q1 = q(0); tmp_m << cos(q1),-sin(q1),0,r(0), sin(q1),cos(q1),0,r(1), 0,0,1,r(2), 0,0,0,1; return tmp_m; }
MatrixXd jointToTransform12(const VectorXd& q){ MatrixXd tmp_m(4,4); Vector3d r = {0,0,0.09925}; double qq = q(1); tmp_m << cos(qq),0,sin(qq),r(0), 0,1,0,r(1),-sin(qq),0,cos(qq),r(2), 0,0,0,1; return tmp_m; }
MatrixXd jointToTransform23(const VectorXd& q){ MatrixXd tmp_m(4,4); Vector3d r = {-0.34865,0,0}; double qq = q(2); tmp_m << cos(qq),0,sin(qq),r(0), 0,1,0,r(1),-sin(qq),0,cos(qq),r(2), 0,0,0,1; return tmp_m; }
MatrixXd jointToTransform34(const VectorXd& q){ MatrixXd tmp_m(4,4); Vector3d r = {0.2518,0,0.08347}; double qq = q(3); tmp_m << 1,0,0,r(0), 0,cos(qq),-sin(qq),r(1), 0,sin(qq),cos(qq),r(2), 0,0,0,1; return tmp_m; }
MatrixXd jointToTransform45(const VectorXd& q){ MatrixXd tmp_m(4,4); Vector3d r = {0.06049,0,0}; double qq = q(4); tmp_m << cos(qq),-sin(qq),0,r(0), sin(qq),cos(qq),0,r(1), 0,0,1,r(2), 0,0,0,1; return tmp_m; }
MatrixXd jointToTransform56(const VectorXd& q){ MatrixXd tmp_m(4,4); Vector3d r = {0.05815,0,0}; double qq = q(5); tmp_m << 1,0,0,r(0), 0,cos(qq),-sin(qq),r(1), 0,sin(qq),cos(qq),r(2), 0,0,0,1; return tmp_m; }
MatrixXd jointToTransform6E(const VectorXd& q){ (void)q; MatrixXd tmp_m(4,4); Vector3d r = {0.06715,0,0}; tmp_m << 1,0,0,r(0), 0,1,0,r(1), 0,0,1,r(2), 0,0,0,1; return tmp_m; }
Matrix4d jointToPose(const VectorXd& q){ return jointToTransform01(q) * jointToTransform12(q) * jointToTransform23(q) * jointToTransform34(q) * jointToTransform45(q) * jointToTransform56(q) * jointToTransform6E(q); }

void calculateGeometricJacobian(const VectorXd& q, MatrixXd& J_P_out, MatrixXd& J_R_out) {
    J_P_out = MatrixXd::Zero(3, 6); J_R_out = MatrixXd::Zero(3, 6);
    Matrix4d T_f[7];
    T_f[0] = jointToTransform01(q); T_f[1] = jointToTransform12(q); T_f[2] = jointToTransform23(q);
    T_f[3] = jointToTransform34(q); T_f[4] = jointToTransform45(q); T_f[5] = jointToTransform56(q); T_f[6] = jointToTransform6E(q);
    Matrix4d T_g[8]; T_g[0] = Matrix4d::Identity();
    for (int i = 0; i < 7; ++i) { T_g[i + 1] = T_g[i] * T_f[i]; }
    Vector3d p_E = T_g[7].block<3, 1>(0, 3);
    Vector3d local_axes[6];
    local_axes[0] << 0,0,1; local_axes[1] << 0,1,0; local_axes[2] << 0,1,0;
    local_axes[3] << 1,0,0; local_axes[4] << 0,0,1; local_axes[5] << 1,0,0;
    for (int i = 0; i < 6; ++i) {
        Matrix3d R_i = T_g[i + 1].block<3, 3>(0, 0); Vector3d p_i = T_g[i + 1].block<3, 1>(0, 3);
        Vector3d z_i_global = R_i * local_axes[i];
        J_P_out.col(i) = z_i_global.cross(p_E - p_i); J_R_out.col(i) = z_i_global;
    }
}
Vector3d calculateOrientationError(const Matrix3d& R_des,const Matrix3d& R_curr){ return 0.5*(R_curr.col(0).cross(R_des.col(0))+R_curr.col(1).cross(R_des.col(1))+R_curr.col(2).cross(R_des.col(2))); }
Vector3d rotationMatrixToEulerAngles(const Matrix3d& R){ return R.eulerAngles(2, 1, 0); } // ZYX 순서: Yaw, Pitch, Roll
VectorXd inverseKinematics(const Vector3d& r_des, const Matrix3d& R_des, bool is_pose_controlled, VectorXd q0, const VectorXd& q_min, const VectorXd& q_max, double tol, int max_iter, double alpha, double lambda, double k0)
{
    VectorXd q = q0; const double orientation_weight = 0.8;
    for(int i=0; i < max_iter; ++i) {
        Matrix4d T_curr = jointToPose(q); MatrixXd J_P(3,6), J_R(3,6);
        calculateGeometricJacobian(q, J_P, J_R); VectorXd dq;
        if (is_pose_controlled) {
            VectorXd error_task(6); error_task.head(3) = r_des - T_curr.block<3,1>(0,3); error_task.tail(3) = calculateOrientationError(R_des, T_curr.block<3,3>(0,0));
            if (error_task.norm() < tol) { break; }
            MatrixXd J(6,6); J.topRows(3) = J_P; J.bottomRows(3) = J_R;
            Matrix<double, 6, 6> W = Matrix<double, 6, 6>::Identity(); W.bottomRightCorner<3, 3>() *= orientation_weight;
            MatrixXd J_w = W * J; VectorXd error_w = W * error_task; MatrixXd Jw_T = J_w.transpose();
            MatrixXd Jw_JwT_lambda = J_w * Jw_T + (lambda * lambda) * Matrix<double, 6, 6>::Identity();
            VectorXd dq_task = Jw_T * Jw_JwT_lambda.inverse() * error_w;
            VectorXd dq_null = VectorXd::Zero(6);
            if (k0 > 1e-5) {
                MatrixXd J_w_pinv = Jw_T * Jw_JwT_lambda.inverse(); MatrixXd N = MatrixXd::Identity(6, 6) - J_w_pinv * J_w;
                VectorXd grad_H = VectorXd::Zero(6);
                for(int j=0; j<6; ++j) { double q_range = q_max(j) - q_min(j); double q_center = (q_max(j) + q_min(j)) / 2.0; grad_H(j) = (q(j) - q_center) / (q_range * q_range); }
                dq_null = N * (-k0 * grad_H);
            }
            dq = dq_task + dq_null;
        } else {
            Vector3d error_pos = r_des - T_curr.block<3,1>(0,3);
            if (error_pos.norm() < tol) { break; }
            MatrixXd J_P_pinv = J_P.transpose() * (J_P * J_P.transpose() + (lambda * lambda) * MatrixXd::Identity(J_P.rows(), J_P.rows())).inverse();
            VectorXd dq_task = J_P_pinv * error_pos;
            VectorXd dq_null = VectorXd::Zero(6);
            if (k0 > 1e-5) {
                VectorXd grad_H = VectorXd::Zero(6);
                for(int j=0; j<6; ++j) { double q_range = q_max(j) - q_min(j); double q_center = (q_max(j) + q_min(j)) / 2.0; grad_H(j) = (q(j) - q_center) / (q_range * q_range); }
                MatrixXd N = MatrixXd::Identity(6, 6) - J_P_pinv * J_P; dq_null = N * (-k0 * grad_H);
            }
            dq = dq_task + dq_null;
        }
        q += alpha * dq; for (int j=0; j<6; ++j) q(j) = std::clamp(q(j), q_min(j), q_max(j));
    }
    return q;
}

// ===================================================================================
// 3. 메인 ROS 2 노드 클래스
// ===================================================================================

class RobotControlNode : public rclcpp::Node
{
public:
    RobotControlNode(): Node("robot_control_node"), target_received_(false), button7_pressed_(false), all_angles_ready_(false), is_running_trajectory_(false)
    {
        angles_initialized_.fill(false);
        current_q_ = VectorXd::Zero(6);
        setup_communications();
        fk_timer_ = this->create_wall_timer(10ms, std::bind(&RobotControlNode::publish_current_pose_callback, this));
        RCLCPP_INFO(this->get_logger(), "Robot Control Node has been started.");
    }

private:
    void setup_communications(){
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
        target_sub_ = this->create_subscription<custom_msgs::msg::Target>("/trajectory_target", qos, std::bind(&RobotControlNode::target_callback, this, std::placeholders::_1));
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("/joy", qos, std::bind(&RobotControlNode::joy_callback, this, std::placeholders::_1));
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/plotjuggler/joint_states", qos);
        ik_command_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/ik_joint_commands", qos);
        orientation_pub_ = this->create_publisher<geometry_msgs::msg::Quaternion>("/plotjuggler/end_effector_orientation", qos);
        pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/robot_end_effector_pose", 10);
        euler_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/robot_end_effector_euler", 10);
        
        // G-code 인터프리터에게 완료/거부 신호를 보낼 퍼블리셔
        finished_pub_ = this->create_publisher<std_msgs::msg::Bool>("/trajectory_finished", 10);
        
        initialize_motor_angle_subscribers();
    }

    void initialize_motor_angle_subscribers(){
        motor_topic_to_joint_index_={ {"/motor_angles/can10_motor_0x141",0}, {"/motor_angles/can10_motor_0x142",1}, {"/motor_angles/can10_motor_0x144",2}, {"/motor_angles/can11_motor_0x144",3}, {"/motor_angles/can11_motor_0x143",4}, {"/motor_angles/can11_motor_0x142",5} };
        motor_angle_subs_.resize(motor_topic_to_joint_index_.size());
        auto qos_m=rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
        int i=0; for(const auto& p : motor_topic_to_joint_index_){
            auto cb=[this,j=p.second](const std_msgs::msg::Float64::SharedPtr msg){
                std::lock_guard<std::mutex> lock(q_mutex_);
                bool reverse_direction = (j == 2 || j == 3 || j == 5);
                this->current_q_(j) = (reverse_direction ? -1.0 : 1.0) * msg->data * M_PI / 180.0;
                
                if(!this->angles_initialized_[j]){ 
                    this->angles_initialized_[j]=true; 
                    if(std::all_of(this->angles_initialized_.begin(),this->angles_initialized_.end(),[](bool v){return v;})){ 
                        this->all_angles_ready_=true; 
                        RCLCPP_INFO(this->get_logger(),"All motor angles received. Robot is ready."); 
                    } 
                }
            };
            motor_angle_subs_[i++]=this->create_subscription<std_msgs::msg::Float64>(p.first,qos_m,cb);
        }
    }

    void target_callback(const custom_msgs::msg::Target::SharedPtr msg){
        if (is_running_trajectory_.load()) {
            RCLCPP_WARN(this->get_logger(), "Target rejected: Another trajectory is already in progress.");
            auto response_msg = std::make_unique<std_msgs::msg::Bool>();
            response_msg->data = false; // 거부
            finished_pub_->publish(std::move(response_msg));
            return;
        }
        if(!all_angles_ready_){
            RCLCPP_ERROR(this->get_logger(),"Target rejected: Motor angles are not ready yet.");
            auto response_msg = std::make_unique<std_msgs::msg::Bool>();
            response_msg->data = false; // 거부
            finished_pub_->publish(std::move(response_msg));
            return;
        }
        { std::lock_guard<std::mutex> lock(target_mutex_); last_target_msg_ = *msg; target_received_ = true; }
        
        RCLCPP_INFO(this->get_logger(), "New target received. Starting IK trajectory generation...");
        std::thread(&RobotControlNode::calculate_and_publish_trajectory, this).detach();
    }

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg){
        if (msg->buttons.size() > 7 && msg->buttons[7] == 1 && !button7_pressed_) {
            button7_pressed_ = true;
            if(!target_received_){ RCLCPP_WARN(this->get_logger(),"IK Start ignored (Joy): No target received yet."); return; }
            if(!all_angles_ready_){ RCLCPP_WARN(this->get_logger(),"IK Start ignored (Joy): Motor angles not ready."); return; }
            if (is_running_trajectory_.load()) { RCLCPP_WARN(this->get_logger(), "Manual start ignored (Joy): Trajectory already in progress."); return; }
            RCLCPP_INFO(this->get_logger(),"Manual start (Joy button) pressed. Starting IK trajectory...");
            std::thread(&RobotControlNode::calculate_and_publish_trajectory, this).detach();
        } else if (msg->buttons.size() > 7 && msg->buttons[7] == 0) {
            button7_pressed_ = false;
        }
    }

    void publish_current_pose_callback() {
        if (!all_angles_ready_) { return; }
        VectorXd q_for_fk; { std::lock_guard<std::mutex> lock(q_mutex_); q_for_fk = current_q_; }
        Matrix4d T_final = jointToPose(q_for_fk);
        Vector3d pos = T_final.block<3,1>(0,3);
        Quaterniond q_rot(T_final.block<3,3>(0,0));
        auto pose_msg = geometry_msgs::msg::PoseStamped();
        rclcpp::Time now = this->get_clock()->now();
        pose_msg.header.stamp = now;
        pose_msg.header.frame_id = "base_link";
        pose_msg.pose.position.x = pos.x(); pose_msg.pose.position.y = pos.y(); pose_msg.pose.position.z = pos.z();
        pose_msg.pose.orientation.x = q_rot.x(); pose_msg.pose.orientation.y = q_rot.y(); pose_msg.pose.orientation.z = q_rot.z(); pose_msg.pose.orientation.w = q_rot.w();
        pose_publisher_->publish(pose_msg);
        
        Vector3d euler_rad = rotationMatrixToEulerAngles(T_final.block<3,3>(0,0));
        auto euler_msg = geometry_msgs::msg::Vector3Stamped();
        euler_msg.header = pose_msg.header;
        euler_msg.vector.x = euler_rad(2) * 180.0 / M_PI; // Roll
        euler_msg.vector.y = euler_rad(1) * 180.0 / M_PI; // Pitch
        euler_msg.vector.z = euler_rad(0) * 180.0 / M_PI; // Yaw
        euler_publisher_->publish(std::move(euler_msg));
    }

    void calculate_and_publish_trajectory()
    {
        is_running_trajectory_.store(true);
        auto trajectory_guard = std::unique_ptr<bool, std::function<void(bool*)>>(
            new bool(true),
            [this](bool*){ 
                this->is_running_trajectory_.store(false); 
                RCLCPP_INFO(this->get_logger(), "Trajectory execution state has been reset.");
            }
        );

        custom_msgs::msg::Target current_target;
        { std::lock_guard<std::mutex> lock(target_mutex_); current_target = last_target_msg_; }

        bool is_pose_controlled = false;
        Matrix3d R_des_target;
        std::string mode_str = "POSITION_ONLY";
        if (current_target.mode == MODE_FULL_POSE) {
            mode_str = "FULL_POSE"; is_pose_controlled = true;
            Quaterniond q_target(current_target.orientation.w, current_target.orientation.x, current_target.orientation.y, current_target.orientation.z);
            R_des_target = q_target.normalized().toRotationMatrix();
        } 
        
        VectorXd q_lim_min(6), q_lim_max(6);
        q_lim_min << -45, 0, -180, -95, -70, -180; q_lim_max << 130, 180, 0, 95, 70, 180;
        q_lim_min *= M_PI/180.0; q_lim_max *= M_PI/180.0;
        double tol, alpha, lambda, k0; int max_iter; double freq;
        if (is_pose_controlled) {
             tol = 1e-3;
             alpha = 0.17;
             max_iter = 13;
             lambda = 0.1; 
             k0 = 0.01; 
             freq = 150.0; 
            } 

        else { 
            tol = 1e-4; 
            alpha = 0.15; 
            max_iter = 15; 
            lambda = 0.1; 
            k0 = 0.005; 
            freq = 100.0; 
        }

        const double dur = std::max(current_target.duration_s, 0.1);
        Vector3d r_target = {current_target.position.x, current_target.position.y, current_target.position.z};
        VectorXd q_start(6); { std::lock_guard<std::mutex> lock(q_mutex_); q_start = current_q_; }
        Matrix4d T_start = jointToPose(q_start);
        Vector3d r_start = T_start.block<3,1>(0,3);
        Quaterniond rot_start(T_start.block<3,3>(0,0));
        std::vector<QuinticPolynomial> traj(3);
        for(int i=0; i<3; ++i) traj[i].plan(r_start(i), r_target(i), dur);
        Quaterniond rot_target(R_des_target);
        if (rot_start.dot(rot_target) < 0.0) { rot_target.coeffs() *= -1.0; }
        
        RCLCPP_INFO(this->get_logger(), "IK Trajectory Start. Mode: %s, Freq: %.1f Hz, Duration: %.2f s", mode_str.c_str(), freq, dur);
        
        rclcpp::Rate rate(freq);
        const int n_steps = static_cast<int>(dur * freq);
        VectorXd q_k(6);

        for (int i = 1; i <= n_steps; ++i) {
            if (!rclcpp::ok()) { RCLCPP_WARN(this->get_logger(), "Trajectory interrupted due to shutdown. Aborting."); return; }
            { std::lock_guard<std::mutex> lock(q_mutex_); q_k = current_q_; }
            double t = static_cast<double>(i) / n_steps * dur;
            Vector3d r_i = {traj[0].getPosition(t), traj[1].getPosition(t), traj[2].getPosition(t)};
            Matrix3d R_i = is_pose_controlled ? rot_start.slerp(t/dur, rot_target).toRotationMatrix() : Matrix3d::Identity();
            VectorXd q_command = inverseKinematics(r_i, R_i, is_pose_controlled, q_k, q_lim_min, q_lim_max, tol, max_iter, alpha, lambda, k0);
            
            auto js = std::make_unique<sensor_msgs::msg::JointState>();
            js->header.stamp = this->get_clock()->now(); js->name = {"j1","j2","j3","j4","j5","j6"};
            js->position.assign(q_command.data(), q_command.data() + q_command.size());
            joint_state_pub_->publish(std::move(js));
            
            const std::vector<double> gear_ratios = {36, 36, 36, 6, 6, 6};
            VectorXd q_rmd_deg = q_command * 180.0 / M_PI;
            q_rmd_deg(2) *= -1.0; q_rmd_deg(3) *= -1.0; q_rmd_deg(5) *= -1.0;
            VectorXd q_rmd(6);
            for (int j = 0; j < 6; ++j) { q_rmd(j) = q_rmd_deg(j) * gear_ratios[j] / 0.01; }
            auto ik_cmd = std::make_unique<std_msgs::msg::Float64MultiArray>();
            ik_cmd->data.assign(q_rmd.data(), q_rmd.data() + q_rmd.size());
            ik_command_pub_->publish(std::move(ik_cmd));
            rate.sleep();
        }
        RCLCPP_INFO(this->get_logger(), "Trajectory command stream finished. Verifying final position...");

        const double completion_tolerance = 0.002; // 2mm
        const auto timeout_duration = std::chrono::seconds(5);
        auto start_time = this->get_clock()->now();
        bool is_completed = false;
        double final_pos_error = -1.0, final_orient_error = -1.0;
        
        while (rclcpp::ok() && (this->get_clock()->now() - start_time) < timeout_duration) {
            VectorXd q_check; { std::lock_guard<std::mutex> lock(q_mutex_); q_check = current_q_; }
            Matrix4d T_actual = jointToPose(q_check);
            Vector3d r_actual = T_actual.block<3,1>(0,3);
            final_pos_error = (r_target - r_actual).norm();
            bool position_ok = final_pos_error < completion_tolerance;
            bool orientation_ok = true;
            if (is_pose_controlled) {
                Matrix3d R_actual = T_actual.block<3,3>(0,0);
                final_orient_error = calculateOrientationError(R_des_target, R_actual).norm();
                orientation_ok = final_orient_error < 0.05; // ~2.8 degrees
            }
            if (position_ok && orientation_ok) {
                RCLCPP_INFO(this->get_logger(), "Target reached within tolerance.");
                is_completed = true;
                break;
            }
            std::this_thread::sleep_for(50ms);
        }

        if (!is_completed) {
            RCLCPP_ERROR(this->get_logger(), "Failed to reach target within timeout! Final Pos Error: %.4f m", final_pos_error);
            if (is_pose_controlled) { RCLCPP_ERROR(this->get_logger(), "Final Orient Error: %.4f rad", final_orient_error); }
        }

        VectorXd q_final_actual; { std::lock_guard<std::mutex> lock(q_mutex_); q_final_actual = current_q_; }
        Matrix4d T_f = jointToPose(q_final_actual);
        Vector3d r_f = T_f.block<3,1>(0,3); Matrix3d R_f = T_f.block<3,3>(0,0);
        double p_err = (r_target - r_f).norm();
        double o_err = is_pose_controlled ? (calculateOrientationError(R_des_target, R_f)).norm() : 0.0;
        Vector3d e_f_deg = rotationMatrixToEulerAngles(R_f) * (180.0/M_PI);
        Vector3d e_d_deg = rotationMatrixToEulerAngles(R_des_target) * (180.0/M_PI);
        
        RCLCPP_INFO(this->get_logger(),"-------------------- IK Result --------------------");
        RCLCPP_INFO(this->get_logger(),"Mode: %s", mode_str.c_str());
        RCLCPP_INFO(this->get_logger(),"Desired Pos: [%.4f, %.4f, %.4f]", r_target.x(), r_target.y(), r_target.z());
        RCLCPP_INFO(this->get_logger(),"Final Pos:   [%.4f, %.4f, %.4f] | Err: %.5f", r_f.x(), r_f.y(), r_f.z(), p_err);
        if(is_pose_controlled){
            RCLCPP_INFO(this->get_logger(),"Desired YPR(deg): (%.2f, %.2f, %.2f)", e_d_deg(0), e_d_deg(1), e_d_deg(2));
            RCLCPP_INFO(this->get_logger(),"Final YPR(deg):   (%.2f, %.2f, %.2f) | Err: %.5f", e_f_deg(0), e_f_deg(1), e_f_deg(2), o_err);
        }
        VectorXd q_k_deg=q_final_actual*180.0/M_PI; VectorXd q_min_deg=q_lim_min*180.0/M_PI; VectorXd q_max_deg=q_lim_max*180.0/M_PI;
        for (int i=0; i<6; ++i) {
            bool at_lim = std::abs(q_k_deg(i)-q_min_deg(i)) < 0.1 || std::abs(q_k_deg(i)-q_max_deg(i)) < 0.1;
            RCLCPP_INFO(this->get_logger(), "%sJoint %d: %8.2f (Min: %8.2f, Max: %8.2f)%s", at_lim?C_RED:C_GREEN, i+1, q_k_deg(i), q_min_deg(i), q_max_deg(i), C_RESET);
        }
        RCLCPP_INFO(this->get_logger(),"-------------------------------------------------");

        RCLCPP_INFO(this->get_logger(), "Sending completion signal to G-code node. Result: %s", is_completed ? "SUCCESS" : "FAILURE (Timeout)");
        auto response_msg = std::make_unique<std_msgs::msg::Bool>();
        response_msg->data = is_completed;
        finished_pub_->publish(std::move(response_msg));
    }

    // Member variables
    rclcpp::Subscription<custom_msgs::msg::Target>::SharedPtr target_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    std::vector<rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr> motor_angle_subs_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr ik_command_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Quaternion>::SharedPtr orientation_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr euler_publisher_;
    rclcpp::TimerBase::SharedPtr fk_timer_;
    
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr finished_pub_;
    std::atomic<bool> is_running_trajectory_; 

    VectorXd current_q_;
    std::array<bool, 6> angles_initialized_;
    bool all_angles_ready_;
    std::mutex q_mutex_;
    custom_msgs::msg::Target last_target_msg_;
    std::mutex target_mutex_;
    bool target_received_;
    bool button7_pressed_;
    std::map<std::string,int> motor_topic_to_joint_index_;
};

// ===================================================================================
// 4. Main 함수
// ===================================================================================
int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    auto robot_node = std::make_shared<RobotControlNode>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    executor.add_node(robot_node);
    RCLCPP_INFO(robot_node->get_logger(), "Starting executor with 4 threads.");
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
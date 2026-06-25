#include "rclcpp/rclcpp.hpp"
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <cmath>
#include <memory>
#include <iostream>
#include <iomanip>
#include <algorithm> // for std::clamp
#include <map>       // 모터 정보 관리를 위해 map 사용
#include <array>     // 각도 초기화 상태 저장을 위해 array 사용

// ROS 2 메시지 타입
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include "custom_msgs/msg/target.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

using namespace Eigen;
using namespace std::chrono_literals;

// ANSI Color Codes for Console Output
const char* C_RED = "\033[1;31m";
const char* C_GREEN = "\033[1;32m";
const char* C_BLUE = "\033[1;34m";
const char* C_RESET = "\033[0m";

// IK 모드 정의
constexpr int8_t MODE_POSITION_ONLY = 0;
constexpr int8_t MODE_FULL_POSE = 1;

// QuinticPolynomial 클래스
class QuinticPolynomial {
public:
    VectorXd c = VectorXd::Zero(6);
    QuinticPolynomial() {}
    void plan(double p_start, double p_end, double duration) {
        if (duration <= 1e-6) { c(0) = p_start; return; }
        double T = duration, T2 = T*T, T3 = T2*T, T4 = T3*T, T5 = T4*T;
        c(0) = p_start; c(1) = 0; c(2) = 0;
        c(3) = 10 * (p_end - p_start) / T3;
        c(4) = -15 * (p_end - p_start) / T4;
        c(5) = 6 * (p_end - p_start) / T5;
    }
    double getPosition(double t) const { return c(0) + c(1)*t + c(2)*t*t + c(3)*pow(t,3) + c(4)*pow(t,4) + c(5)*pow(t,5); }
    double getVelocity(double t) const { return c(1) + 2*c(2)*t + 3*c(3)*t*t + 4*c(4)*pow(t,3) + 5*c(5)*pow(t,4); }
    double getAcceleration(double t) const { return 2*c(2) + 6*c(3)*t + 12*c(4)*t*t + 20*c(5)*pow(t,3); }
};

// ==========================================================
// 함수 선언부 (Forward Declarations)
// ==========================================================
MatrixXd jointToTransform01(const VectorXd& q);
MatrixXd jointToTransform12(const VectorXd& q);
MatrixXd jointToTransform23(const VectorXd& q);
MatrixXd jointToTransform34(const VectorXd& q);
MatrixXd jointToTransform45(const VectorXd& q);
MatrixXd jointToTransform56(const VectorXd& q);
MatrixXd jointToTransform6E(const VectorXd& q);
Matrix4d jointToPose(const VectorXd& q);
Vector3d calculateOrientationError(const Matrix3d& R_des, const Matrix3d& R_curr);
void calculateGeometricJacobian(const VectorXd& q_current, MatrixXd& J_out);
MatrixXd pseudoInverseMat(const MatrixXd& M, double lambda);
VectorXd inverseKinematics_PositionOnly(const Vector3d& r_des, VectorXd q0, const VectorXd& q_min, const VectorXd& q_max, double tol, int max_iter, double alpha, double lambda, bool verbose);
VectorXd inverseKinematics_Pose(const Vector3d& r_des, const Matrix3d& R_des, VectorXd q0, const VectorXd& q_min, const VectorXd& q_max, double tol, int max_iter, double alpha, double lambda, bool verbose);
MatrixXd calculateJacobianDerivative(const VectorXd& q, const VectorXd& q_dot, double h);
Vector3d rotationMatrixToEulerAngles(const Matrix3d& R);

// ==========================================================
// 순방향 기구학 (Forward Kinematics) 함수 구현부
// ==========================================================
MatrixXd jointToTransform01(const VectorXd& q){ MatrixXd tmp_m(4,4); Vector3d r = {0,0,-0.01525}; double q1 = q(0); tmp_m << cos(q1), -sin(q1), 0, r(0), sin(q1), cos(q1), 0, r(1), 0, 0, 1, r(2), 0, 0, 0, 1; return tmp_m; }
MatrixXd jointToTransform12(const VectorXd& q){ MatrixXd tmp_m(4,4); Vector3d r = {0,0,0.09925}; double qq = q(1); tmp_m << cos(qq), 0, sin(qq), r(0), 0, 1, 0, r(1), -sin(qq), 0, cos(qq), r(2), 0, 0, 0, 1; return tmp_m; }
MatrixXd jointToTransform23(const VectorXd& q){ MatrixXd tmp_m(4,4); Vector3d r = {-0.34865,0,0}; double qq = q(2); tmp_m << cos(qq), 0, -sin(qq), r(0), 0, 1, 0, r(1), sin(qq), 0, cos(qq), r(2), 0, 0, 0, 1; return tmp_m; }
MatrixXd jointToTransform34(const VectorXd& q){ MatrixXd tmp_m(4,4); Vector3d r = {0.2518,0,0.08347}; double qq = q(3); tmp_m << 1, 0, 0, r(0), 0, cos(qq), -sin(qq), r(1), 0, sin(qq), cos(qq), r(2), 0, 0, 0, 1; return tmp_m; }
MatrixXd jointToTransform45(const VectorXd& q){ MatrixXd tmp_m(4,4); Vector3d r = {0.06049,0,0}; double qq = q(4); tmp_m << cos(qq), -sin(qq), 0, r(0), sin(qq), cos(qq), 0, r(1), 0, 0, 1, r(2), 0, 0, 0, 1; return tmp_m; }
MatrixXd jointToTransform56(const VectorXd& q){ MatrixXd tmp_m(4,4); Vector3d r = {0.05815,0,0}; double qq = q(5); tmp_m << 1, 0, 0, r(0), 0, cos(qq), -sin(qq), r(1), 0, sin(qq), cos(qq), r(2), 0, 0, 0, 1; return tmp_m; }
MatrixXd jointToTransform6E(const VectorXd& q){ (void)q; MatrixXd tmp_m(4,4); Vector3d r = {0.02215,0,0}; tmp_m << 1, 0, 0, r(0), 0, 1, 0, r(1), 0, 0, 1, r(2), 0, 0, 0, 1; return tmp_m; }

Matrix4d jointToPose(const VectorXd& q){
    MatrixXd T_01 = jointToTransform01(q); MatrixXd T_12 = jointToTransform12(q); MatrixXd T_23 = jointToTransform23(q); MatrixXd T_34 = jointToTransform34(q); MatrixXd T_45 = jointToTransform45(q); MatrixXd T_56 = jointToTransform56(q); MatrixXd T_6E = jointToTransform6E(q);
    return T_01 * T_12 * T_23 * T_34 * T_45 * T_56 * T_6E;
}

// ==========================================================
// 자코비안 및 관련 함수 구현부
// ==========================================================
void calculateGeometricJacobian(const VectorXd& q, MatrixXd& J_out) {
    MatrixXd J_P(3, 6), J_R(3, 6);
    Matrix4d T_I1=jointToTransform01(q); Matrix4d T_12=jointToTransform12(q); Matrix4d T_I2=T_I1*T_12; Matrix4d T_23=jointToTransform23(q); Matrix4d T_I3=T_I2*T_23; Matrix4d T_34=jointToTransform34(q); Matrix4d T_I4=T_I3*T_34; Matrix4d T_45=jointToTransform45(q); Matrix4d T_I5=T_I4*T_45; Matrix4d T_56=jointToTransform56(q); Matrix4d T_I6=T_I5*T_56; Matrix4d T_6E=jointToTransform6E(q); Matrix4d T_IE=T_I6*T_6E;
    Vector3d p0=Vector3d::Zero(); Vector3d p1=T_I1.block<3,1>(0,3); Vector3d p2=T_I2.block<3,1>(0,3); Vector3d p3=T_I3.block<3,1>(0,3); Vector3d p4=T_I4.block<3,1>(0,3); Vector3d p5=T_I5.block<3,1>(0,3); Vector3d pE=T_IE.block<3,1>(0,3);
    Matrix3d R0=Matrix3d::Identity(); Matrix3d R1=T_I1.block<3,3>(0,0); Matrix3d R2=T_I2.block<3,3>(0,0); Matrix3d R3=T_I3.block<3,3>(0,0); Matrix3d R4=T_I4.block<3,3>(0,0); Matrix3d R5=T_I5.block<3,3>(0,0);
    Vector3d z1={0,0,1}, z2={0,1,0}, z3={0,1,0}, z4={1,0,0}, z5={0,0,1}, z6={1,0,0};
    Vector3d Z1=R0*z1; Vector3d Z2=R1*z2; Vector3d Z3=R2*z3; Vector3d Z4=R3*z4; Vector3d Z5=R4*z5; Vector3d Z6=R5*z6;
    J_P.col(0) = Z1.cross(pE-p0); J_P.col(1) = Z2.cross(pE-p1); J_P.col(2) = Z3.cross(pE-p2); J_P.col(3) = Z4.cross(pE-p3); J_P.col(4) = Z5.cross(pE-p4); J_P.col(5) = Z6.cross(pE-p5);
    J_R.col(0)=Z1; J_R.col(1)=Z2; J_R.col(2)=Z3; J_R.col(3)=Z4; J_R.col(4)=Z5; J_R.col(5)=Z6;
    J_out.resize(6, 6); // J_out의 크기를 명시적으로 설정
    J_out.topRows(3) = J_P;
    J_out.bottomRows(3) = J_R;
}

MatrixXd pseudoInverseMat(const MatrixXd& M, double lambda) {
    if (M.rows() >= M.cols()) { MatrixXd I = MatrixXd::Identity(M.cols(), M.cols()); return (M.transpose() * M + lambda * lambda * I).inverse() * M.transpose(); }
    else { MatrixXd I = MatrixXd::Identity(M.rows(), M.rows()); return M.transpose() * (M * M.transpose() + lambda * lambda * I).inverse(); }
}

MatrixXd calculateJacobianDerivative(const VectorXd& q, const VectorXd& q_dot, double h) {
    MatrixXd J_dot=MatrixXd::Zero(6,6), J_curr(6,6), J_h(6,6);
    calculateGeometricJacobian(q, J_curr);
    for(int i=0; i<6; ++i){ VectorXd q_h=q; q_h(i)+=h; calculateGeometricJacobian(q_h, J_h); MatrixXd dJ_dqi=(J_h-J_curr)/h; J_dot+=dJ_dqi*q_dot(i); }
    return J_dot;
}

Vector3d calculateOrientationError(const Matrix3d& R_des, const Matrix3d& R_curr) {
    Vector3d nc=R_curr.col(0), oc=R_curr.col(1), ac=R_curr.col(2); Vector3d nd=R_des.col(0), od=R_des.col(1), ad=R_des.col(2);
    return 0.5 * (nc.cross(nd) + oc.cross(od) + ac.cross(ad));
}

VectorXd inverseKinematics_PositionOnly(const Vector3d& r_des, VectorXd q0, const VectorXd& q_min_rad, const VectorXd& q_max_rad, double tol, int max_iter, double alpha, double lambda, bool verbose) {
    int num_it = 0; MatrixXd J_full(6,6), J_P(3,6); VectorXd q = q0;
    while (num_it < max_iter) {
        Vector3d r_curr = jointToPose(q).block<3,1>(0,3);
        Vector3d dr = r_des - r_curr;
        if (dr.norm() < tol) break;
        calculateGeometricJacobian(q, J_full);
        J_P = J_full.topRows(3);
        VectorXd dq = pseudoInverseMat(J_P, lambda) * dr;
        q += alpha * dq;
        for (int i = 0; i < q.size(); ++i) { q(i) = std::clamp(q(i), q_min_rad(i), q_max_rad(i)); }
        num_it++;
    }
    // ... 수렴 실패 시 경고 및 이전 값 반환 로직은 생략하지 않는 것이 좋음
    return q;
}


// [★★★★★ 핵심 수정: 성능 및 안정성 개선 버전 ★★★★★]
VectorXd inverseKinematics_Pose(const Vector3d& r_des, const Matrix3d& R_des, VectorXd q0,
                                const VectorXd& q_min_rad, const VectorXd& q_max_rad,
                                double tol, int max_iter, double alpha, double lambda, bool verbose) {
    int num_it = 0;
    MatrixXd J(6, 6);
    VectorXd q = q0;
    VectorXd dx(6);

    // 관절 한계 회피를 위한 파라미터
    double k0 = 0.005; // 비용 함수 강도
    double activation_threshold = 0.8; // 관절 범위의 80% 이상 사용 시 활성화

    while (num_it < max_iter) {
        Matrix4d T_curr = jointToPose(q);
        dx.head(3) = r_des - T_curr.block<3,1>(0,3);
        dx.tail(3) = calculateOrientationError(R_des, T_curr.block<3,3>(0,0));

        if (dx.norm() < tol) {
            if (verbose) RCLCPP_INFO(rclcpp::get_logger("IK_Function"), "Converged in %d iterations.", num_it);
            return q;
        }

        calculateGeometricJacobian(q, J);

        // 관절이 한계에 가까울 때만 비용 함수 활성화
        VectorXd grad_H = VectorXd::Zero(6);
        for (int i = 0; i < 6; ++i) {
            double q_range = q_max_rad(i) - q_min_rad(i);
            double q_mid = (q_max_rad(i) + q_min_rad(i)) / 2.0;
            double current_usage = std::abs(q(i) - q_mid) / (q_range / 2.0);

            if (current_usage > activation_threshold) {
                grad_H(i) = -k0 * (q(i) - q_mid) / (q_range * q_range);
            }
        }
        
        // Levenberg-Marquardt (DLS) 방식의 표준 구현
        MatrixXd A = J.transpose() * J + (lambda * lambda) * MatrixXd::Identity(6, 6);
        VectorXd b = J.transpose() * dx + grad_H;

        // 역행렬 대신 QR 분해 솔버를 사용하여 dq 계산 (성능 및 안정성 향상)
        VectorXd dq = A.colPivHouseholderQr().solve(b);
        
        q += alpha * dq;

        for (int i = 0; i < q.size(); ++i) {
            q(i) = std::clamp(q(i), q_min_rad(i), q_max_rad(i));
        }
        num_it++;
    }

    if (verbose) {
        RCLCPP_WARN(rclcpp::get_logger("IK_Function"), "IK did not converge within %d iterations. Final error: %f", max_iter, dx.norm());
    }
    
    if (dx.norm() > tol * 50) { return q0; }
    return q;
}


Vector3d rotationMatrixToEulerAngles(const Matrix3d& R) {
    double sy=std::sqrt(R(0,0)*R(0,0)+R(1,0)*R(1,0)); bool singular=sy<1e-6; double x,y,z;
    if(!singular){x=std::atan2(R(2,1),R(2,2)); y=std::atan2(-R(2,0),sy); z=std::atan2(R(1,0),R(0,0));}
    else{x=std::atan2(-R(1,2),R(1,1)); y=std::atan2(-R(2,0),sy); z=0;}
    return {x,y,z};
}

// ==========================================================
// ROS 2 IK Node
// ==========================================================
class InverseKinematicsNode : public rclcpp::Node
{
public:
    InverseKinematicsNode() : Node("inverse_kinematics_node"), 
                              target_received_(false), button7_pressed_(false), all_angles_ready_(false)
    {
        angles_initialized_.fill(false);
        auto qos_reliable = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
        auto qos_best_effort = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
        target_sub_ = this->create_subscription<custom_msgs::msg::Target>("/trajectory_target", qos_best_effort, std::bind(&InverseKinematicsNode::target_callback, this, std::placeholders::_1));
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("/joy", qos_best_effort, std::bind(&InverseKinematicsNode::joy_callback, this, std::placeholders::_1));
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/plotjuggler/joint_states", qos_reliable);
        ik_command_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/ik_joint_commands", qos_reliable);
        current_q_ = VectorXd::Zero(6);
        initialize_motor_angle_subscribers();
        RCLCPP_INFO(this->get_logger(), "Inverse Kinematics Node has been started.");
        RCLCPP_INFO(this->get_logger(), "Waiting for real motor angles, target from UMPC, and joystick input...");
    }

private:
    void initialize_motor_angle_subscribers() {
        motor_topic_to_joint_index_ = {{"/motor_angles/can10_motor_0x141",0}, {"/motor_angles/can10_motor_0x142",1}, {"/motor_angles/can10_motor_0x144",2}, {"/motor_angles/can11_motor_0x144",3}, {"/motor_angles/can11_motor_0x143",4}, {"/motor_angles/can11_motor_0x142",5}};
        motor_angle_subs_.resize(motor_topic_to_joint_index_.size());
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
        int i=0;
        for(const auto& pair : motor_topic_to_joint_index_){
            auto cb=[this, j=pair.second](const std_msgs::msg::Float64::SharedPtr msg){
                this->current_q_(j) = (j==2) ? -msg->data*M_PI/180.0 : msg->data*M_PI/180.0;
                if(!this->angles_initialized_[j]){
                    this->angles_initialized_[j]=true;
                    if(std::all_of(this->angles_initialized_.begin(),this->angles_initialized_.end(),[](bool v){return v;})){
                        this->all_angles_ready_=true;
                        RCLCPP_INFO(this->get_logger(),"All motor angles received. System is ready.");
                    }
                }
            };
            motor_angle_subs_[i++] = this->create_subscription<std_msgs::msg::Float64>(pair.first, qos, cb);
        }
    }
    
    void target_callback(const custom_msgs::msg::Target::SharedPtr msg) {
        last_target_mode_ = msg->mode;
        last_target_pos_ << msg->position.x, msg->position.y, msg->position.z;
        last_target_rot_ = Quaterniond(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z).normalized();
        last_target_duration_ = msg->duration_s;
        target_received_ = true;
        std::string mode_str = (last_target_mode_==MODE_FULL_POSE)?"FULL POSE":"POSITION ONLY";
        RCLCPP_INFO(this->get_logger(), "Received new target (Mode: %s)", mode_str.c_str());
        RCLCPP_INFO(this->get_logger(), " -> Pos: [%.3f, %.3f, %.3f], Quat: [%.2f, %.2f, %.2f, %.2f], Duration: %.2f s",
                    last_target_pos_.x(), last_target_pos_.y(), last_target_pos_.z(), 
                    last_target_rot_.w(), last_target_rot_.x(), last_target_rot_.y(), last_target_rot_.z(), last_target_duration_);
    }

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        if (msg->buttons.size()>7 && msg->buttons[7]==1) {
            if (!button7_pressed_) {
                button7_pressed_=true;
                if(!target_received_){RCLCPP_WARN(this->get_logger(),"Button 7 pressed, but no target received yet."); return;}
                if(!all_angles_ready_){RCLCPP_WARN(this->get_logger(),"Button 7 pressed, but not all motor angles received yet."); return;}
                RCLCPP_INFO(this->get_logger(), "Button 7 pressed. Starting trajectory IK calculation.");
                calculate_and_publish_trajectory();
            }
        } else { button7_pressed_ = false; }
    }
    
    void calculate_and_publish_trajectory() {
        VectorXd q_lim_min(6), q_lim_max(6);
        q_lim_min << 0.0, 0.0, -180.0, -90.0, -50.0, -90.0;
        q_lim_max << 120.0, 180.0, 0.0, 90.0, 50.0, 90.0;
        q_lim_min *= M_PI/180.0; q_lim_max *= M_PI/180.0;
        
        // [파라미터 튜닝] 안정성을 위해 스텝 사이즈와 람다를 조정
        double tol=1e-4, alpha=0.08, lambda=0.001; int max_iter=500;

        const int num_steps=500; const double duration=std::max(last_target_duration_, 0.1);
        const double freq=static_cast<double>(num_steps)/duration;
        rclcpp::Rate rate(freq);
        const double dt=1.0/freq;

        Matrix4d T_start=jointToPose(current_q_);
        Vector3d r_start=T_start.block<3,1>(0,3);
        Quaterniond q_rot_start(T_start.block<3,3>(0,0));
        VectorXd q_k = current_q_;

        std::vector<QuinticPolynomial> traj(3);
        traj[0].plan(r_start.x(),last_target_pos_.x(),duration);
        traj[1].plan(r_start.y(),last_target_pos_.y(),duration);
        traj[2].plan(r_start.z(),last_target_pos_.z(),duration);
        RCLCPP_INFO(this->get_logger(), "Starting trajectory. Freq: %.1f Hz, Steps: %d", freq, num_steps);

        for (int i=1; i<=num_steps; ++i) {
            if(!rclcpp::ok()) break;
            double t = static_cast<double>(i) * dt;
            Vector3d r_inter={traj[0].getPosition(t), traj[1].getPosition(t), traj[2].getPosition(t)};
            
            if(last_target_mode_==MODE_FULL_POSE){
                double s=t/duration;
                Quaterniond q_rot_inter=q_rot_start.slerp(s,last_target_rot_);
                q_k=inverseKinematics_Pose(r_inter, q_rot_inter.toRotationMatrix(), q_k, q_lim_min, q_lim_max, tol, max_iter, alpha, lambda, false);
            }else{
                q_k=inverseKinematics_PositionOnly(r_inter, q_k, q_lim_min, q_lim_max, tol, max_iter, alpha, lambda, false);
            }
            
            Vector3d r_dot_inter={traj[0].getVelocity(t),traj[1].getVelocity(t),traj[2].getVelocity(t)};
            MatrixXd J_full(6,6), J_P(3,6);
            calculateGeometricJacobian(q_k, J_full);
            J_P = J_full.topRows(3);
            MatrixXd J_P_pinv=pseudoInverseMat(J_P,lambda);
            VectorXd q_dot_p=J_P_pinv*r_dot_inter;
            double k_null=5.0;
            VectorXd q_dot_0=VectorXd::Zero(6);
            q_dot_0(3)=-k_null*q_k(3); q_dot_0(4)=-k_null*q_k(4); q_dot_0(5)=-k_null*q_k(5);
            MatrixXd N=MatrixXd::Identity(6,6)-J_P_pinv*J_P;
            VectorXd q_dot_k=q_dot_p+N*q_dot_0;
            Vector3d r_ddot_inter={traj[0].getAcceleration(t),traj[1].getAcceleration(t),traj[2].getAcceleration(t)};
            MatrixXd J_dot=calculateJacobianDerivative(q_k,q_dot_k,1e-6);
            MatrixXd J_P_dot=J_dot.topRows(3);
            VectorXd q_ddot_k=J_P_pinv*(r_ddot_inter-J_P_dot*q_dot_k);
            
            auto js_msg=std::make_unique<sensor_msgs::msg::JointState>();
            js_msg->header.stamp=this->get_clock()->now();
            js_msg->name={"j1_deg","j2_deg","j3_deg","j4_deg","j5_deg","j6_deg"};
            VectorXd q_k_deg=q_k*180.0/M_PI;
            js_msg->position.assign(q_k_deg.data(),q_k_deg.data()+q_k_deg.size());
            js_msg->velocity.assign(q_dot_k.data(),q_dot_k.data()+q_dot_k.size());
            js_msg->effort.assign(q_ddot_k.data(),q_ddot_k.data()+q_ddot_k.size());
            joint_state_pub_->publish(std::move(js_msg));

            const std::vector<double> gear_ratios={36,36,36,6,6,6};
            VectorXd q_rmd_deg=q_k*180.0/M_PI; q_rmd_deg(2)*=-1.0;
            VectorXd q_rmd(6);
            for(int j=0; j<6; ++j) q_rmd(j)=q_rmd_deg(j)*gear_ratios[j]/0.01;
            auto ik_cmd_msg=std::make_unique<std_msgs::msg::Float64MultiArray>();
            ik_cmd_msg->data.assign(q_rmd.data(),q_rmd.data()+q_rmd.size());
            ik_command_pub_->publish(std::move(ik_cmd_msg));
            
            rate.sleep();
        }

        current_q_ = q_k;
        
        RCLCPP_INFO(this->get_logger(), "Trajectory finished.");
        Matrix4d T_final=jointToPose(current_q_);
        Vector3d r_final=T_final.block<3,1>(0,3);
        Matrix3d R_final=T_final.block<3,3>(0,0);
        double pos_err=(last_target_pos_-r_final).norm();
        double orient_err=(calculateOrientationError(last_target_rot_.toRotationMatrix(),R_final)).norm();
        Vector3d final_euler_deg=rotationMatrixToEulerAngles(R_final)*180.0/M_PI;
        Vector3d target_euler_deg=rotationMatrixToEulerAngles(last_target_rot_.toRotationMatrix())*180.0/M_PI;

        RCLCPP_INFO(this->get_logger(), "-------------------- Trajectory Result --------------------");
        RCLCPP_INFO(this->get_logger(), "Mode: %s", (last_target_mode_==MODE_FULL_POSE?"FULL POSE":"POSITION ONLY"));
        RCLCPP_INFO(this->get_logger(), "Desired Position:  [%.4f, %.4f, %.4f]", last_target_pos_.x(), last_target_pos_.y(), last_target_pos_.z());
        RCLCPP_INFO(this->get_logger(), "Final Position:    [%.4f, %.4f, %.4f] | Error: %.5f (m)", r_final.x(), r_final.y(), r_final.z(), pos_err);
        if(last_target_mode_==MODE_FULL_POSE){
             RCLCPP_INFO(this->get_logger(), "Desired Orient (RPY deg): [%.2f, %.2f, %.2f]", target_euler_deg.x(), target_euler_deg.y(), target_euler_deg.z());
             RCLCPP_INFO(this->get_logger(), "Final Orient (RPY deg):   [%.2f, %.2f, %.2f] | Error Norm: %.5f", final_euler_deg.x(), final_euler_deg.y(), final_euler_deg.z(), orient_err);
        }
        RCLCPP_INFO(this->get_logger(), "-----------------------------------------------------------");
    }

    rclcpp::Subscription<custom_msgs::msg::Target>::SharedPtr target_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr ik_command_pub_;
    std::vector<rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr> motor_angle_subs_;
    std::map<std::string, int> motor_topic_to_joint_index_;
    int8_t last_target_mode_;
    Vector3d last_target_pos_;
    Quaterniond last_target_rot_;
    double last_target_duration_;
    bool target_received_;
    bool button7_pressed_;
    VectorXd current_q_;
    std::array<bool, 6> angles_initialized_;
    bool all_angles_ready_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<InverseKinematicsNode>());
    rclcpp::shutdown();
    return 0;
}
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
// ===== 수정된 부분 1: PoseStamped 메시지 타입 포함 =====
#include "geometry_msgs/msg/pose_stamped.hpp" 
#include <Eigen/Dense>
// ===== 수정된 부분 2: 쿼터니안 및 기하학 변환을 위한 헤더 포함 =====
#include <Eigen/Geometry> 
#include <vector>
#include <array>
#include <cmath>
#include <memory>
#include <chrono>

using namespace Eigen;
using namespace std::chrono_literals;

// 함수 선언부
MatrixXd jointToTransform01(const VectorXd& q);
MatrixXd jointToTransform12(const VectorXd& q);
MatrixXd jointToTransform23(const VectorXd& q);
MatrixXd jointToTransform34(const VectorXd& q);
MatrixXd jointToTransform45(const VectorXd& q);
MatrixXd jointToTransform56(const VectorXd& q);
MatrixXd jointToTransform6E(const VectorXd& q);
// ===== 수정된 부분 3: 최종 변환 행렬을 반환하도록 함수 이름 및 역할 변경 =====
Matrix4d getFinalTransform(const VectorXd& q); 

class ForwardKinematicsNode : public rclcpp::Node
{
public:
    ForwardKinematicsNode() : Node("forward_kinematics_node"), 
                              joint_angles_(VectorXd::Zero(6))
    {
        RCLCPP_INFO(this->get_logger(), "Forward Kinematics Node starting (6-DOF with dual motor)...");
        initialize_subscribers();
        
        // ===== 수정된 부분 4: PointStamped 대신 PoseStamped 퍼블리셔 생성 =====
        // PoseStamped는 위치(position)와 방향(orientation) 정보를 모두 포함합니다.
        pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/robot_end_effector_pose", 10);
        
        timer_ = this->create_wall_timer(
            100ms, std::bind(&ForwardKinematicsNode::calculate_and_publish_pose, this));
        angle_updated_.fill(false);
    }
private:
    void initialize_subscribers() {
        rclcpp::QoS qos_profile(rclcpp::KeepLast(1));
        qos_profile.reliable();

        subs_[0] = this->create_subscription<std_msgs::msg::Float64>("/motor_angles/can10_motor_0x141", qos_profile, 
            [this](const std_msgs::msg::Float64::SharedPtr msg){ update_angle(0, msg->data); });
        subs_[1] = this->create_subscription<std_msgs::msg::Float64>("/motor_angles/can10_motor_0x142", qos_profile, 
            [this](const std_msgs::msg::Float64::SharedPtr msg){ update_angle(1, msg->data); });
        subs_[2] = this->create_subscription<std_msgs::msg::Float64>("/motor_angles/can10_motor_0x144", qos_profile, 
            [this](const std_msgs::msg::Float64::SharedPtr msg){ update_angle(2, -msg->data); });
        subs_[5] = this->create_subscription<std_msgs::msg::Float64>("/motor_angles/can11_motor_0x142", qos_profile, 
            [this](const std_msgs::msg::Float64::SharedPtr msg){ update_angle(5, msg->data); });
        subs_[4] = this->create_subscription<std_msgs::msg::Float64>("/motor_angles/can11_motor_0x143", qos_profile, 
            [this](const std_msgs::msg::Float64::SharedPtr msg){ update_angle(4, -msg->data); });
        subs_[3] = this->create_subscription<std_msgs::msg::Float64>("/motor_angles/can11_motor_0x144", qos_profile, 
            [this](const std_msgs::msg::Float64::SharedPtr msg){ update_angle(3, msg->data); });

        for(const auto& sub : subs_){
             RCLCPP_INFO(this->get_logger(), "Subscribed to %s", sub->get_topic_name());
        }
    }

    void update_angle(int index, double angle_deg) {
        joint_angles_(index) = angle_deg * M_PI / 180.0;
        angle_updated_[index] = true;
    }

    // ===== 수정된 부분 5: 위치와 방향을 모두 계산하고 퍼블리시하는 함수 =====
    void calculate_and_publish_pose() {
        bool all_updated = std::all_of(angle_updated_.begin(), angle_updated_.end(), [](bool v){ return v; });

        if (!all_updated) {
            RCLCPP_WARN_ONCE(this->get_logger(), "Waiting for all 6 joint angles to be published...");
            return;
        }

        // 1. 최종 변환 행렬(T_0_E) 계산
        Matrix4d T_final = getFinalTransform(joint_angles_);

        // 2. 위치(Position) 추출
        Vector3d position = T_final.block<3,1>(0,3);

        // 3. 방향(Orientation)을 회전 행렬(Rotation Matrix)로 추출
        Matrix3d rotation_matrix = T_final.block<3,3>(0,0);

        // 4. 회전 행렬을 쿼터니안으로 변환
        Quaterniond q(rotation_matrix);
        q.normalize(); // 정규화하여 수치적 안정성 확보

        // 5. 쿼터니안을 오일러 각(ZYX 순서: Yaw, Pitch, Roll)으로 변환 (디버깅/로깅용)
        // Eigen의 eulerAngles(2, 1, 0)는 Z-Y-X 순서의 회전을 의미합니다.
        Vector3d euler_angles_rad = q.toRotationMatrix().eulerAngles(2, 1, 0); // 라디안 단위
        Vector3d euler_angles_deg = euler_angles_rad * (180.0 / M_PI); // 각도 단위
        
        // 로그 출력
        RCLCPP_INFO(this->get_logger(), "End-Effector Pose:");
        RCLCPP_INFO(this->get_logger(), "  Position (x, y, z) = (%.4f, %.4f, %.4f)", position.x(), position.y(), position.z());
        RCLCPP_INFO(this->get_logger(), "  Euler Angles (Roll, Pitch, Yaw) [deg] = (%.2f, %.2f, %.2f)", 
                euler_angles_deg(2), // Roll 값 (벡터의 3번째 요소)
                euler_angles_deg(1), // Pitch 값 (벡터의 2번째 요소)
                euler_angles_deg(0)  // Yaw 값 (벡터의 1번째 요소)
               );
        // 6. PoseStamped 메시지 생성 및 퍼블리시
        auto pose_msg = geometry_msgs::msg::PoseStamped();
        pose_msg.header.stamp = this->get_clock()->now();
        pose_msg.header.frame_id = "base_link"; 
        
        // 위치 정보 채우기
        pose_msg.pose.position.x = position.x();
        pose_msg.pose.position.y = position.y();
        pose_msg.pose.position.z = position.z();
        
        // 방향 정보(쿼터니안) 채우기
        pose_msg.pose.orientation.x = q.x();
        pose_msg.pose.orientation.y = q.y();
        pose_msg.pose.orientation.z = q.z();
        pose_msg.pose.orientation.w = q.w();

        pose_publisher_->publish(pose_msg);
    }
    
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subs_[6];
    // ===== 수정된 부분 6: 퍼블리셔 타입 변경 =====
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    VectorXd joint_angles_;
    std::array<bool, 6> angle_updated_;
};

// ==========================================================
// 변환 행렬 함수들 (이 부분은 원본과 동일)
// ==========================================================
MatrixXd jointToTransform01(const VectorXd& q){
    MatrixXd tmp_m(4,4);
    Vector3d r = {0,0,-0.01525};
    double q1 = q(0);
    tmp_m(0,0) = cos(q1);   tmp_m(0,1) = -sin(q1);  tmp_m(0,2) = 0;      tmp_m(0,3) = r(0);
    tmp_m(1,0) = sin(q1);   tmp_m(1,1) =  cos(q1);  tmp_m(1,2) = 0;      tmp_m(1,3) = r(1);
    tmp_m(2,0) = 0;         tmp_m(2,1) = 0;         tmp_m(2,2) = 1;      tmp_m(2,3) = r(2);
    tmp_m(3,0) = 0;         tmp_m(3,1) = 0;         tmp_m(3,2) = 0;      tmp_m(3,3) = 1;
    return tmp_m;
}
MatrixXd jointToTransform12(const VectorXd& q){
    MatrixXd tmp_m(4,4);
    Vector3d r = {0,0,0.09925};
    double qq = q(1);
    tmp_m(0,0) = cos(qq);   tmp_m(0,1) = 0;         tmp_m(0,2) = sin(qq);   tmp_m(0,3) = r(0);
    tmp_m(1,0) = 0;         tmp_m(1,1) = 1;         tmp_m(1,2) = 0;         tmp_m(1,3) = r(1);
    tmp_m(2,0) = -sin(qq);  tmp_m(2,1) = 0;         tmp_m(2,2) = cos(qq);   tmp_m(2,3) = r(2);
    tmp_m(3,0) = 0;         tmp_m(3,1) = 0;         tmp_m(3,2) = 0;         tmp_m(3,3) = 1;
    return tmp_m;
}
MatrixXd jointToTransform23(const VectorXd& q){
    MatrixXd tmp_m(4,4);
    Vector3d r = {-0.34865,0,0};
    double qq = q(2);
    tmp_m(0,0) = cos(qq);   tmp_m(0,1) = 0;         tmp_m(0,2) = sin(qq);   tmp_m(0,3) = r(0);
    tmp_m(1,0) = 0;         tmp_m(1,1) = 1;         tmp_m(1,2) = 0;         tmp_m(1,3) = r(1);
    tmp_m(2,0) = -sin(qq);  tmp_m(2,1) = 0;         tmp_m(2,2) = cos(qq);   tmp_m(2,3) = r(2);
    tmp_m(3,0) = 0;         tmp_m(3,1) = 0;         tmp_m(3,2) = 0;         tmp_m(3,3) = 1;
    return tmp_m;
}
MatrixXd jointToTransform34(const VectorXd& q){
    MatrixXd tmp_m(4,4);
    Vector3d r = {0.2518,0,0.08347};
    double qq = q(3);
    tmp_m(0,0) = 1;         tmp_m(0,1) = 0;         tmp_m(0,2) = 0;         tmp_m(0,3) = r(0);
    tmp_m(1,0) = 0;         tmp_m(1,1) = cos(qq);   tmp_m(1,2) = -sin(qq);  tmp_m(1,3) = r(1);
    tmp_m(2,0) = 0;         tmp_m(2,1) = sin(qq);   tmp_m(2,2) = cos(qq);   tmp_m(2,3) = r(2);
    tmp_m(3,0) = 0;         tmp_m(3,1) = 0;         tmp_m(3,2) = 0;         tmp_m(3,3) = 1;
    return tmp_m;
}

MatrixXd jointToTransform45(const VectorXd& q){
    MatrixXd tmp_m(4,4);
    Vector3d r = {0.06049,0,0};
    double qq = q(4);
    tmp_m(0,0) = cos(qq);   tmp_m(0,1) = sin(qq);  tmp_m(0,2) = 0;      tmp_m(0,3) = r(0);
    tmp_m(1,0) = -sin(qq);  tmp_m(1,1) =  cos(qq);  tmp_m(1,2) = 0;      tmp_m(1,3) = r(1);
    tmp_m(2,0) = 0;         tmp_m(2,1) = 0;         tmp_m(2,2) = 1;      tmp_m(2,3) = r(2);
    tmp_m(3,0) = 0;         tmp_m(3,1) = 0;         tmp_m(3,2) = 0;      tmp_m(3,3) = 1;
    return tmp_m;
}

MatrixXd jointToTransform56(const VectorXd& q){
    MatrixXd tmp_m(4,4);
    Vector3d r = {0.05815,0,0};
    double qq = q(5);
    tmp_m(0,0) = 1;         tmp_m(0,1) = 0;         tmp_m(0,2) = 0;         tmp_m(0,3) = r(0);
    tmp_m(1,0) = 0;         tmp_m(1,1) = cos(qq);   tmp_m(1,2) = -sin(qq);  tmp_m(1,3) = r(1);
    tmp_m(2,0) = 0;         tmp_m(2,1) = sin(qq);   tmp_m(2,2) = cos(qq);   tmp_m(2,3) = r(2);
    tmp_m(3,0) = 0;         tmp_m(3,1) = 0;         tmp_m(3,2) = 0;         tmp_m(3,3) = 1;
    return tmp_m;
}
MatrixXd jointToTransform6E(const VectorXd& q){
    MatrixXd tmp_m(4,4);
    Vector3d r = {0.02215,0,0};
    tmp_m(0,0) = 1;         tmp_m(0,1) = 0;   tmp_m(0,2) = 0;   tmp_m(0,3) = r(0);
    tmp_m(1,0) = 0;         tmp_m(1,1) = 1;   tmp_m(1,2) = 0;   tmp_m(1,3) = r(1);
    tmp_m(2,0) = 0;         tmp_m(2,1) = 0;   tmp_m(2,2) = 1;   tmp_m(2,3) = r(2);
    tmp_m(3,0) = 0;         tmp_m(3,1) = 0;   tmp_m(3,2) = 0;   tmp_m(3,3) = 1;
    return tmp_m;
}

// ===== 수정된 부분 7: 기존 jointToPosition 함수를 수정하여 전체 변환 행렬을 반환 =====
Matrix4d getFinalTransform(const VectorXd& q){
    // Eigen::Matrix4d는 MatrixXd(4,4)와 동일합니다. 타입 명시성을 위해 사용.
    Matrix4d T_IE = jointToTransform01(q) * 
                    jointToTransform12(q) * 
                    jointToTransform23(q) * 
                    jointToTransform34(q) *
                    jointToTransform45(q) *
                    jointToTransform56(q) *
                    jointToTransform6E(q);
    return T_IE;
}

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ForwardKinematicsNode>());
    rclcpp::shutdown();
    return 0;
}
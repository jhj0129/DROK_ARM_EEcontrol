#include "rclcpp/rclcpp.hpp"
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <cmath>
#include <memory>
#include <iostream>
#include <iomanip>
#include <algorithm> // for std::clamp
#include <map>
#include <array>

#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "custom_msgs/msg/target.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

using namespace Eigen;
using namespace std::chrono_literals;

// ANSI Color Codes
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
    void plan(double p_start, double p_end, double duration) {
        if(duration <= 1e-6){ c(0) = p_start; return; }
        double T=duration, T2=T*T, T3=T2*T, T4=T3*T, T5=T4*T;
        c(0)=p_start; c(1)=0; c(2)=0;
        c(3)=10*(p_end-p_start)/T3; c(4)=-15*(p_end-p_start)/T4; c(5)=6*(p_end-p_start)/T5;
    }
    double getPosition(double t){ return c(0)+c(1)*t+c(2)*pow(t,2)+c(3)*pow(t,3)+c(4)*pow(t,4)+c(5)*pow(t,5); }
    double getVelocity(double t){ return c(1)+2*c(2)*t+3*c(3)*t*t+4*c(4)*pow(t,3)+5*c(5)*pow(t,4); }
};

// ==========================================================
// 함수 선언부
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
void calculateGeometricJacobian(const VectorXd& q, MatrixXd& J_P_out, MatrixXd& J_R_out);
VectorXd solveDLS_robust(const MatrixXd& J, double lambda, const VectorXd& error);
VectorXd inverseKinematics(const Vector3d& r_des, const Matrix3d& R_des, bool is_full_pose, VectorXd q0, const VectorXd& q_min, const VectorXd& q_max, double tol, int max_iter, double alpha, double lambda);
Vector3d rotationMatrixToEulerAngles(const Matrix3d& R);

// ==========================================================
// 순방향 기구학 (Forward Kinematics) - 교체된 모델
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
    tmp_m(0,0) = cos(qq);   tmp_m(0,1) = -sin(qq);  tmp_m(0,2) = 0;      tmp_m(0,3) = r(0);
    tmp_m(1,0) = sin(qq);   tmp_m(1,1) =  cos(qq);  tmp_m(1,2) = 0;      tmp_m(1,3) = r(1);
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
    (void)q; // Unused parameter
    MatrixXd tmp_m(4,4);
    Vector3d r = {0.02215,0,0};
    tmp_m(0,0) = 1;         tmp_m(0,1) = 0;   tmp_m(0,2) = 0;   tmp_m(0,3) = r(0);
    tmp_m(1,0) = 0;         tmp_m(1,1) = 1;   tmp_m(1,2) = 0;   tmp_m(1,3) = r(1);
    tmp_m(2,0) = 0;         tmp_m(2,1) = 0;   tmp_m(2,2) = 1;   tmp_m(2,3) = r(2);
    tmp_m(3,0) = 0;         tmp_m(3,1) = 0;   tmp_m(3,2) = 0;   tmp_m(3,3) = 1;
    return tmp_m;
}

Matrix4d jointToPose(const VectorXd& q){
    return jointToTransform01(q) * jointToTransform12(q) * jointToTransform23(q) * 
           jointToTransform34(q) * jointToTransform45(q) * jointToTransform56(q) * 
           jointToTransform6E(q);
}
// ==========================================================
// 자코비안 및 관련 함수
// ==========================================================
void calculateGeometricJacobian(const VectorXd& q, MatrixXd& J_P_out, MatrixXd& J_R_out) {
    J_P_out=MatrixXd::Zero(3,6);J_R_out=MatrixXd::Zero(3,6);
    Matrix4d T_f[7];T_f[0]=jointToTransform01(q);T_f[1]=jointToTransform12(q);T_f[2]=jointToTransform23(q);T_f[3]=jointToTransform34(q);T_f[4]=jointToTransform45(q);T_f[5]=jointToTransform56(q);T_f[6]=jointToTransform6E(q);
    Matrix4d T_g[8];T_g[0]=Matrix4d::Identity();for(int i=0;i<7;++i)T_g[i+1]=T_g[i]*T_f[i];
    Vector3d p_E=T_g[7].block<3,1>(0,3);
    Vector3d la[6]={{0,0,1},{0,1,0},{0,1,0},{1,0,0},{0,0,1},{1,0,0}};
    for(int i=0;i<6;++i){
        Vector3d p_p=T_g[i].block<3,1>(0,3);Matrix3d R_p=T_g[i].block<3,3>(0,0);
        Vector3d z_g=R_p*la[i];
        J_P_out.col(i)=z_g.cross(p_E-p_p);J_R_out.col(i)=z_g;
    }
}
Vector3d calculateOrientationError(const Matrix3d& R_des,const Matrix3d& R_curr){return 0.5*(R_curr.col(0).cross(R_des.col(0))+R_curr.col(1).cross(R_des.col(1))+R_curr.col(2).cross(R_des.col(2)));}
Vector3d rotationMatrixToEulerAngles(const Matrix3d& R){double sy=sqrt(R(0,0)*R(0,0)+R(1,0)*R(1,0));bool s=sy<1e-6;double x,y,z;if(!s){x=atan2(R(2,1),R(2,2));y=atan2(-R(2,0),sy);z=atan2(R(1,0),R(0,0));}else{x=atan2(-R(1,2),R(1,1));y=atan2(-R(2,0),sy);z=0;}return {x,y,z};}

// [★★★★★ 핵심: 제시해주신 안정적인 DLS 알고리즘 ★★★★★]
VectorXd solveDLS_robust(const MatrixXd& J, double lambda, const VectorXd& error) {
    MatrixXd A = J * J.transpose() + (lambda * lambda) * MatrixXd::Identity(J.rows(), J.rows());
    return J.transpose() * A.colPivHouseholderQr().solve(error);
}

// [통합 IK 함수]
VectorXd inverseKinematics(const Vector3d& r_des, const Matrix3d& R_des, bool is_full_pose, VectorXd q0, const VectorXd& q_min, const VectorXd& q_max, double tol, int max_iter, double alpha, double lambda) {
    VectorXd q = q0;
    for(int i=0; i < max_iter; ++i) {
        VectorXd error; MatrixXd J; Matrix4d T_curr = jointToPose(q);
        MatrixXd J_P(3,6), J_R(3,6); calculateGeometricJacobian(q, J_P, J_R);
        
        if (is_full_pose) {
            error.resize(6); J.resize(6, 6);
            error.head(3) = r_des - T_curr.block<3,1>(0,3); error.tail(3) = calculateOrientationError(R_des, T_curr.block<3,3>(0,0));
            J.topRows(3) = J_P; J.bottomRows(3) = J_R;
        } else {
            error.resize(3); J.resize(3, 6);
            error = r_des - T_curr.block<3,1>(0,3);
            J = J_P;
        }
        
        if (error.norm() < tol) break;
        
        q += alpha * solveDLS_robust(J, lambda, error);
        
        for (int j=0; j<6; ++j) {
            q(j) = std::clamp(q(j), q_min(j), q_max(j));
        }
    }
    return q;
}

// ==========================================================
// ROS 2 IK Node
// ==========================================================
class InverseKinematicsNode : public rclcpp::Node
{
public:
    InverseKinematicsNode():Node("inverse_kinematics_node"),target_received_(false),button7_pressed_(false),all_angles_ready_(false){
        angles_initialized_.fill(false);
        auto qos=rclcpp::QoS(rclcpp::KeepLast(10));
        target_sub_=this->create_subscription<custom_msgs::msg::Target>("/trajectory_target",qos,std::bind(&InverseKinematicsNode::target_callback,this,std::placeholders::_1));
        joy_sub_=this->create_subscription<sensor_msgs::msg::Joy>("/joy",qos,std::bind(&InverseKinematicsNode::joy_callback,this,std::placeholders::_1));
        joint_state_pub_=this->create_publisher<sensor_msgs::msg::JointState>("/plotjuggler/joint_states",qos);
        ik_command_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/ik_joint_commands", qos);
        current_q_=VectorXd::Zero(6);
        initialize_motor_angle_subscribers();
        RCLCPP_INFO(this->get_logger(), "IK Trajectory Node has been started.");
    }
private:
    void initialize_motor_angle_subscribers(){
        motor_topic_to_joint_index_={{"/motor_angles/can10_motor_0x141",0},{"/motor_angles/can10_motor_0x142",1},{"/motor_angles/can10_motor_0x144",2},{"/motor_angles/can11_motor_0x144",3},{"/motor_angles/can11_motor_0x143",4},{"/motor_angles/can11_motor_0x142",5}};
        motor_angle_subs_.resize(motor_topic_to_joint_index_.size());
        auto qos_m=rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
        int i=0;for(const auto&p:motor_topic_to_joint_index_){
            auto cb=[this,j=p.second](const std_msgs::msg::Float64::SharedPtr msg){
                this->current_q_(j)=(j==2)?-msg->data*M_PI/180.0:msg->data*M_PI/180.0;
                if(!this->angles_initialized_[j]){this->angles_initialized_[j]=true;
                if(std::all_of(angles_initialized_.begin(),angles_initialized_.end(),[](bool v){return v;})){all_angles_ready_=true;RCLCPP_INFO(this->get_logger(),"All angles received.");}}
            };
            motor_angle_subs_[i++]=this->create_subscription<std_msgs::msg::Float64>(p.first,qos_m,cb);
        }
    }
    void target_callback(const custom_msgs::msg::Target::SharedPtr msg){
        last_target_mode_=msg->mode;last_target_pos_<<msg->position.x,msg->position.y,msg->position.z;
        last_target_rot_=Quaterniond(msg->orientation.w,msg->orientation.x,msg->orientation.y,msg->orientation.z).normalized();
        last_target_duration_=msg->duration_s;target_received_=true;
        RCLCPP_INFO(this->get_logger(),"New target(Mode: %s): Pos[%.3f,%.3f,%.3f], Duration: %.2f", (last_target_mode_==1?"POSE":"POS"), last_target_pos_.x(),last_target_pos_.y(),last_target_pos_.z(),last_target_duration_);
    }
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg){
        if(msg->buttons.size()>7&&msg->buttons[7]==1){if(!button7_pressed_){button7_pressed_=true;
        if(!target_received_){RCLCPP_WARN(this->get_logger(),"Btn pressed, but no target.");return;}
        if(!all_angles_ready_){RCLCPP_WARN(this->get_logger(),"Btn pressed, but angles not ready.");return;}
        RCLCPP_INFO(this->get_logger(),"Btn pressed. Starting trajectory.");calculate_and_publish_trajectory();
        }}else{button7_pressed_=false;}
    }
    void calculate_and_publish_trajectory(){
        VectorXd q_lim_min(6),q_lim_max(6);q_lim_min<<0,0,-180,-90,-50,-90;q_lim_max<<130,180,0,90,50,90;
        q_lim_min*=M_PI/180.0;q_lim_max*=M_PI/180.0;
        
        double tol=1e-4, alpha=0.1, lambda=0.05, max_iter=500;

        const int n_steps=500;const double dur=std::max(last_target_duration_,0.1);
        rclcpp::Rate rate(static_cast<double>(n_steps)/dur);const double dt=dur/n_steps;
        VectorXd q_k=current_q_;Vector3d r_s=jointToPose(q_k).block<3,1>(0,3);Quaterniond rot_s(jointToPose(q_k).block<3,3>(0,0));
        std::vector<QuinticPolynomial> traj(3);
        traj[0].plan(r_s.x(),last_target_pos_.x(),dur);traj[1].plan(r_s.y(),last_target_pos_.y(),dur);traj[2].plan(r_s.z(),last_target_pos_.z(),dur);
        RCLCPP_INFO(this->get_logger(),"Trajectory start. Freq: %.1f, Steps: %d",static_cast<double>(n_steps)/dur,n_steps);
        
        for(int i=1;i<=n_steps;++i){
            if(!rclcpp::ok())break;
            double t=static_cast<double>(i)*dt;
            Vector3d r_i={traj[0].getPosition(t),traj[1].getPosition(t),traj[2].getPosition(t)};
            Matrix3d R_i=rot_s.slerp(t/dur,last_target_rot_).toRotationMatrix();
            bool is_full_pose=(last_target_mode_==MODE_FULL_POSE);
            q_k=inverseKinematics(r_i,R_i,is_full_pose,q_k,q_lim_min,q_lim_max,tol,max_iter,alpha,lambda);
            
            auto js=std::make_unique<sensor_msgs::msg::JointState>();
            js->header.stamp=this->get_clock()->now();js->name={"j1_rad","j2_rad","j3_rad","j4_rad","j5_rad","j6_rad"};
            js->position.assign(q_k.data(),q_k.data()+q_k.size());
            joint_state_pub_->publish(std::move(js));
            
            const std::vector<double>gear_ratios={36,36,36,6,6,6};
            VectorXd q_rmd_deg=q_k*180.0/M_PI;q_rmd_deg(2)*=-1.0;
            VectorXd q_rmd(6);for(int j=0;j<6;++j)q_rmd(j)=q_rmd_deg(j)*gear_ratios[j]/0.01;
            auto ik_cmd=std::make_unique<std_msgs::msg::Float64MultiArray>();
            ik_cmd->data.assign(q_rmd.data(),q_rmd.data()+q_rmd.size());
            ik_command_pub_->publish(std::move(ik_cmd));

            rate.sleep();
        }
        current_q_=q_k;
        RCLCPP_INFO(this->get_logger(),"Trajectory finished.");
        Matrix4d T_f=jointToPose(current_q_);Vector3d r_f=T_f.block<3,1>(0,3);Matrix3d R_f=T_f.block<3,3>(0,0);
        double p_err=(last_target_pos_-r_f).norm();double o_err=(calculateOrientationError(last_target_rot_.toRotationMatrix(),R_f)).norm();
        Vector3d e_f=rotationMatrixToEulerAngles(R_f)*180.0/M_PI;Vector3d e_d=rotationMatrixToEulerAngles(last_target_rot_.toRotationMatrix())*180.0/M_PI;
        RCLCPP_INFO(this->get_logger(),"-------------------- Result --------------------");
        RCLCPP_INFO(this->get_logger(),"Mode: %s",(last_target_mode_==1?"POSE":"POS"));
        RCLCPP_INFO(this->get_logger(),"Desired Pos: [%.4f, %.4f, %.4f]",last_target_pos_.x(),last_target_pos_.y(),last_target_pos_.z());
        RCLCPP_INFO(this->get_logger(),"Final Pos:   [%.4f, %.4f, %.4f] | Err: %.5f",r_f.x(),r_f.y(),r_f.z(),p_err);
        if(last_target_mode_==1){
            RCLCPP_INFO(this->get_logger(),"Desired RPY: [%.2f, %.2f, %.2f]",e_d.x(),e_d.y(),e_d.z());
            RCLCPP_INFO(this->get_logger(),"Final RPY:   [%.2f, %.2f, %.2f] | Err: %.5f",e_f.x(),e_f.y(),e_f.z(),o_err);
        }
        RCLCPP_INFO(this->get_logger(),"----------------------------------------------");
    }
    
    rclcpp::Subscription<custom_msgs::msg::Target>::SharedPtr target_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr ik_command_pub_;
    std::vector<rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr> motor_angle_subs_;
    std::map<std::string,int> motor_topic_to_joint_index_;
    int8_t last_target_mode_;Vector3d last_target_pos_;Quaterniond last_target_rot_;
    double last_target_duration_;bool target_received_,button7_pressed_,all_angles_ready_;
    VectorXd current_q_;
    std::array<bool, 6> angles_initialized_;
};

int main(int argc,char*argv[]){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<InverseKinematicsNode>());
    rclcpp::shutdown();
    return 0;
}
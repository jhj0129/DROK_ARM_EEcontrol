#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include <sys/socket.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>

#include <chrono>
#include <vector>
#include <string>
#include <sstream>
#include <iomanip>
#include <mutex>
#include <cmath>
#include <map>
#include <sched.h>
#include <sys/mman.h>

#define _USE_MATH_DEFINES // M_PI와 같은 수학 상수 사용을 위해
using namespace std::chrono_literals;

// 제어 모드를 명확하게 구분하기 위한 열거형 클래스
enum class ControlMode {
    MANUAL_JOYSTICK, // 조이스틱 수동 제어 모드
    IK_AUTO          // IK 기반 자동 제어 모드
};

class JoystickToRMDControl : public rclcpp::Node {
private:
    // 각 모터의 상태 정보를 저장하는 구조체
    struct MotorState {
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr angle_subscriber; // 각도 피드백 구독자
        double current_angle_degrees = 0.0;     // 현재 각도 (도)
        bool has_received_initial_angle = false; // 초기 각도 수신 여부
        std::mutex data_mutex;                   // 데이터 접근 동기화를 위한 뮤텍스
        double min_angle_limit_deg = -1.0e6;     // 최소 각도 제한 (도)
        double max_angle_limit_deg = 1.0e6;     // 최대 각도 제한 (도)
    };

    // 각 모터의 하드웨어 설정 정보를 저장하는 구조체
    struct MotorConfig {
        std::string name;       // 모터 이름
        int can_socket;         // 사용하는 CAN 소켓 디스크립터
        int motor_id;           // 모터 CAN ID
        int motor_vector_index; // 내부 데이터 벡터에서의 인덱스
    };

public:
    JoystickToRMDControl() : Node("joystick_to_rmd_control"),
                             motor_states_(8), // 8개의 모터 상태 초기화
                             current_mode_(ControlMode::MANUAL_JOYSTICK) // 초기 모드는 수동
    {
        RCLCPP_INFO(this->get_logger(), "Joystick to RMD Control 노드 초기화 중...");

        // CAN 소켓 초기화
        can_socket_10_ = open_can_socket("can10");
        can_socket_11_ = open_can_socket("can11");
        if (!can_sockets_ok()) {
            RCLCPP_FATAL(this->get_logger(), "CAN 소켓 열기 실패. 노드를 종료합니다...");
            rclcpp::shutdown();
            return;
        }

        // 각종 설정 초기화
        initialize_motor_configs();
        initialize_gear_ratios();
        initialize_parameters();
        initialize_subscribers();
        initialize_ik_command_subscriber();

        // 조이스틱 메시지 구독자 생성
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", rclcpp::SensorDataQoS(), std::bind(&JoystickToRMDControl::joy_callback, this, std::placeholders::_1));

        // 내부 변수들 크기 설정
        target_angleControl_motors_.resize(motor_configs_.size(), 0);
        internal_target_angles_.resize(motor_configs_.size(), 0);

        // 디버깅용 퍼블리셔 생성
        debug_angles_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/motor_debug/can11_group2_angles_deg", rclcpp::QoS(10));


        RCLCPP_INFO(this->get_logger(), "모터 드라이버 활성화 및 0점 이동 명령 전송...");
        send_startup_raw_frames();
        
        RCLCPP_INFO(this->get_logger(), "초기 각도 수신 대기 중...");
        wait_for_initial_angles();
        
        RCLCPP_INFO(this->get_logger(), "목표 각도를 0으로 초기화합니다...");
        initialize_target_angles();

        // 10ms 주기의 제어 타이머 생성
        timer_ = this->create_wall_timer(command_interval_, std::bind(&JoystickToRMDControl::timer_callback, this));
        last_callback_start_time_ = std::chrono::steady_clock::now();
        RCLCPP_INFO(this->get_logger(), "초기화 완료. 현재 모드: MANUAL_JOYSTICK");
        RCLCPP_INFO(this->get_logger(), "안전을 위해 조이스틱을 움직여 수동 제어를 활성화해주세요.");
    }

    ~JoystickToRMDControl() {
        // 노드 소멸 시 CAN 소켓 정리
        close_can_sockets();
    }

private:
    /**
     * @brief RMD 모터에 위치 제어 명령을 전송합니다.
     */
    void rmd_position_control(int s, int ID, int32_t angleControl, uint16_t maxSpeed) {
        if (s < 0) return;
        struct can_frame frame;
        frame.can_id = ID;
        frame.can_dlc = 8;
        frame.data[0] = 0xA4; // 위치 제어 명령 코드
        frame.data[1] = 0x00;
        
        std::memcpy(&frame.data[2], &maxSpeed, 2);
        std::memcpy(&frame.data[4], &angleControl, 4);

        if (write(s, &frame, sizeof(frame)) != sizeof(frame)) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "CAN Write 실패 (ID: 0x%X). errno: %d (%s)", ID, errno, strerror(errno));
        }
    }

    /**
     * @brief Raw CAN 프레임을 전송합니다.
     */
    void rmd_send_raw_frame(int s, uint32_t can_id, const std::vector<uint8_t>& data) {
        if (s < 0) return;
        struct can_frame frame;
        frame.can_id = can_id;
        frame.can_dlc = data.size();
        std::memcpy(frame.data, data.data(), data.size());
        if (write(s, &frame, sizeof(frame)) != sizeof(frame)) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Raw CAN 프레임 전송 실패 (ID: 0x%X). errno: %d (%s)", can_id, errno, strerror(errno));
        }
    }

    // 8개 모터의 설정 정보를 하드코딩으로 초기화
    void initialize_motor_configs() {
        motor_configs_ = {
            {"can10_motor1", can_socket_10_, 0x141, 0}, {"can10_motor4", can_socket_10_, 0x144, 1},
            {"can10_motor2", can_socket_10_, 0x142, 2}, {"can10_motor3", can_socket_10_, 0x143, 3},
            {"can11_motor2", can_socket_11_, 0x142, 4}, {"can11_motor1", can_socket_11_, 0x141, 5},
            {"can11_motor4", can_socket_11_, 0x144, 6}, {"can11_motor3", can_socket_11_, 0x143, 7}
        };
    }

    // 모터별 기어비를 초기화
    void initialize_gear_ratios() {
        motor_gear_ratios_.resize(motor_configs_.size());
        for(const auto& config : motor_configs_) {
            if (config.can_socket == can_socket_10_) {
                motor_gear_ratios_[config.motor_vector_index] = 36.0;
            } else if (config.can_socket == can_socket_11_) {
                 if (config.motor_id == 0x141 || config.motor_id == 0x142) {
                    motor_gear_ratios_[config.motor_vector_index] = 1.0;
                } else {
                    motor_gear_ratios_[config.motor_vector_index] = 6.0;
                }
            }
        }
    }

    // ROS 파라미터 서버로부터 각종 설정을 불러와 초기화
    void initialize_parameters() {
        // 조이스틱 축 및 반전 설정
        this->declare_parameter<int>("axis.can10_motor1", 3);
        this->declare_parameter<int>("axis.can10_motor4", 1);
        this->declare_parameter<int>("axis.can10_motor2", 4);
        this->declare_parameter<int>("axis.can11_motor1", 0);
        this->declare_parameter<int>("axis.can11_motor2", 7);
        this->declare_parameter<int>("axis.can11_motor3", 6);
        this->declare_parameter<bool>("invert.can10_motor1", false);
        this->declare_parameter<bool>("invert.can10_motor4", false);
        this->declare_parameter<bool>("invert.can10_motor2", false);
        this->declare_parameter<bool>("invert.can11_motor1", false);
        this->declare_parameter<bool>("invert.can11_motor2", false);
        this->declare_parameter<bool>("invert.can11_motor3", false);
        this->declare_parameter<bool>("invert.can11_motor4", false);

        axis_can10_motor1_ = this->get_parameter("axis.can10_motor1").as_int();
        axis_can10_motor4_ = this->get_parameter("axis.can10_motor4").as_int();
        axis_can10_motor2_ = this->get_parameter("axis.can10_motor2").as_int();
        axis_can11_motor1_ = this->get_parameter("axis.can11_motor1").as_int();
        axis_can11_motor2_ = this->get_parameter("axis.can11_motor2").as_int();
        axis_can11_motor3_ = this->get_parameter("axis.can11_motor3").as_int();
        invert_can10_motor1_ = this->get_parameter("invert.can10_motor1").as_bool();
        invert_can10_motor4_ = this->get_parameter("invert.can10_motor4").as_bool();
        invert_can10_motor2_ = this->get_parameter("invert.can10_motor2").as_bool();
        invert_can11_motor1_ = this->get_parameter("invert.can11_motor1").as_bool();
        invert_can11_motor2_ = this->get_parameter("invert.can11_motor2").as_bool();
        invert_can11_motor3_ = this->get_parameter("invert.can11_motor3").as_bool();
        invert_can11_motor4_ = this->get_parameter("invert.can11_motor4").as_bool();
        
        // 그룹별 튜닝 파라미터 선언 및 로드
        // 그룹 1: can10 모터들
        this->declare_parameter<std::vector<double>>("group_can10.speed_scales", {70000.0, 150000.0, 300000.0});
        this->declare_parameter<double>("group_can10.cos_acceleration_scale", 1.0);
        this->declare_parameter<double>("group_can10.cos_curve_factor", 1.0);
        this->declare_parameter<double>("group_can10.curve_exponent", 2.0);
        this->declare_parameter<int>("group_can10.max_speed_scale", 65534);
        speed_scales_can10_ = this->get_parameter("group_can10.speed_scales").as_double_array();
        cos_acceleration_scale_can10_ = this->get_parameter("group_can10.cos_acceleration_scale").as_double();
        cos_curve_factor_can10_ = this->get_parameter("group_can10.cos_curve_factor").as_double();
        curve_exponent_can10_ = this->get_parameter("group_can10.curve_exponent").as_double();
        max_speed_scale_can10_ = this->get_parameter("group_can10.max_speed_scale").as_int();

        // 그룹 2: can11 모터들 (id1, id2)
        this->declare_parameter<std::vector<double>>("group_can11_id1_id2.speed_scales", {70000.0, 150000.0, 300000.0});
        this->declare_parameter<double>("group_can11_id1_id2.cos_acceleration_scale", 1.0);
        this->declare_parameter<double>("group_can11_id1_id2.cos_curve_factor", 1.0);
        this->declare_parameter<double>("group_can11_id1_id2.curve_exponent", 1.0);
        this->declare_parameter<int>("group_can11_id1_id2.max_speed_scale", 65534);
        speed_scales_can11_g1_ = this->get_parameter("group_can11_id1_id2.speed_scales").as_double_array();
        cos_acceleration_scale_can11_g1_ = this->get_parameter("group_can11_id1_id2.cos_acceleration_scale").as_double();
        cos_curve_factor_can11_g1_ = this->get_parameter("group_can11_id1_id2.cos_curve_factor").as_double();
        curve_exponent_can11_g1_ = this->get_parameter("group_can11_id1_id2.curve_exponent").as_double();
        max_speed_scale_can11_g1_ = this->get_parameter("group_can11_id1_id2.max_speed_scale").as_int();

        // 그룹 3: can11 모터들 (id3, id4)
        this->declare_parameter<std::vector<double>>("group_can11_id3_id4.speed_scales", {150000.0, 300000.0, 500000.0});
        this->declare_parameter<double>("group_can11_id3_id4.cos_acceleration_scale", 1.0);
        this->declare_parameter<double>("group_can11_id3_id4.cos_curve_factor", 1.0);
        this->declare_parameter<double>("group_can11_id3_id4.curve_exponent", 2.0);
        this->declare_parameter<int>("group_can11_id3_id4.max_speed_scale", 65534);
        speed_scales_can11_g2_ = this->get_parameter("group_can11_id3_id4.speed_scales").as_double_array();
        cos_acceleration_scale_can11_g2_ = this->get_parameter("group_can11_id3_id4.cos_acceleration_scale").as_double();
        cos_curve_factor_can11_g2_ = this->get_parameter("group_can11_id3_id4.cos_curve_factor").as_double();
        curve_exponent_can11_g2_ = this->get_parameter("group_can11_id3_id4.curve_exponent").as_double();
        max_speed_scale_can11_g2_ = this->get_parameter("group_can11_id3_id4.max_speed_scale").as_int();

        // 각 모터의 각도 제한 파라미터 선언 및 로드
        for (const auto& config : motor_configs_) {
            if (config.name == "can10_motor3") continue;
            this->declare_parameter<double>("limits." + config.name + ".min", -1.0e6);
            this->declare_parameter<double>("limits." + config.name + ".max", 1.0e6);
            motor_states_[config.motor_vector_index].min_angle_limit_deg = this->get_parameter("limits." + config.name + ".min").as_double();
            motor_states_[config.motor_vector_index].max_angle_limit_deg = this->get_parameter("limits." + config.name + ".max").as_double();
        }
    }

    // 각 모터의 각도 피드백 토픽 구독자 생성
    void initialize_subscribers() {
        RCLCPP_INFO(this->get_logger(), "모터 각도 피드백 구독자 생성 중...");
        for (const auto& config : motor_configs_) {
            std::stringstream topic_name_ss;
            topic_name_ss << "/motor_angles/" << (config.can_socket == can_socket_10_ ? "can10" : "can11") << "_motor_0x" << std::hex << config.motor_id;
            std::string topic_name = topic_name_ss.str();
            int index = config.motor_vector_index;
            
            auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
            
            motor_states_[index].angle_subscriber = this->create_subscription<std_msgs::msg::Float64>(
                topic_name, qos_profile,
                [this, index](const std_msgs::msg::Float64::SharedPtr msg) {
                    std::lock_guard<std::mutex> lock(this->motor_states_[index].data_mutex);
                    this->motor_states_[index].current_angle_degrees = msg->data;
                    this->motor_states_[index].has_received_initial_angle = true;
                });
        }
    }

    // IK 제어 명령 토픽 구독자 생성
    void initialize_ik_command_subscriber() {
        auto ik_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
        ik_command_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/ik_joint_commands", ik_qos,
            std::bind(&JoystickToRMDControl::ik_command_callback, this, std::placeholders::_1)
        );
        RCLCPP_INFO(this->get_logger(), "IK 명령어 토픽 구독: /ik_joint_commands");
    }

    /**
     * @brief IK 명령 콜백 함수.
     */
    void ik_command_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (current_mode_ != ControlMode::IK_AUTO) return;
        if (msg->data.size() < 6) {
            RCLCPP_WARN_ONCE(this->get_logger(), "IK 명령 데이터 크기가 6보다 작습니다.");
            return;
        }

        std::lock_guard<std::mutex> lock(command_mutex_);
        for (size_t i = 0; i < 6; ++i) {
            int motor_idx = map_ik_joint_to_motor_index(i);
            if (motor_idx != -1) {
                target_angleControl_motors_[motor_idx] = static_cast<int32_t>(msg->data[i] * motor_gear_ratios_[motor_idx] / 0.01);
                
                if (i == 1) {
                    target_angleControl_motors_[3] = -target_angleControl_motors_[2];
                }
            }
        }
    }

    /**
     * @brief IK 조인트 인덱스를 내부 모터 벡터 인덱스로 변환합니다.
     */
    int map_ik_joint_to_motor_index(int ik_joint_index) {
        switch(ik_joint_index) {
            case 0: return 0; // IK Joint 0 -> Motor Index 0 (can10_motor1)
            case 1: return 2; // IK Joint 1 -> Motor Index 2 (can10_motor2)
            case 2: return 1; // IK Joint 2 -> Motor Index 1 (can10_motor4)
            case 3: return 5; // IK Joint 3 -> Motor Index 5 (can11_motor1)
            case 4: return 4; // IK Joint 4 -> Motor Index 4 (can11_motor2)
            case 5: return 7; // IK Joint 5 -> Motor Index 7 (can11_motor3)
            default: return -1;
        }
    }

    /**
     * @brief 조이스틱 메시지 콜백 함수.
     */
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(joy_msg_mutex_);
        latest_joy_msg_ = msg; 

        if (msg->buttons.size() > 1 && msg->buttons[1] == 1 && prev_mode_button_state_ == 0) {
            if (current_mode_ == ControlMode::MANUAL_JOYSTICK) {
                current_mode_ = ControlMode::IK_AUTO;
                RCLCPP_INFO(this->get_logger(), "----------- 모드 변경: IK_AUTO -----------");
                initialize_target_angles(); 
            } else {
                current_mode_ = ControlMode::MANUAL_JOYSTICK;
                RCLCPP_INFO(this->get_logger(), "----------- 모드 변경: MANUAL_JOYSTICK -----------");
                initialize_target_angles(); 
                joystick_activated_ = false; 
                RCLCPP_INFO(this->get_logger(), "안전을 위해 조이스틱을 움직여 수동 제어를 다시 활성화해주세요.");
            }
        }
        if (msg->buttons.size() > 1) {
            prev_mode_button_state_ = msg->buttons[1];
        }
    }

    /**
     * @brief 메인 제어 루프 타이머 콜백.
     */
    void timer_callback() {
        auto current_time = std::chrono::steady_clock::now();
        double period_sec = std::chrono::duration<double>(current_time - last_callback_start_time_).count();
        
        if (period_sec > 0.1) {
            RCLCPP_WARN(this->get_logger(), "제어 루프가 %.3f초 지연되었습니다. 안전을 위해 기본 주기로 복원합니다.", period_sec);
            period_sec = command_interval_.count() / 1000.0;
        }
        last_callback_start_time_ = current_time;

        if (current_mode_ == ControlMode::MANUAL_JOYSTICK) {
            sensor_msgs::msg::Joy::SharedPtr msg;
            {
                std::lock_guard<std::mutex> lock(joy_msg_mutex_);
                msg = latest_joy_msg_;
            }
            if (msg) {
                process_manual_control(msg, period_sec);
                publish_motor_data();
            }
        } else { // IK_AUTO
            process_ik_control();
            publish_motor_data();
        }
    }

    /**
     * @brief 수동 조이스틱 제어 로직 처리.
     */
    void process_manual_control(const sensor_msgs::msg::Joy::SharedPtr& msg, double period_sec) {
        std::lock_guard<std::mutex> lock(command_mutex_);

        if (!joystick_activated_) {
            bool is_joystick_moved = false;
            for (const auto& axis_val : msg->axes) {
                if (std::abs(axis_val) > 0.1) {
                    is_joystick_moved = true;
                    break;
                }
            }
            if (is_joystick_moved) {
                RCLCPP_INFO(this->get_logger(), "조이스틱 입력 감지. 수동 제어를 활성화합니다.");
                joystick_activated_ = true;
            } else { return; }
        }

        int current_button_state = (msg && msg->buttons.size() > 2) ? msg->buttons[2] : 0;
        if (current_button_state == 1 && prev_speed_button_state_ == 0) {
            speed_level_ = (speed_level_ + 1) % 3;
            RCLCPP_INFO(this->get_logger(), "전체 속도 레벨 변경: %d", speed_level_);
        }
        prev_speed_button_state_ = current_button_state;

        float joy_inputs[8] = {0.0f};
        joy_inputs[0] = get_joy_axis(msg, axis_can10_motor1_, invert_can10_motor1_);
        joy_inputs[1] = get_joy_axis(msg, axis_can10_motor4_, invert_can10_motor4_);
        joy_inputs[2] = get_joy_axis(msg, axis_can10_motor2_, invert_can10_motor2_);
        joy_inputs[4] = get_joy_axis(msg, axis_can11_motor2_, invert_can11_motor2_);
        joy_inputs[5] = get_joy_axis(msg, axis_can11_motor1_, invert_can11_motor1_);
        joy_inputs[7] = get_joy_axis(msg, axis_can11_motor3_, invert_can11_motor3_);

        float l2_trigger = (get_joy_axis(msg, 2) + 1.0f) / 2.0f;
        float r2_trigger = (get_joy_axis(msg, 5) + 1.0f) / 2.0f;
        float combined_trigger = r2_trigger - l2_trigger;
        joy_inputs[6] = combined_trigger;

        for (int i = 0; i < 8; ++i) {
            if (i == 3) continue;

            double current_max_angle_scale;
            if (i <= 3) { // 그룹 1: can10 모터
                current_max_angle_scale = speed_scales_can10_[speed_level_];
            } else if (i == 4 || i == 5) { // 그룹 2: can11 (id1, id2) 모터
                current_max_angle_scale = speed_scales_can11_g1_[speed_level_];
            } else { // 그룹 3: can11 (id3, id4) 모터
                current_max_angle_scale = speed_scales_can11_g2_[speed_level_];
            }
            
            int32_t increment = calculate_angle_increment(i, joy_inputs[i], period_sec, current_max_angle_scale);
            
            internal_target_angles_[i] += increment;

            if (i == 2) {
                internal_target_angles_[3] -= increment;
            }

            uint16_t max_speed = calculate_max_speed(joy_inputs[i], i);
            rmd_position_control(motor_configs_[i].can_socket, motor_configs_[i].motor_id, internal_target_angles_[i], max_speed);
            
            if (i == 2) {
                 rmd_position_control(motor_configs_[3].can_socket, motor_configs_[3].motor_id, internal_target_angles_[3], max_speed);
            }
        }
    }

    /**
     * @brief IK 자동 제어 로직 처리.
     */
    void process_ik_control() {
        std::lock_guard<std::mutex> lock(command_mutex_);
        for (const auto& config : motor_configs_) {
            rmd_position_control(config.can_socket, config.motor_id, target_angleControl_motors_[config.motor_vector_index], 300);
        }
    }

    /**
     * @brief 모터의 현재 각도와 목표 각도를 퍼블리싱합니다.
     */
    void publish_motor_data() {
        if (debug_angles_pub_->get_subscription_count() > 0) {
            auto msg = std::make_unique<std_msgs::msg::Float64MultiArray>();
            msg->data.resize(4);

            // can11_motor1 (인덱스 5)
            std::lock_guard<std::mutex> lock1(motor_states_[5].data_mutex);
            double current_angle1 = motor_states_[5].current_angle_degrees;
            double target_angle1 = internal_target_angles_[5] * 0.01 / motor_gear_ratios_[5];

            // can11_motor2 (인덱스 4)
            std::lock_guard<std::mutex> lock2(motor_states_[4].data_mutex);
            double current_angle2 = motor_states_[4].current_angle_degrees;
            double target_angle2 = internal_target_angles_[4] * 0.01 / motor_gear_ratios_[4];

            msg->data[0] = current_angle1;
            msg->data[1] = target_angle1;
            msg->data[2] = current_angle2;
            msg->data[3] = target_angle2;

            debug_angles_pub_->publish(std::move(msg));
        }
    }

    /**
     * @brief 모터의 목표 각도 증분을 계산합니다.
     */
    int32_t calculate_angle_increment(int motor_index, float joy_input, double period_sec, double max_angle_scale) {
        if (std::abs(joy_input) <= 0.12) return 0;

        double current_cos_accel_scale, current_cos_curve_factor, current_curve_exponent;
        
        if (motor_index <= 3) { // 그룹 1: can10 모터
            current_cos_accel_scale = cos_acceleration_scale_can10_;
            current_cos_curve_factor = cos_curve_factor_can10_;
            current_curve_exponent = curve_exponent_can10_;
        } else if (motor_index == 4 || motor_index == 5) { // 그룹 2: can11 (id1, id2) 모터
            current_cos_accel_scale = cos_acceleration_scale_can11_g1_;
            current_cos_curve_factor = cos_curve_factor_can11_g1_;
            current_curve_exponent = curve_exponent_can11_g1_;
        } else { // 그룹 3: can11 (id3, id4) 모터
            current_cos_accel_scale = cos_acceleration_scale_can11_g2_;
            current_cos_curve_factor = cos_curve_factor_can11_g2_;
            current_curve_exponent = curve_exponent_can11_g2_;
        }

        float dynamic_angle_scale = std::max(min_angle_scale_, (float)max_angle_scale * std::abs(joy_input));
        float normalized_input = std::pow(std::abs(joy_input), current_curve_exponent);
        float cos_value = (1.0 - cos(normalized_input * M_PI * current_cos_curve_factor)) / 2.0;
        double base_increment = joy_input * dynamic_angle_scale * period_sec * cos_value * current_cos_accel_scale;
        double gear_ratio_scale = motor_gear_ratios_[motor_index] / base_gear_ratio_;
        int32_t angle_increment = static_cast<int32_t>(base_increment * gear_ratio_scale);

        std::lock_guard<std::mutex> lock(motor_states_[motor_index].data_mutex);
        if (motor_states_[motor_index].has_received_initial_angle) {
            double min_limit = motor_states_[motor_index].min_angle_limit_deg;
            double max_limit = motor_states_[motor_index].max_angle_limit_deg;
            double current_internal_angle_deg = internal_target_angles_[motor_index] * 0.01 / motor_gear_ratios_[motor_index];
            double next_angle_deg = current_internal_angle_deg + (angle_increment * 0.01 / motor_gear_ratios_[motor_index]);
            
            if ((angle_increment > 0 && next_angle_deg >= max_limit) || (angle_increment < 0 && next_angle_deg <= min_limit)) {
                angle_increment = 0;
            }
        }
        return angle_increment;
    }

    /**
     * @brief 조이스틱 메시지에서 특정 축의 값을 읽어옵니다.
     */
    float get_joy_axis(const sensor_msgs::msg::Joy::SharedPtr& msg, int axis_index, bool inverted = false, float scale = 1.0f) {
        float value = 0.0;
        if (msg && axis_index >= 0 && static_cast<size_t>(axis_index) < msg->axes.size()) {
            value = msg->axes[axis_index];
        }
        value *= scale;
        if (inverted) {
            value *= -1.0;
        }
        return value;
    }
    
    /**
     * @brief 조이스틱 입력 크기에 비례하여 모터의 최대 속도를 계산합니다.
     */
    uint16_t calculate_max_speed(float joystick_input, int motor_index) {
        if (std::abs(joystick_input) <= 0.1) return min_max_speed_;
        
        uint16_t current_max_speed_scale;
        if (motor_index <= 3) { // 그룹 1: can10 모터
            current_max_speed_scale = max_speed_scale_can10_;
        } else if (motor_index == 4 || motor_index == 5) { // 그룹 2: can11 (id1, id2) 모터
            current_max_speed_scale = max_speed_scale_can11_g1_;
        } else { // 그룹 3: can11 (id3, id4) 모터
            current_max_speed_scale = max_speed_scale_can11_g2_;
        }

        float speed_multiplier = 1.0f;
        float calculated_speed_float = std::abs(joystick_input) * current_max_speed_scale * speed_multiplier;
        if (calculated_speed_float > 65535.0f) {
            calculated_speed_float = 65535.0f;
        }
        return static_cast<uint16_t>(calculated_speed_float);
    }

    /**
     * @brief CAN 인터페이스 소켓을 엽니다.
     */
    int open_can_socket(const char *interface_name) {
        int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (s < 0) {
            RCLCPP_ERROR(this->get_logger(), "CAN 소켓 생성 실패 (%s): %s", interface_name, strerror(errno));
            return -1;
        }
        struct ifreq ifr;
        strncpy(ifr.ifr_name, interface_name, IFNAMSIZ - 1);
        ifr.ifr_name[IFNAMSIZ - 1] = '\0';
        if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
            RCLCPP_ERROR(this->get_logger(), "CAN 인터페이스 '%s'를 찾을 수 없습니다. (ioctl 실패: %s)", interface_name, strerror(errno));
            RCLCPP_ERROR(this->get_logger(), "해결 방법: 'ip link show'로 인터페이스 확인, 'sudo ip link set %s up type can bitrate 1000000'로 활성화 확인", interface_name);
            close(s);
            return -1;
        }
        struct sockaddr_can addr;
        memset(&addr, 0, sizeof(addr));
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "CAN 소켓 바인딩 실패 (%s): %s", interface_name, strerror(errno));
            close(s);
            return -1;
        }
        return s;
    }

    /**
     * @brief 노드 시작 시 모든 모터에 0점 이동 명령을 보냅니다.
     */
    void send_startup_raw_frames() {
        std::vector<uint8_t> data = {0xA4, 0x00, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00};
        for(const auto& config : motor_configs_){
            rmd_send_raw_frame(config.can_socket, config.motor_id, data);
        }
    }

    void close_can_sockets() {
        if (can_socket_10_ >= 0) close(can_socket_10_);
        if (can_socket_11_ >= 0) close(can_socket_11_);
    }
    
    bool can_sockets_ok() const {
        return can_socket_10_ >= 0 && can_socket_11_ >= 0;
    }
    
    void wait_for_initial_angles() {
        auto start_wait = this->get_clock()->now();
        while(rclcpp::ok() && (this->get_clock()->now() - start_wait).seconds() < 5.0) {
            bool all_received = true;
            for(size_t i = 0; i < motor_configs_.size(); ++i) {
                std::lock_guard<std::mutex> lock(motor_states_[i].data_mutex);
                if(!motor_states_[i].has_received_initial_angle) { all_received = false; break; }
            }
            if(all_received) { RCLCPP_INFO(this->get_logger(), "모든 초기 모터 각도 수신 완료."); return; }
            rclcpp::spin_some(this->get_node_base_interface());
            std::this_thread::sleep_for(100ms);
        }
        RCLCPP_WARN(this->get_logger(), "초기 모터 각도 대기 시간 초과.");
    }
    
    void initialize_target_angles() {
        std::lock_guard<std::mutex> lock(command_mutex_);
        RCLCPP_INFO(this->get_logger(), "목표 각도를 0으로 초기화 중...");
        for(const auto& config : motor_configs_){
            int idx = config.motor_vector_index;
            target_angleControl_motors_[idx] = 0;
            internal_target_angles_[idx] = 0;
        }
        RCLCPP_INFO(this->get_logger(), "모든 모터의 목표 각도가 0으로 설정되었습니다.");
    }
    
    // --- 멤버 변수들 ---
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    int can_socket_10_ = -1, can_socket_11_ = -1;
    
    std::vector<MotorConfig> motor_configs_;
    std::vector<MotorState> motor_states_;
    std::vector<double> motor_gear_ratios_;

    std::vector<int32_t> target_angleControl_motors_; // IK 모드용
    std::vector<int32_t> internal_target_angles_; // 수동 제어용 누적 목표 위치

    sensor_msgs::msg::Joy::SharedPtr latest_joy_msg_;
    std::mutex joy_msg_mutex_;
    std::mutex command_mutex_;

    std::chrono::steady_clock::time_point last_callback_start_time_;
    const std::chrono::milliseconds command_interval_{10ms};
    const uint16_t min_max_speed_ = 0;
    const float min_angle_scale_ = 0.0f;
    const double base_gear_ratio_ = 36.0;

    // --- 튜닝 그룹 1: CAN10 모터들 ---
    std::vector<double> speed_scales_can10_;
    double cos_acceleration_scale_can10_, cos_curve_factor_can10_, curve_exponent_can10_;
    uint16_t max_speed_scale_can10_;

    // --- 튜닝 그룹 2: CAN11 모터들 (id1, id2) ---
    std::vector<double> speed_scales_can11_g1_;
    double cos_acceleration_scale_can11_g1_, cos_curve_factor_can11_g1_, curve_exponent_can11_g1_;
    uint16_t max_speed_scale_can11_g1_;

    // --- 튜닝 그룹 3: CAN11 모터들 (id3, id4) ---
    std::vector<double> speed_scales_can11_g2_;
    double cos_acceleration_scale_can11_g2_, cos_curve_factor_can11_g2_, curve_exponent_can11_g2_;
    uint16_t max_speed_scale_can11_g2_;

    // --- 조이스틱 축 설정 ---
    int axis_can10_motor1_, axis_can10_motor4_, axis_can10_motor2_;
    int axis_can11_motor1_, axis_can11_motor2_, axis_can11_motor3_;
    bool invert_can10_motor1_, invert_can10_motor4_, invert_can10_motor2_;
    bool invert_can11_motor1_, invert_can11_motor2_, invert_can11_motor3_, invert_can11_motor4_;

    // --- 공통 제어 변수 ---
    int speed_level_ = 0;
    int prev_speed_button_state_ = 0;
    int prev_mode_button_state_ = 0;
    ControlMode current_mode_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr ik_command_sub_;
    
    bool joystick_activated_ = false;

    // 디버깅용 퍼블리셔
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_angles_pub_;
};

/**
 * @brief 현재 프로세스에 실시간 스케줄링(SCHED_FIFO)을 적용합니다.
 */
bool set_realtime_scheduling() {
    struct sched_param sched_params;
    sched_params.sched_priority = 99;
    if (sched_setscheduler(0, SCHED_FIFO, &sched_params) != 0) {
        perror("sched_setscheduler 실패. sudo 권한으로 실행했는지 확인하세요.");
        return false;
    }
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        perror("mlockall 실패");
        return false;
    }
    return true;
}

int main(int argc, char *argv[]) {
    if (!set_realtime_scheduling()) {
        RCLCPP_WARN(rclcpp::get_logger("main"), "실시간 스케줄링 설정 실패. 성능이 저하될 수 있습니다.");
    }
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoystickToRMDControl>());
    rclcpp::shutdown();
    return 0;
}

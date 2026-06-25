#include "rclcpp/rclcpp.hpp"
#include "custom_msgs/msg/target.hpp"
#include "std_msgs/msg/bool.hpp"
#include <iostream>
#include <thread>
#include <string>
#include <sstream>
#include <vector>
#include <map>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <algorithm>
#include <stdexcept>
#include <cctype>
#include <cmath>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

enum class InputMode { SINGLE_LINE, MULTI_LINE };

class GcodeInterpreterNode : public rclcpp::Node
{
public:
    GcodeInterpreterNode() : Node("gcode_interpreter_node"), waiting_for_completion_(false)
    {
        publisher_ = this->create_publisher<custom_msgs::msg::Target>("/trajectory_target", 10);
        
        finished_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/trajectory_finished", 10,
            std::bind(&GcodeInterpreterNode::trajectory_finished_callback, this, std::placeholders::_1));

        is_absolute_mode_ = true;
        current_feed_rate_s_ = 2.0;
        current_pos_.set__x(0.0).set__y(0.0).set__z(0.0);
        current_orient_.set__w(1.0).set__x(0.0).set__y(0.0).set__z(0.0);

        RCLCPP_INFO(this->get_logger(), "G-code Publisher Node started.");
        RCLCPP_INFO(this->get_logger(), "Enter G-code line by line, or use 'BEGIN'/'RUN' for multi-line input.");
        
        input_thread_ = std::thread(&GcodeInterpreterNode::input_loop, this);
    }

    ~GcodeInterpreterNode() {
        { std::lock_guard<std::mutex> lock(mtx_); waiting_for_completion_ = false; }
        cv_.notify_all();
        if (input_thread_.joinable()) {
            input_thread_.join();
        }
    }

private:
    static constexpr int MAX_RETRIES = 3;

    rclcpp::Publisher<custom_msgs::msg::Target>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr finished_sub_;
    std::thread input_thread_;
    std::mutex mtx_;
    std::condition_variable cv_;
    
    // [수정] 콜백에서 사용할 상태 변수 단순화
    bool waiting_for_completion_;
    bool last_command_succeeded_;

    bool is_absolute_mode_;
    double current_feed_rate_s_;
    geometry_msgs::msg::Point current_pos_; 
    geometry_msgs::msg::Quaternion current_orient_;

    void trajectory_finished_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mtx_);
        if (waiting_for_completion_) {
            last_command_succeeded_ = msg->data;
            waiting_for_completion_ = false;
            cv_.notify_one();
        }
    }

    void input_loop() {
        std::string line;
        InputMode mode = InputMode::SINGLE_LINE;
        std::vector<std::string> command_buffer;

        while (rclcpp::ok()) {
            if (mode == InputMode::SINGLE_LINE) std::cout << "G-code > " << std::flush;
            else std::cout << "         > " << std::flush;

            if (!std::getline(std::cin, line)) {
                if (rclcpp::ok()) rclcpp::shutdown();
                break;
            }

            std::string upper_line = line;
            std::transform(upper_line.begin(), upper_line.end(), upper_line.begin(), ::toupper);
            upper_line.erase(upper_line.find_last_not_of(" \n\r\t")+1);

            if (upper_line == "EXIT" || upper_line == "QUIT") {
                if(rclcpp::ok()) rclcpp::shutdown();
                break;
            }

            if (mode == InputMode::SINGLE_LINE) {
                if (upper_line == "BEGIN") {
                    mode = InputMode::MULTI_LINE;
                    command_buffer.clear();
                    std::cout << "--- Multi-line input started. Enter 'RUN' to execute. ---" << std::endl;
                } else {
                    if (!line.empty()) execute_gcode_line(line);
                }
            } else {
                if (upper_line == "RUN") {
                    mode = InputMode::SINGLE_LINE;
                    std::cout << "--- Executing " << command_buffer.size() << " commands... ---" << std::endl;
                    bool success = true;
                    for (const auto& cmd : command_buffer) {
                        if (!rclcpp::ok()) break;
                        if (!execute_gcode_line(cmd)) {
                            RCLCPP_ERROR(this->get_logger(), "Execution failed for command: '%s'. Aborting multi-line run.", cmd.c_str());
                            success = false;
                            break;
                        }
                    }
                    if (success) {
                        std::cout << "--- Multi-line execution finished successfully. ---" << std::endl;
                    } else {
                        std::cout << "--- Multi-line execution aborted due to failure. ---" << std::endl;
                    }
                } else {
                    if (!line.empty()) command_buffer.push_back(line);
                }
            }
        }
    }
    
    // [수정] 재시도 로직을 명확하게 수정
    bool execute_gcode_line(const std::string& line) {
        std::string processed_line = line.substr(0, line.find_first_of(";("));
        std::transform(processed_line.begin(), processed_line.end(), processed_line.begin(), ::toupper);
        std::stringstream ss(processed_line);
        
        std::string word, command_code;
        std::map<char, double> params;
        
        while (ss >> word) {
            if (word.empty()) continue;
            char key = word[0];
            if (command_code.empty() && (key == 'G' || key == 'M')) { command_code = word; }
            else if (isalpha(key) && word.length() > 1) {
                try { params[key] = std::stod(word.substr(1)); }
                catch (const std::exception&) {}
            }
        }
        
        if (command_code.empty()) return true;
        if (params.count('F')) current_feed_rate_s_ = params.at('F'); 

        char command_type = command_code.front();
        int command_num = std::stoi(command_code.substr(1));
        bool is_motion_command = false;
        
        auto msg = std::make_shared<custom_msgs::msg::Target>();
        bool msg_valid = false;

        if (command_type == 'G') {
            switch (command_num) {
                case 0: case 1:
                    *msg = create_linear_move_msg(params, command_num == 0);
                    is_motion_command = true; msg_valid = true; break;
                case 28:
                    *msg = create_go_home_msg();
                    is_motion_command = true; msg_valid = true; break;
                case 90: is_absolute_mode_ = true; RCLCPP_INFO(this->get_logger(), "State set to: Absolute Mode (G90)"); break;
                case 91: is_absolute_mode_ = false; RCLCPP_INFO(this->get_logger(), "State set to: Incremental Mode (G91)"); break;
                case 92: execute_set_position(params); break;
                default: RCLCPP_WARN(this->get_logger(), "Unsupported G-code: G%d", command_num); break;
            }
        }
        
        if (is_motion_command && msg_valid) {
            for (int i = 0; i <= MAX_RETRIES; ++i) {
                if (i > 0) {
                    RCLCPP_WARN(this->get_logger(), "Retrying command... (Attempt %d/%d)", i, MAX_RETRIES);
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                }

                publisher_->publish(*msg);
                if (i == 0) RCLCPP_INFO(this->get_logger(), "Published command for G%d. Waiting for completion...", command_num);

                // 대기 로직 시작
                {
                    std::unique_lock<std::mutex> lock(mtx_);
                    waiting_for_completion_ = true;
                    // cv.wait()는 조건이 충족될 때까지 대기. Spurious wakeup을 방지.
                    cv_.wait(lock, [this]{ return !waiting_for_completion_ || !rclcpp::ok(); });
                }

                if (!rclcpp::ok()) return false; // 노드 종료

                if (last_command_succeeded_) {
                    RCLCPP_INFO(this->get_logger(), "Command executed successfully.");
                    current_pos_ = msg->position;
                    current_orient_ = msg->orientation;
                    return true; // 성공! 루프 탈출
                } else {
                    RCLCPP_WARN(this->get_logger(), "Command failed or was rejected by robot.");
                }
            }

            // 모든 재시도 후에도 실패
            RCLCPP_ERROR(this->get_logger(), "Command '%s' failed after %d retries. Aborting this command.", line.c_str(), MAX_RETRIES);
            return false;
        }

        return true; // 움직임 명령이 아니면 성공
    }
    
    // create_linear_move_msg, create_go_home_msg, execute_set_position 함수는 이전과 동일
    custom_msgs::msg::Target create_linear_move_msg(const std::map<char, double>& params, bool is_rapid) {
        auto msg = custom_msgs::msg::Target();
        if (is_absolute_mode_) {
            msg.position.x = params.count('X') ? params.at('X')/1000.0 : current_pos_.x;
            msg.position.y = params.count('Y') ? params.at('Y')/1000.0 : current_pos_.y;
            msg.position.z = params.count('Z') ? params.at('Z')/1000.0 : current_pos_.z;
        } else {
            msg.position.x = current_pos_.x + (params.count('X') ? params.at('X')/1000.0 : 0.0);
            msg.position.y = current_pos_.y + (params.count('Y') ? params.at('Y')/1000.0 : 0.0);
            msg.position.z = current_pos_.z + (params.count('Z') ? params.at('Z')/1000.0 : 0.0);
        }
        bool has_orientation = params.count('A') || params.count('B') || params.count('C');
        if (has_orientation) {
            msg.mode = 1; tf2::Quaternion q;
            q.setRPY( (params.count('A') ? params.at('A') : 0.0) * M_PI/180.0, 
                      (params.count('B') ? params.at('B') : 0.0) * M_PI/180.0, 
                      (params.count('C') ? params.at('C') : 0.0) * M_PI/180.0 );
            msg.orientation = tf2::toMsg(q);
        } else {
            msg.mode = 0; msg.orientation = current_orient_;
        }
        msg.duration_s = current_feed_rate_s_;
        if (is_rapid) { msg.duration_s = std::max(0.1, msg.duration_s / 2.0); }
        return msg;
    }
    
    custom_msgs::msg::Target create_go_home_msg() {
        auto msg = custom_msgs::msg::Target();
        RCLCPP_INFO(this->get_logger(), "Executing custom home command (G28).");
        msg.mode = 1; msg.position.x = 0.1; msg.position.y = 0.0; msg.position.z = 0.3;
        tf2::Quaternion q; q.setRPY(0.0, 0.0, 0.0);
        msg.orientation = tf2::toMsg(q);
        msg.duration_s = current_feed_rate_s_;
        return msg;
    }

    void execute_set_position(const std::map<char, double>& params) {
        if (params.count('X')) current_pos_.x = params.at('X')/1000.0;
        if (params.count('Y')) current_pos_.y = params.at('Y')/1000.0;
        if (params.count('Z')) current_pos_.z = params.at('Z')/1000.0;
        RCLCPP_INFO(this->get_logger(), "State updated via G92: Current position is now (%.3f, %.3f, %.3f)", current_pos_.x, current_pos_.y, current_pos_.z);
    }
};

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GcodeInterpreterNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <fstream>
#include <string>
#include <chrono>

class PWMController : public rclcpp::Node {
public:
    PWMController() : Node("pwm_controller") {
        // 初始化PWM
        if(!initialize_pwm()) {
            RCLCPP_ERROR(this->get_logger(), "PWM初始化失败");
            throw std::runtime_error("PWM初始化失败");
        }

        // 创建订阅者
        subscription_ = this->create_subscription<std_msgs::msg::Float32>(
            "/set_rpm",
            10,
            [this](const std_msgs::msg::Float32::SharedPtr msg) {
                // 边界限制
                float rpm = std::clamp(msg->data, 0.0f, 100.0f);
                
                // 计算占空比 (5%~10%对应1000000~2000000ns)
                int duty_ns = 1000000 + static_cast<int>((2000000-1000000) * (rpm/100.0));
                
                // 写入sysfs
                if(!write_sysfs("/sys/class/pwm/pwmchip0/pwm0/duty_cycle", std::to_string(duty_ns))) {
                    RCLCPP_ERROR(this->get_logger(), "占空比设置失败");
                }
            });
    }

private:
    bool initialize_pwm() {
        // 设置PWM周期（50Hz = 20ms）
        if(!write_sysfs("/sys/class/pwm/pwmchip0/pwm0/period", "20000000")) return false;
        
        // 启用PWM
        if(!write_sysfs("/sys/class/pwm/pwmchip0/pwm0/enable", "1")) return false;
        
        return true;
    }

    bool write_sysfs(const std::string& path, const std::string& value) {
        std::ofstream file(path);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "无法打开文件: %s", path.c_str());
            return false;
        }
        file << value;
        return true;
    }

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PWMController>());
    rclcpp::shutdown();
    return 0;
}

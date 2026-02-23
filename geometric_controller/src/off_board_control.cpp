#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include "rclcpp/qos.hpp"

class OffboardInitNode : public rclcpp::Node {
public:
    OffboardInitNode() : Node("offboard_init_node") {
        using namespace std::chrono_literals;

        // Publishers
        vehicle_cmd_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);
        offboard_ctrl_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);

        // Timer
        timer_ = this->create_wall_timer(100ms, std::bind(&OffboardInitNode::timer_callback, this));
        
        // 记录节点启动时间
        start_time_ = this->get_clock()->now();
    }

private:
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_cmd_pub_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_ctrl_mode_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time start_time_; // 新增变量

    bool mode_sent_ = false;
    bool armed_ = false;

    void timer_callback() {
        auto now = this->get_clock()->now();
        auto timestamp = now.nanoseconds() / 1000;

        // 1. 持续发送 OffboardControlMode 心跳包 (必须持续发送，否则无法切模式)
        px4_msgs::msg::OffboardControlMode mode_msg{};
        mode_msg.timestamp = timestamp;
        mode_msg.position = false;
        mode_msg.velocity = false;
        mode_msg.acceleration = false;
        mode_msg.attitude = false;
        mode_msg.body_rate = false;  
        mode_msg.thrust_and_torque = true;
        offboard_ctrl_mode_pub_->publish(mode_msg);

        // 2. 增加延时判断：只有启动超过 10秒 后才发送解锁和模式切换指令
        // 这是为了等待 Planner 计算完 A* 并让控制器输出稳定的悬停推力
        if ((now - start_time_).seconds() < 10.0) {
            if((int)(now.seconds()) % 2 == 0) { // 每2秒打印一次等待信息，避免刷屏
                 // RCLCPP_INFO(this->get_logger(), "Waiting for systems to be ready... (%.1fs / 10.0s)", (now - start_time_).seconds());
            }
            return;
        }

        // 3. 发送切换模式指令 (Offboard)
        if (!mode_sent_) {
            px4_msgs::msg::VehicleCommand cmd_msg{};
            cmd_msg.timestamp = timestamp;
            cmd_msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
            cmd_msg.param1 = 1.0;  
            cmd_msg.param2 = 6.0;  
            cmd_msg.target_system = 1;
            cmd_msg.target_component = 1;
            cmd_msg.source_system = 1;
            cmd_msg.source_component = 1;
            cmd_msg.from_external = true;
            vehicle_cmd_pub_->publish(cmd_msg);
            RCLCPP_INFO(this->get_logger(), "Sent OFFBOARD mode command");
            mode_sent_ = true;
        }

        // 4. 发送解锁指令 (Arm)
        if (!armed_) {
            px4_msgs::msg::VehicleCommand arm_cmd{};
            arm_cmd.timestamp = timestamp;
            arm_cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
            arm_cmd.param1 = 1.0;  
            arm_cmd.target_system = 1;
            arm_cmd.target_component = 1;
            arm_cmd.source_system = 1;
            arm_cmd.source_component = 1;
            arm_cmd.from_external = true;
            vehicle_cmd_pub_->publish(arm_cmd);
            RCLCPP_INFO(this->get_logger(), "Sent ARM command");
            armed_ = true;
        }
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardInitNode>());
    rclcpp::shutdown();
    return 0;
}
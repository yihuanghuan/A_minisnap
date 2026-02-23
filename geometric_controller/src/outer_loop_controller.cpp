// outer_loop_controller.cpp

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/accel_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include "geometric_controller/math_utils.hpp"

using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using std::placeholders::_1;
using namespace std::chrono_literals;

class OuterLoopController : public rclcpp::Node {
public:
    OuterLoopController()
        : Node("outer_loop_controller"), i_time_(0){    
        RCLCPP_INFO(this->get_logger(), "Node started");     
        // Gains
        Kp_ << 8, 0, 0,
               0, 8, 0,
               0, 0, 8;

        Kv_ << 5, 0, 0,
               0, 5, 0,
               0, 0, 5;

        g_ = 9.81;
        e3_ << 0, 0, 1;
        zb_des_ << 0, 0, 1;

        mass = 2.5;

        rclcpp::QoS qos_profile = rclcpp::SensorDataQoS();

        // Subscribers
        odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", qos_profile, std::bind(&OuterLoopController::odom_callback, this, _1));
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/target_pose", 10, std::bind(&OuterLoopController::pose_callback, this, _1));
        twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/target_twist", 10, std::bind(&OuterLoopController::twist_callback, this, _1));
        accel_sub_ = this->create_subscription<geometry_msgs::msg::AccelStamped>(
            "/target_accel", 10, std::bind(&OuterLoopController::accel_callback, this, _1));

        // Publishers
        zb_des_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/geom_control/zb_des", 10);
        psi_pub_ = this->create_publisher<std_msgs::msg::Float64>("/geom_control/psi_des", 10);
        uT_pub_ = this->create_publisher<std_msgs::msg::Float64>("/geom_control/u_T", 10);
        ref_pos_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/reference_position", 10);

        // Timer for control loop
        timer_ = this->create_wall_timer(1ms, std::bind(&OuterLoopController::control_loop, this));
    }

private:
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr zb_des_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr psi_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr uT_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr ref_pos_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
    rclcpp::Subscription<geometry_msgs::msg::AccelStamped>::SharedPtr accel_sub_;

    Matrix3d Kp_, Kv_, Rb_;
    Vector3d zb_des_, e3_, ang_vel_, err_pos, err_vel, acc_cmd;
    Vector3d pos_, vel_;
    
    geometry_msgs::msg::Vector3 zb_msg;
    std_msgs::msg::Float64 psi_msg, uT_msg;
    geometry_msgs::msg::Vector3 ref_pos_msg;

    float u_T_;
    float g_;
    int i_time_;
    float mass;

    // References
    Vector3d q, dot_q, ddot_q;
    float psi;
    bool refs_ready_ = false;

    void odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
        // Odom is already in NED (from PX4)
        pos_ << msg->position[0], msg->position[1], msg->position[2];
        vel_ << msg->velocity[0], msg->velocity[1], msg->velocity[2];
        Eigen::Quaterniond quat(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
        Rb_ = quat.toRotationMatrix();
    }   

    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        // 关键修改：坐标系转换 ENU -> NED
        // X -> X, Y -> -Y, Z -> -Z
        q << msg->pose.position.x, -msg->pose.position.y, -msg->pose.position.z;
        
        Eigen::Quaterniond quat(
            msg->pose.orientation.w,
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z);
        // ENU RPY
        Eigen::Vector3d rpy = quat.toRotationMatrix().eulerAngles(0, 1, 2);
        // Yaw Conversion: ENU (+Left) -> NED (+Right) => psi_ned = -psi_enu
        psi = -rpy[2]; 
        
        refs_ready_ = true;
    }

    void twist_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        // 关键修改：速度转换 ENU -> NED
        dot_q << msg->twist.linear.x, -msg->twist.linear.y, -msg->twist.linear.z;
    }

    void accel_callback(const geometry_msgs::msg::AccelStamped::SharedPtr msg) {
        // 关键修改：加速度转换 ENU -> NED
        ddot_q << msg->accel.linear.x, -msg->accel.linear.y, -msg->accel.linear.z;
    }

    void control_loop() {
        
        if (!refs_ready_) {
            static int wait_log_cnt = 0;
            if (wait_log_cnt++ % 1000 == 0) {
                RCLCPP_WARN(this->get_logger(), "[Outer] Waiting for Planner Reference... (Pose/Twist/Accel)");
            }
            return;
        }

        ref_pos_msg.x = q.x();
        ref_pos_msg.y = q.y();
        ref_pos_msg.z = q.z();

        err_pos = pos_ - q;
        err_vel = vel_ - dot_q;

        acc_cmd = -Kp_ * err_pos - Kv_ * err_vel - mass * g_ * e3_ + mass * ddot_q;
       
        u_T_ = -(acc_cmd.transpose() * Rb_ * e3_).value();
        zb_des_ = -acc_cmd / acc_cmd.norm();

        zb_msg.x = zb_des_.x(); zb_msg.y = zb_des_.y(); zb_msg.z = zb_des_.z();

        psi_msg.data = psi;
        uT_msg.data = u_T_;

        zb_des_pub_->publish(zb_msg);
        psi_pub_->publish(psi_msg);
        uT_pub_->publish(uT_msg);
        ref_pos_pub_->publish(ref_pos_msg);

        i_time_++;

        // Debug Log
        static int log_cnt = 0;
        if (log_cnt++ % 500 == 0) { // 0.5s
            RCLCPP_INFO(this->get_logger(), 
                "[Outer] Pos(NED): [%.2f, %.2f, %.2f] | Ref(NED): [%.2f, %.2f, %.2f] | Thrust: %.2f",
                pos_.x(), pos_.y(), pos_.z(),
                q.x(), q.y(), q.z(),
                u_T_);
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto controller = std::make_shared<OuterLoopController>();
    rclcpp::spin(controller);
    rclcpp::shutdown();
    return 0;
}
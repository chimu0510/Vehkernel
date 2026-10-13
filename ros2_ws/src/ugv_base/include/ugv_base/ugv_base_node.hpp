#pragma once

#include <string>
#include <memory>
#include <atomic>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "ugv_protocol.hpp"
#include "ugv_base/serial_transport.hpp"
#include "ugv_base/clock_sync.hpp"

namespace ugv::base {

class UgvBaseNode : public rclcpp::Node {
public:
    explicit UgvBaseNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~UgvBaseNode() override;

private:
    void init_parameters();
    void setup_communication();
    void send_handshake();
    void send_heartbeat();

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void read_serial_stream();

    void handle_packet(protocol::MessageId id, const uint8_t* payload, size_t size);
    void handle_handshake_resp(const protocol::HandshakeResp& resp);
    void handle_heartbeat_ack(const protocol::HeartbeatAck& ack);
    void handle_telemetry(const protocol::Telemetry& telem);

    void update_odometry(double vx, double wz, const rclcpp::Time& stamp);

    // Parameters
    std::string port_{"/dev/ttyACM0"};
    int baud_{115200};
    std::string heading_param_{"drive_front"};
    bool publish_tf_{true};
    std::string odom_frame_{"odom"};
    std::string base_frame_{"base_link"};
    double linear_scale_{1.0};
    double angular_scale_{1.0};

    protocol::HeadingMode expected_heading_{protocol::HeadingMode::DRIVE_FRONT};

    // Transport & Protocol
    SerialTransport transport_;
    protocol::PacketParser parser_;
    ClockSync clock_sync_;

    // Connection states
    std::atomic<bool> is_connected_{false};
    std::atomic<bool> heading_matched_{false};
    uint32_t host_seq_{0};

    // Odometry state
    double odom_x_{0.0};
    double odom_y_{0.0};
    double odom_yaw_{0.0};
    rclcpp::Time last_telem_stamp_{0, 0, RCL_ROS_TIME};

    // ROS interfaces
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::TimerBase::SharedPtr poll_timer_;
    rclcpp::TimerBase::SharedPtr heartbeat_timer_;
};

} // namespace ugv::base

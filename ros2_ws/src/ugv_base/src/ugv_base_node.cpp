#include "ugv_base/ugv_base_node.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace ugv::base {

UgvBaseNode::UgvBaseNode(const rclcpp::NodeOptions& options)
    : Node("ugv_base_node", options) {
    init_parameters();
    setup_communication();

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", rclcpp::QoS(10),
        std::bind(&UgvBaseNode::cmd_vel_callback, this, std::placeholders::_1));

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", rclcpp::QoS(10));
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu/data_raw", rclcpp::QoS(10));
    battery_pub_ = create_publisher<sensor_msgs::msg::BatteryState>("/battery_state", rclcpp::QoS(10));

    // 100 Hz Serial polling timer
    poll_timer_ = create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&UgvBaseNode::read_serial_stream, this));

    // 10 Hz Heartbeat timer
    heartbeat_timer_ = create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&UgvBaseNode::send_heartbeat, this));

    send_handshake();
}

UgvBaseNode::~UgvBaseNode() {
    transport_.close();
}

void UgvBaseNode::init_parameters() {
    port_ = declare_parameter<std::string>("port", "/dev/ttyACM0");
    baud_ = declare_parameter<int>("baud", 115200);
    heading_param_ = declare_parameter<std::string>("heading", "drive_front");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    linear_scale_ = declare_parameter<double>("linear_scale", 1.0);
    angular_scale_ = declare_parameter<double>("angular_scale", 1.0);

    if (heading_param_ == "omni_front") {
        expected_heading_ = protocol::HeadingMode::OMNI_FRONT;
        RCLCPP_INFO(get_logger(), "Configured Heading: OMNI_FRONT (ADR-0001)");
    } else {
        expected_heading_ = protocol::HeadingMode::DRIVE_FRONT;
        RCLCPP_INFO(get_logger(), "Configured Heading: DRIVE_FRONT (ADR-0001)");
    }
}

void UgvBaseNode::setup_communication() {
    RCLCPP_INFO(get_logger(), "Opening serial port %s at %d baud...", port_.c_str(), baud_);
    if (!transport_.open(port_, baud_)) {
        RCLCPP_WARN(get_logger(), "Could not open serial port %s! Will retry on loop.", port_.c_str());
    } else {
        RCLCPP_INFO(get_logger(), "Serial port opened successfully.");
    }
}

void UgvBaseNode::send_handshake() {
    if (!transport_.is_open()) {
        return;
    }
    protocol::HandshakeReq req{};
    req.header.seq = host_seq_++;
    req.header.timestamp_us = 0;
    req.host_proto_version = protocol::PROTOCOL_VERSION;
    req.expected_heading = static_cast<uint8_t>(expected_heading_);

    auto frame = protocol::encode(req);
    transport_.write(frame.data(), frame.size());
    RCLCPP_INFO(get_logger(), "Sent HandshakeReq to C50C (proto_ver=%u, heading=%u)",
                req.host_proto_version, req.expected_heading);
}

void UgvBaseNode::send_heartbeat() {
    if (!transport_.is_open()) {
        if (transport_.open(port_, baud_)) {
            send_handshake();
        }
        return;
    }

    if (!is_connected_) {
        send_handshake();
        return;
    }

    protocol::Heartbeat hb{};
    hb.header.seq = host_seq_++;
    hb.header.timestamp_us = 0;
    hb.host_state = 0; // OK
    hb.req_autonomous = (heading_matched_ ? 1 : 0);

    auto frame = protocol::encode(hb);
    transport_.write(frame.data(), frame.size());
}

void UgvBaseNode::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    if (!is_connected_ || !heading_matched_) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Cannot execute cmd_vel: C50C not connected or heading mismatched (ADR-0001)!");
        return;
    }

    protocol::SetVelocity cmd{};
    cmd.header.seq = host_seq_++;
    cmd.header.timestamp_us = 0;
    cmd.target_vx = static_cast<float>(msg->linear.x * linear_scale_);
    cmd.target_wz = static_cast<float>(msg->angular.z * angular_scale_);

    auto frame = protocol::encode(cmd);
    transport_.write(frame.data(), frame.size());
}

void UgvBaseNode::read_serial_stream() {
    if (!transport_.is_open()) {
        return;
    }

    uint8_t buf[256];
    ssize_t bytes_read = transport_.read(buf, sizeof(buf), 0);
    if (bytes_read > 0) {
        parser_.parse_chunk(buf, bytes_read, [this](protocol::MessageId id, const uint8_t* payload, size_t size) {
            handle_packet(id, payload, size);
        });
    }
}

void UgvBaseNode::handle_packet(protocol::MessageId id, const uint8_t* payload, size_t size) {
    switch (id) {
    case protocol::MessageId::HandshakeResp:
        if (size >= sizeof(protocol::HandshakeResp)) {
            handle_handshake_resp(*reinterpret_cast<const protocol::HandshakeResp*>(payload));
        }
        break;

    case protocol::MessageId::HeartbeatAck:
        if (size >= sizeof(protocol::HeartbeatAck)) {
            handle_heartbeat_ack(*reinterpret_cast<const protocol::HeartbeatAck*>(payload));
        }
        break;

    case protocol::MessageId::Telemetry:
        if (size >= sizeof(protocol::Telemetry)) {
            handle_telemetry(*reinterpret_cast<const protocol::Telemetry*>(payload));
        }
        break;

    default:
        break;
    }
}

void UgvBaseNode::handle_handshake_resp(const protocol::HandshakeResp& resp) {
    RCLCPP_INFO(get_logger(), "Received HandshakeResp from MCU: proto_ver=%u, heading=%u, matched=%u, hw_ver=0x%02X",
                resp.mcu_proto_version, resp.mcu_heading, resp.heading_matched, resp.hw_version);

    if (resp.heading_matched == 1) {
        is_connected_ = true;
        heading_matched_ = true;
        RCLCPP_INFO(get_logger(), "Handshake successful! System verified and ready for autonomous navigation.");
    } else {
        is_connected_ = true;
        heading_matched_ = false;
        RCLCPP_FATAL(get_logger(),
                     "CRITICAL: Heading mismatch! Host expects %u, but C50C Flash is %u. "
                     "Autonomous mode BLOCKED (ADR-0001). Please update Flash setting via OLED or parameter!",
                     static_cast<uint8_t>(expected_heading_), resp.mcu_heading);
    }
}

void UgvBaseNode::handle_heartbeat_ack(const protocol::HeartbeatAck& ack) {
    // Heartbeat received
    (void)ack;
}

void UgvBaseNode::handle_telemetry(const protocol::Telemetry& telem) {
    uint64_t host_now_ns = now().nanoseconds();
    clock_sync_.update(host_now_ns, telem.header.timestamp_us);

    rclcpp::Time stamp = clock_sync_.to_ros_time(telem.header.timestamp_us);

    // 1. Odometry update
    update_odometry(telem.measured_vx, telem.measured_wz, stamp);

    // 2. Publish IMU
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp = stamp;
    imu_msg.header.frame_id = "imu_link";

    imu_msg.linear_acceleration.x = telem.imu_accel_x;
    imu_msg.linear_acceleration.y = telem.imu_accel_y;
    imu_msg.linear_acceleration.z = telem.imu_accel_z;

    imu_msg.angular_velocity.x = telem.imu_gyro_x;
    imu_msg.angular_velocity.y = telem.imu_gyro_y;
    imu_msg.angular_velocity.z = telem.imu_gyro_z;

    // Covariances
    imu_msg.linear_acceleration_covariance[0] = 0.04;
    imu_msg.linear_acceleration_covariance[4] = 0.04;
    imu_msg.linear_acceleration_covariance[8] = 0.04;

    imu_msg.angular_velocity_covariance[0] = 0.002;
    imu_msg.angular_velocity_covariance[4] = 0.002;
    imu_msg.angular_velocity_covariance[8] = 0.002;

    // Orientation covariance -1 indicates orientation not provided in data_raw
    imu_msg.orientation_covariance[0] = -1.0;

    imu_pub_->publish(imu_msg);

    // 3. Publish Battery State
    sensor_msgs::msg::BatteryState bat_msg;
    bat_msg.header.stamp = stamp;
    bat_msg.voltage = telem.battery_voltage;
    bat_msg.present = (telem.battery_level != static_cast<uint8_t>(protocol::BatteryLevel::NO_BATTERY));

    if (telem.battery_level == static_cast<uint8_t>(protocol::BatteryLevel::CRITICAL)) {
        bat_msg.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_DEAD;
    } else if (telem.battery_level == static_cast<uint8_t>(protocol::BatteryLevel::LOW)) {
        bat_msg.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
    } else {
        bat_msg.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
    }

    battery_pub_->publish(bat_msg);
}

void UgvBaseNode::update_odometry(double vx, double wz, const rclcpp::Time& stamp) {
    if (last_telem_stamp_.nanoseconds() == 0) {
        last_telem_stamp_ = stamp;
        return;
    }

    double dt = (stamp - last_telem_stamp_).seconds();
    last_telem_stamp_ = stamp;

    if (dt <= 0.0 || dt > 0.5) {
        return;
    }

    // Runge-Kutta 2nd order integration
    double delta_theta = wz * dt;
    double delta_x = vx * std::cos(odom_yaw_ + delta_theta * 0.5) * dt;
    double delta_y = vx * std::sin(odom_yaw_ + delta_theta * 0.5) * dt;

    odom_x_ += delta_x;
    odom_y_ += delta_y;
    odom_yaw_ += delta_theta;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, odom_yaw_);

    // Publish Odometry message
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;

    odom.pose.pose.position.x = odom_x_;
    odom.pose.pose.position.y = odom_y_;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = wz;

    // Covariances
    odom.pose.covariance[0] = 0.001;
    odom.pose.covariance[7] = 0.001;
    odom.pose.covariance[35] = 0.005;

    odom.twist.covariance[0] = 0.001;
    odom.twist.covariance[7] = 0.001;
    odom.twist.covariance[35] = 0.005;

    odom_pub_->publish(odom);

    // Broadcast TF if enabled
    if (publish_tf_) {
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = stamp;
        tf_msg.header.frame_id = odom_frame_;
        tf_msg.child_frame_id = base_frame_;

        tf_msg.transform.translation.x = odom_x_;
        tf_msg.transform.translation.y = odom_y_;
        tf_msg.transform.translation.z = 0.0;
        tf_msg.transform.rotation.x = q.x();
        tf_msg.transform.rotation.y = q.y();
        tf_msg.transform.rotation.z = q.z();
        tf_msg.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(tf_msg);
    }
}

} // namespace ugv::base

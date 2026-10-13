#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "ugv_base/ugv_base_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ugv::base::UgvBaseNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

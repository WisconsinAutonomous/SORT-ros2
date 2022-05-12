#include "SORT-ros2/SortRos.h"

#include <memory>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<vision_tracker::SortRos>();

    RCLCPP_INFO(node->get_logger(), "sort_ros started up!");
    // actually run the node
    rclcpp::spin(node);  // should not return
    rclcpp::shutdown();
    return 0;
}

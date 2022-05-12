#ifndef SORT_ROS_H
#define SORT_ROS_H

#include "Sort.h"

#include "rclcpp/rclcpp.hpp"
#include "wauto_perception_msgs/msg/roi_array.hpp"

#include <memory>

namespace vision_tracker {

class SortRos : public rclcpp::Node {
    // --------
    // Typedefs
    // --------

    typedef wauto_perception_msgs::msg::RoiArray RoiArrayMsg;

    typedef rclcpp::Subscription<RoiArrayMsg> RoiArraySubscriber;
    typedef std::shared_ptr<RoiArraySubscriber> RoiArraySubscriberPtr;

    typedef rclcpp::Publisher<RoiArrayMsg> RoiArrayPublisher;
    typedef std::shared_ptr<RoiArrayPublisher> RoiArrayPublisherPtr;

  public:
    SortRos(const rclcpp::NodeOptions& options = {});
    ~SortRos();

    // -------------
    // ROS Functions
    // -------------

    void callback(const RoiArrayMsg& msg);

  private:
    // ----------------
    // Helper Functions
    // ----------------

    // ---------------
    // Class Variables
    // ---------------

    RoiArraySubscriberPtr m_rois_subscriber;
    RoiArrayPublisherPtr m_rois_publisher;

    std::shared_ptr<Sort> m_sort;
};

}  // namespace vision_tracker

#endif

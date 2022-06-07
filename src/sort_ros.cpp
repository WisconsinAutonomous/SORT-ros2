#include "SORT-ros2/SortRos.h"

namespace vision_tracker {

SortRos::SortRos(const rclcpp::NodeOptions options) : rclcpp::Node("sort_ros", options) {
    double maxAge = this->declare_parameter<double>("max_age", 2.0);
    double minHits = this->declare_parameter<double>("min_hits", 3.0);
    double iouThreshold = this->declare_parameter<double>("iou_threshold", 0.3);

    m_sort = std::make_shared<Sort>(maxAge, minHits, iouThreshold);

    using std::placeholders::_1;
    m_rois_subscriber =
        this->create_subscription<RoiArrayMsg>("~/input/rois", 1, std::bind(&SortRos::callback, this, _1));
    m_rois_publisher = this->create_publisher<RoiArrayMsg>("~/output/rois", 1);
}

SortRos::~SortRos() {}

void SortRos::callback(const RoiArrayMsg& rois) {
    // ------
    // Update
    // ------
    std::vector<SortRect> rects;
    rects.reserve(rois.rois.size());
    for (auto& roi : rois.rois) {
        SortRect rect;
        rect.id = 0;
        rect.centerX = (roi.top_right.x + roi.bottom_left.x) / 2;
        rect.centerY = (roi.top_right.y + roi.bottom_left.y) / 2;
        rect.width = (roi.top_right.x - roi.bottom_left.x);
        rect.height = (roi.bottom_left.y - roi.top_right.y);

        rects.push_back(rect);
    }
    rects = m_sort->update(rects);

    // -------
    // Publish
    // -------
    // Overwrite the previous message with new uuids

    auto tracked_rois = std::make_unique<RoiArrayMsg>(rois);
    std::size_t idx = 0;
	for (auto iter = tracked_rois->rois.begin(); iter != tracked_rois->rois.end(); ) {
        if (rects.size() > idx) {
            // Okay
            // For readability
            auto& rect = rects[idx];
            auto& roi = tracked_rois->rois[idx];

            roi.id = rect.id % 63 + 1;

            ++iter;
            ++idx;
        }
        else {
            // Don't have a valid ID yet
            // Ignore
            iter = tracked_rois->rois.erase(iter);
        }
	}
    m_rois_publisher->publish(std::move(tracked_rois));
}

}  // namespace vision_tracker

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(vision_tracker::SortRos)

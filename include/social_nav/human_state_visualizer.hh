#pragma once
#include <human_state_msgs/msg/human_state_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace SocialNav {
class HumanStateVisualizer : public rclcpp::Node {
  public:
    HumanStateVisualizer();

  private:
    void cb_(human_state_msgs::msg::HumanStateArray::ConstSharedPtr msg);

    rclcpp::Subscription<human_state_msgs::msg::HumanStateArray>::SharedPtr
        sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;
};
} // namespace SocialNav

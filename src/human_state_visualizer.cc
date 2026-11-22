#include <social_nav/human_state_visualizer.hh>

namespace SocialNav {
HumanStateVisualizer::HumanStateVisualizer() : Node("human_state_visualizer") {
    auto in_topic =
        declare_parameter<std::string>("input_topic", "/human_detector/poses");
    auto out_topic = declare_parameter<std::string>("output_topic",
                                                    "/human_detector/markers");

    pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        out_topic, rclcpp::QoS(10));
    sub_ = create_subscription<human_state_msgs::msg::HumanStateArray>(
        in_topic, rclcpp::QoS(10),
        [this](human_state_msgs::msg::HumanStateArray::ConstSharedPtr m) {
            cb_(m);
        });
}

void HumanStateVisualizer::cb_(
    human_state_msgs::msg::HumanStateArray::ConstSharedPtr msg) {
    visualization_msgs::msg::MarkerArray arr;

    visualization_msgs::msg::Marker clear;
    clear.header.frame_id = msg->header.frame_id;
    clear.header.stamp = rclcpp::Time(0);
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(clear);

    int id = 0;
    for (const auto &h : msg->humans) {
        visualization_msgs::msg::Marker body;
        body.header.frame_id = msg->header.frame_id;
        body.header.stamp = rclcpp::Time(0);
        body.ns = "human_body";
        body.id = id;
        body.type = visualization_msgs::msg::Marker::CYLINDER;
        body.action = visualization_msgs::msg::Marker::ADD;
        body.pose = h.pose;
        body.pose.position.z = 0.85;
        body.scale.x = 0.45;
        body.scale.y = 0.45;
        body.scale.z = 1.70;
        body.color.g = 1.0f;
        body.color.a = 0.8f;
        body.lifetime = rclcpp::Duration::from_seconds(0.5);
        arr.markers.push_back(body);

        visualization_msgs::msg::Marker label;
        label.header.frame_id = msg->header.frame_id;
        label.header.stamp = rclcpp::Time(0);
        label.ns = "human_label";
        label.id = id;
        label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        label.action = visualization_msgs::msg::Marker::ADD;
        label.pose = h.pose;
        label.pose.position.z = 1.95;
        label.scale.z = 0.25;
        label.color.r = label.color.g = label.color.b = label.color.a = 1.0f;
        label.text = "H" + std::to_string(id);
        label.lifetime = rclcpp::Duration::from_seconds(0.5);
        arr.markers.push_back(label);

        visualization_msgs::msg::Marker arrow;
        arrow.header.frame_id = msg->header.frame_id;
        arrow.header.stamp = rclcpp::Time(0);
        arrow.ns = "human_heading";
        arrow.id = id;
        arrow.type = visualization_msgs::msg::Marker::ARROW;
        arrow.action = visualization_msgs::msg::Marker::ADD;
        arrow.pose = h.pose;
        arrow.pose.position.z = 1.0;
        arrow.scale.x = 0.7;
        arrow.scale.y = 0.1;
        arrow.scale.z = 0.1;
        arrow.color.r = 1.0f;
        arrow.color.g = 0.3f;
        arrow.color.a = 1.0f;
        arrow.lifetime = rclcpp::Duration::from_seconds(0.5);
        arr.markers.push_back(arrow);
        ++id;
    }
    pub_->publish(arr);
}
} // namespace SocialNav

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SocialNav::HumanStateVisualizer>());
    rclcpp::shutdown();
    return 0;
}

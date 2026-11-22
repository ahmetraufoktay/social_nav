#pragma once

#include <algorithm>
#include <cmath>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <image_geometry/pinhole_camera_model.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <cv_bridge/cv_bridge.hpp>

#include <human_state_msgs/msg/human_state_array.hpp>
#include <hungarian_algorithm.hh>

using namespace std::chrono_literals;

namespace SocialNav {

struct Track {
    double px, py;
    double vx = 0.0, vy = 0.0;
    double P00, P01, P11;
    double yaw = 0.0;
    bool has_heading = false;
    int missed = 0;
    rclcpp::Time stamp;
};

class HumanDetector : public ::rclcpp::Node {
  public:
    HumanDetector();

  private:
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr
        intrinsics_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_img_sub_;
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr
        yolo_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr spaam_sub_;

    rclcpp::Publisher<human_state_msgs::msg::HumanStateArray>::SharedPtr
        human_poses_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::vector<Track> tracks_;
    std::string fixed_frame_;
    double v_min_, kf_q_, kf_r_;
    int max_missed_;

    sensor_msgs::msg::Image::ConstSharedPtr depth_img_;
    image_geometry::PinholeCameraModel cam_model_;
    vision_msgs::msg::Detection2DArray::ConstSharedPtr yolo_array_;
    geometry_msgs::msg::PoseArray::ConstSharedPtr spaam_array_;

    bool camera_model_initialized_{false};
    bool yolo_received_{false};
    bool depth_received_{false};
    bool spaam_received_{false};

    void intrinsics_cb_(sensor_msgs::msg::CameraInfo::ConstSharedPtr int_msg);
    void depth_img_cb_(sensor_msgs::msg::Image::ConstSharedPtr depth_img);
    void
    yolo_cb_(vision_msgs::msg::Detection2DArray::ConstSharedPtr yolo_array);
    void spaam_cb_(geometry_msgs::msg::PoseArray::ConstSharedPtr spaam_array);

    void timer_cb();

    human_state_msgs::msg::HumanStateArray
    match_sensors_(std::vector<geometry_msgs::msg::Pose> &yolo_in_base,
                   std::vector<geometry_msgs::msg::Pose> &spaam_in_base);

    bool transform_pose_(const geometry_msgs::msg::Pose &in,
                         const std::string &src, const std::string &dst,
                         geometry_msgs::msg::Pose &out);
    float
    process_depth(const sensor_msgs::msg::Image::ConstSharedPtr &depth_msg,
                  int u, int v, int window = 5);
};
} // namespace SocialNav

#include <geometry_msgs/msg/pose_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

class HumanDetector : public rclcpp::Node {
  public:
    HumanDetector() : Node("human_detector") {
        intrinsics_sub =
            this->create_subscription<sensor_msgs::msg::CameraInfo>(
                "/oakd/rgb/preview/camera_info", rclcpp::QoS(10),
                [&](sensor_msgs::msg::CameraInfo::ConstSharedPtr int_msg) {
                    intrinsics_cb(int_msg);
                });

        pc2_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/oakd/rgb/preview/depth/points", rclcpp::QoS(10),
            [&](sensor_msgs::msg::PointCloud2::ConstSharedPtr pc2_msg) {
                pc2_cb(pc2_msg);
            });

        yolo_sub =
            this->create_subscription<vision_msgs::msg::Detection2DArray>(
                "/human_detections", rclcpp::QoS(10),
                [&](vision_msgs::msg::Detection2DArray::ConstSharedPtr
                        yolo_array) { yolo_cb(yolo_array); });

        spaam_sub = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/dr_spaam_detections", rclcpp::QoS(10),
            [&](geometry_msgs::msg::PoseArray::ConstSharedPtr spaam_array) {
                this->spaam_cb(spaam_array);
            });
    }

  private:
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr
        intrinsics_sub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc2_sub;
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr
        yolo_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr spaam_sub;

    sensor_msgs::msg::CameraInfo::ConstSharedPtr intrinsics_msg;
    sensor_msgs::msg::PointCloud2::ConstSharedPtr pc2_msg;
    vision_msgs::msg::Detection2DArray::ConstSharedPtr yolo_array;
    geometry_msgs::msg::PoseArray::ConstSharedPtr spaam_array;

    void intrinsics_cb(sensor_msgs::msg::CameraInfo::ConstSharedPtr int_msg) {
        this->intrinsics_msg = int_msg;
    }

    void pc2_cb(sensor_msgs::msg::PointCloud2::ConstSharedPtr pc2_msg) {
        this->pc2_msg = pc2_msg;
    }

    void
    yolo_cb(vision_msgs::msg::Detection2DArray::ConstSharedPtr yolo_array) {
        this->yolo_array = yolo_array;
    }

    void spaam_cb(geometry_msgs::msg::PoseArray::ConstSharedPtr spaam_array) {
        this->spaam_array = spaam_array;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HumanDetector>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

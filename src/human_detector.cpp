#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <image_geometry/pinhole_camera_model.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <cv_bridge/cv_bridge.hpp>

#include <hungarian_algorithm.hpp>

using namespace std::chrono_literals;

struct Track {
    int id;
    double x, y;
    double vx{0.0}, vy{0.0};
    double yaw{0.0};
    bool has_heading{false};
    int misses{0};
};

class HumanDetector : public rclcpp::Node {
  public:
    HumanDetector() : Node("human_detector") {
        fixed_frame_ =
            this->declare_parameter<std::string>("fixed_frame", "odom");
        track_gate_ = this->declare_parameter<double>("track_gate", 0.8);
        alpha_ = this->declare_parameter<double>("alpha", 0.5);
        beta_ = this->declare_parameter<double>("beta", 0.3);
        v_min_ = this->declare_parameter<double>("v_min", 0.25);
        yaw_lp_ = this->declare_parameter<double>("yaw_lowpass", 0.3);
        max_misses_ = this->declare_parameter<int>("max_misses", 5);

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ =
            std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        intrinsics_sub_ =
            this->create_subscription<sensor_msgs::msg::CameraInfo>(
                "/oakd/rgb/preview/camera_info", rclcpp::QoS(10),
                [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr int_msg) {
                    intrinsics_cb_(int_msg);
                });

        depth_img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/oakd/rgb/preview/depth", rclcpp::QoS(10),
            [this](sensor_msgs::msg::Image::ConstSharedPtr depth_img) {
                depth_img_cb_(depth_img);
            });

        yolo_sub_ =
            this->create_subscription<vision_msgs::msg::Detection2DArray>(
                "/yolo_detections", rclcpp::QoS(10),
                [this](vision_msgs::msg::Detection2DArray::ConstSharedPtr
                           yolo_array) { yolo_cb_(yolo_array); });

        spaam_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/dr_spaam_detections", rclcpp::QoS(10),
            [this](geometry_msgs::msg::PoseArray::ConstSharedPtr spaam_array) {
                this->spaam_cb_(spaam_array);
            });

        fused_poses_pub_ =
            this->create_publisher<geometry_msgs::msg::PoseArray>(
                "/human_detector/poses", rclcpp::QoS(10));

        marker_pub_ =
            this->create_publisher<visualization_msgs::msg::MarkerArray>(
                "/human_detector/markers", rclcpp::QoS(10));

        timer_ = this->create_wall_timer(100ms, [this]() { this->timer_cb(); });
    }

  private:
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr
        intrinsics_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_img_sub_;
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr
        yolo_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr spaam_sub_;

    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr
        fused_poses_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
        marker_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    sensor_msgs::msg::Image::ConstSharedPtr depth_img_;
    image_geometry::PinholeCameraModel cam_model_;
    vision_msgs::msg::Detection2DArray::ConstSharedPtr yolo_array_;
    geometry_msgs::msg::PoseArray::ConstSharedPtr spaam_array_;

    bool camera_model_initialized_{false};
    bool yolo_received_{false};
    bool depth_received_{false};
    bool spaam_received_{false};

    std::vector<Track> tracks_;
    int next_id_{0};
    rclcpp::Time last_track_time_{0, 0, RCL_ROS_TIME};
    bool track_time_valid_{false};

    std::string fixed_frame_;
    double track_gate_, alpha_, beta_, v_min_, yaw_lp_;
    int max_misses_;

    void intrinsics_cb_(sensor_msgs::msg::CameraInfo::ConstSharedPtr int_msg) {
        if (!camera_model_initialized_) {
            cam_model_.fromCameraInfo(*int_msg);
            camera_model_initialized_ = true;
            RCLCPP_INFO(this->get_logger(), "CAMERA INTRINSICS RECEIVED!");
        }
    }

    void depth_img_cb_(sensor_msgs::msg::Image::ConstSharedPtr depth_img) {
        this->depth_img_ = depth_img;
        if (!depth_received_) {
            depth_received_ = true;
            RCLCPP_INFO(this->get_logger(), "FIRST DEPTH IMAGE RECEIVED!");
        }
    }

    void
    yolo_cb_(vision_msgs::msg::Detection2DArray::ConstSharedPtr yolo_array) {
        this->yolo_array_ = yolo_array;
        if (!yolo_received_) {
            yolo_received_ = true;
            RCLCPP_INFO(this->get_logger(),
                        "FIRST YOLO DETECTION RECEIVED! (Count: %zu)",
                        yolo_array->detections.size());
        }
    }

    void spaam_cb_(geometry_msgs::msg::PoseArray::ConstSharedPtr spaam_array) {
        this->spaam_array_ = spaam_array;
        if (!spaam_received_) {
            spaam_received_ = true;
            RCLCPP_INFO(this->get_logger(),
                        "FIRST DR-SPAAM POSE RECEIVED! (Count: %zu)",
                        spaam_array->poses.size());
        }
    }

    // ------------------------------------------------------------------
    // Ana dongu
    // ------------------------------------------------------------------
    void timer_cb() {
        if (!camera_model_initialized_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "No CAMERA INTRINSIC data available yet.");
            return;
        }

        if (!yolo_array_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "No YOLO data available yet.");
            return;
        }

        if (!depth_img_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "No DEPTH data available yet.");
            return;
        }

        // SPAAM COORDINATE TRANSFORMATION
        std::vector<geometry_msgs::msg::Pose> spaam_in_base;
        if (spaam_array_) {
            for (const auto &lidar_pose : spaam_array_->poses) {
                geometry_msgs::msg::Pose base_lidar_pose;
                if (transform_pose(lidar_pose, spaam_array_->header.frame_id,
                                   "base_link", base_lidar_pose)) {
                    spaam_in_base.push_back(base_lidar_pose);
                }
            }
        }

        // YOLO COORDINATE TRANSFORMATION
        // x = (u - cx) * z_meter / fx;
        // y = (v - cy) * z_meter / fy;

        float fx = cam_model_.fx();
        float cx = cam_model_.cx();
        float fy = cam_model_.fy();
        float cy = cam_model_.cy();

        std::vector<geometry_msgs::msg::Pose> yolo_in_base;
        for (const auto &detection : yolo_array_->detections) {
            double u = detection.bbox.center.position.x;
            double v = detection.bbox.center.position.y;

            float z_meter = process_depth(depth_img_, u, v);

            if (std::isnan(z_meter)) {
                continue;
            }

            float x = (u - cx) * z_meter / fx;
            float y = (v - cy) * z_meter / fy;

            geometry_msgs::msg::Pose optical_pose;
            optical_pose.position.x = x;
            optical_pose.position.y = y;
            optical_pose.position.z = z_meter;
            optical_pose.orientation.w = 1;

            geometry_msgs::msg::Pose base_pose;
            if (transform_pose(optical_pose, depth_img_->header.frame_id,
                               "base_link", base_pose)) {
                yolo_in_base.push_back(base_pose);
            }
        }

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Transformed - YOLO: %zu, SPAAM: %zu",
                             yolo_in_base.size(), spaam_in_base.size());

        // SPAAM - YOLO COORDINATE MATCHING (ayni anda, sensorler arasi)
        std::vector<geometry_msgs::msg::Pose> fused_in_base;

        const double MAX_DIST = 0.5;
        size_t num_yolo = yolo_in_base.size();
        size_t num_spaam = spaam_in_base.size();

        std::vector<int> assignments(num_yolo, -1);
        if (num_yolo > 0 && num_spaam > 0) {
            std::vector<std::vector<double>> cost_matrix(
                num_yolo, std::vector<double>(num_spaam, 0.0));

            for (size_t i = 0; i < num_yolo; ++i) {
                for (size_t j = 0; j < num_spaam; ++j) {
                    double dx = yolo_in_base[i].position.x -
                                spaam_in_base[j].position.x;
                    double dy = yolo_in_base[i].position.y -
                                spaam_in_base[j].position.y;
                    cost_matrix[i][j] = std::sqrt(dx * dx + dy * dy);
                }
            }

            assignments = HungarianAlgorithm::solve(cost_matrix, MAX_DIST);
        }

        for (size_t i = 0; i < num_yolo; ++i) {
            int matched_spaam_idx = assignments[i];
            geometry_msgs::msg::Pose final_pose;
            final_pose.orientation.w = 1.0;

            if (matched_spaam_idx != -1) {
                const auto &sp = spaam_in_base[matched_spaam_idx];
                final_pose.position.x =
                    0.7 * sp.position.x + 0.3 * yolo_in_base[i].position.x;
                final_pose.position.y =
                    0.7 * sp.position.y + 0.3 * yolo_in_base[i].position.y;
            } else {
                final_pose.position.x = yolo_in_base[i].position.x;
                final_pose.position.y = yolo_in_base[i].position.y;
            }
            final_pose.position.z = 0.0;
            fused_in_base.push_back(final_pose);
        }

        std::vector<geometry_msgs::msg::Pose> dets_fixed;
        for (const auto &p : fused_in_base) {
            geometry_msgs::msg::Pose p_fixed;
            if (transform_pose(p, "base_link", fixed_frame_, p_fixed)) {
                dets_fixed.push_back(p_fixed);
            }
        }

        rclcpp::Time now = this->now();
        double dt = 0.1;
        if (track_time_valid_) {
            dt = (now - last_track_time_).seconds();
        }
        last_track_time_ = now;
        track_time_valid_ = true;

        if (dt > 1.0) {
            tracks_.clear();
        }
        dt = std::clamp(dt, 0.01, 0.5);

        update_tracks(dets_fixed, dt);

        geometry_msgs::msg::PoseArray fused_output;
        fused_output.header.stamp = now;
        fused_output.header.frame_id = "base_link";

        std::vector<int> out_ids;
        std::vector<bool> out_has_heading;

        for (const auto &t : tracks_) {
            if (t.misses > 0) {
                continue;
            }

            geometry_msgs::msg::Pose p_fixed;
            p_fixed.position.x = t.x;
            p_fixed.position.y = t.y;
            p_fixed.position.z = 0.0;
            p_fixed.orientation.z = std::sin(t.yaw / 2.0);
            p_fixed.orientation.w = std::cos(t.yaw / 2.0);

            geometry_msgs::msg::Pose p_base;
            if (!transform_pose(p_fixed, fixed_frame_, "base_link", p_base)) {
                continue;
            }
            p_base.position.z = 0.0;
            if (!t.has_heading) {
                p_base.orientation = geometry_msgs::msg::Quaternion();
                p_base.orientation.w = 1.0;
            }

            fused_output.poses.push_back(p_base);
            out_ids.push_back(t.id);
            out_has_heading.push_back(t.has_heading);
        }

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Final Fusion Count: %zu (tracks: %zu)",
                             fused_output.poses.size(), tracks_.size());

        fused_poses_pub_->publish(fused_output);

        visualization_msgs::msg::MarkerArray marker_array;

        for (const char *ns : {"humans", "human_heading", "human_ids"}) {
            visualization_msgs::msg::Marker clear;
            clear.header.frame_id = "base_link";
            clear.ns = ns;
            clear.id = 0;
            clear.action = visualization_msgs::msg::Marker::DELETEALL;
            marker_array.markers.push_back(clear);
        }

        for (size_t k = 0; k < fused_output.poses.size(); ++k) {
            const auto &pose = fused_output.poses[k];
            const int id = out_ids[k];

            visualization_msgs::msg::Marker cylinder;
            cylinder.header.frame_id = "base_link";
            cylinder.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
            cylinder.ns = "humans";
            cylinder.id = id;
            cylinder.type = visualization_msgs::msg::Marker::CYLINDER;
            cylinder.action = visualization_msgs::msg::Marker::ADD;
            cylinder.frame_locked = true;

            cylinder.pose.position.x = pose.position.x;
            cylinder.pose.position.y = pose.position.y;
            cylinder.pose.position.z = 0.85;
            cylinder.pose.orientation.w = 1.0;

            cylinder.scale.x = 0.45;
            cylinder.scale.y = 0.45;
            cylinder.scale.z = 1.70;

            cylinder.color.r = 0.0f;
            cylinder.color.g = 1.0f;
            cylinder.color.b = 0.0f;
            cylinder.color.a = 0.8f;
            marker_array.markers.push_back(cylinder);

            if (out_has_heading[k]) {
                visualization_msgs::msg::Marker arrow;
                arrow.header.frame_id = "base_link";
                arrow.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
                arrow.ns = "human_heading";
                arrow.id = id;
                arrow.type = visualization_msgs::msg::Marker::ARROW;
                arrow.action = visualization_msgs::msg::Marker::ADD;
                arrow.frame_locked = true;

                arrow.pose.position.x = pose.position.x;
                arrow.pose.position.y = pose.position.y;
                arrow.pose.position.z = 1.0;
                arrow.pose.orientation = pose.orientation;

                arrow.scale.x = 0.8;
                arrow.scale.y = 0.10;
                arrow.scale.z = 0.10;

                arrow.color.r = 0.1f;
                arrow.color.g = 0.4f;
                arrow.color.b = 1.0f;
                arrow.color.a = 1.0f;
                marker_array.markers.push_back(arrow);
            }

            visualization_msgs::msg::Marker text;
            text.header.frame_id = "base_link";
            text.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
            text.ns = "human_ids";
            text.id = id;
            text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text.action = visualization_msgs::msg::Marker::ADD;
            text.frame_locked = true;
            text.pose.position.x = pose.position.x;
            text.pose.position.y = pose.position.y;
            text.pose.position.z = 2.0;
            text.pose.orientation.w = 1.0;
            text.scale.z = 0.25;
            text.color.r = text.color.g = text.color.b = 1.0f;
            text.color.a = 1.0f;
            text.text = "ID " + std::to_string(id);
            marker_array.markers.push_back(text);
        }

        marker_pub_->publish(marker_array);
    }

    void update_tracks(const std::vector<geometry_msgs::msg::Pose> &dets,
                       double dt) {
        for (auto &t : tracks_) {
            t.x += t.vx * dt;
            t.y += t.vy * dt;
        }

        std::vector<int> assign(tracks_.size(), -1);
        if (!tracks_.empty() && !dets.empty()) {
            std::vector<std::vector<double>> cost(
                tracks_.size(), std::vector<double>(dets.size(), 0.0));
            for (size_t i = 0; i < tracks_.size(); ++i) {
                for (size_t j = 0; j < dets.size(); ++j) {
                    cost[i][j] = std::hypot(tracks_[i].x - dets[j].position.x,
                                            tracks_[i].y - dets[j].position.y);
                }
            }
            assign = HungarianAlgorithm::solve(cost, track_gate_);
        }

        std::vector<bool> used(dets.size(), false);
        for (size_t i = 0; i < tracks_.size(); ++i) {
            auto &t = tracks_[i];

            if (assign[i] < 0) {
                t.misses++;
                continue;
            }

            const auto &z = dets[assign[i]];
            used[assign[i]] = true;

            const double rx = z.position.x - t.x;
            const double ry = z.position.y - t.y;

            t.x += alpha_ * rx;
            t.y += alpha_ * ry;
            t.vx += beta_ * rx / dt;
            t.vy += beta_ * ry / dt;
            t.misses = 0;

            const double speed = std::hypot(t.vx, t.vy);
            if (speed > v_min_) {
                const double target = std::atan2(t.vy, t.vx);
                if (!t.has_heading) {
                    t.yaw = target;
                    t.has_heading = true;
                } else {
                    // aci sarmasini (+-pi) dikkate alan low-pass
                    const double d = std::remainder(target - t.yaw, 2.0 * M_PI);
                    t.yaw = std::remainder(t.yaw + yaw_lp_ * d, 2.0 * M_PI);
                }
            }
        }

        for (size_t j = 0; j < dets.size(); ++j) {
            if (!used[j]) {
                Track nt;
                nt.id = next_id_++;
                nt.x = dets[j].position.x;
                nt.y = dets[j].position.y;
                tracks_.push_back(nt);
            }
        }

        tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
                                     [&](const Track &t) {
                                         return t.misses > max_misses_;
                                     }),
                      tracks_.end());
    }

    bool transform_pose(const geometry_msgs::msg::Pose &in_pose,
                        const std::string &source_frame,
                        const std::string &target_frame,
                        geometry_msgs::msg::Pose &out_pose) {
        geometry_msgs::msg::PoseStamped pose_in, pose_out;
        pose_in.header.frame_id = source_frame;
        pose_in.header.stamp =
            rclcpp::Time(0); // Take the most recent transform
        pose_in.pose = in_pose;

        try {
            pose_out = tf_buffer_->transform(pose_in, target_frame,
                                             tf2::durationFromSec(0.05));
            out_pose = pose_out.pose;
            return true;
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "TF Error (%s -> %s): %s",
                                 source_frame.c_str(), target_frame.c_str(),
                                 ex.what());
            return false;
        }
    }

    float
    process_depth(const sensor_msgs::msg::Image::ConstSharedPtr &depth_msg,
                  int u, int v, int window = 5) {
        if (!depth_msg)
            return std::numeric_limits<float>::quiet_NaN();

        cv_bridge::CvImageConstPtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvShare(depth_msg);
        } catch (const cv_bridge::Exception &e) {
            return std::numeric_limits<float>::quiet_NaN();
        }

        int half_win = window / 2;
        std::vector<float> valid_depths;
        valid_depths.reserve(window * window);

        int rows = cv_ptr->image.rows;
        int cols = cv_ptr->image.cols;

        for (int dy = -half_win; dy <= half_win; ++dy) {
            for (int dx = -half_win; dx <= half_win; ++dx) {
                int px = u + dx;
                int py = v + dy;

                if (px >= 0 && px < cols && py >= 0 && py < rows) {
                    // ENCODING TYPE_32FC1
                    float z = cv_ptr->image.at<float>(py, px);

                    if (!std::isnan(z) && z > 0.2f && z < 8.0f) {
                        valid_depths.push_back(z);
                    }
                }
            }
        }

        if (valid_depths.empty()) {
            return std::numeric_limits<float>::quiet_NaN();
        }

        size_t mid = valid_depths.size() / 2;
        std::nth_element(valid_depths.begin(), valid_depths.begin() + mid,
                         valid_depths.end());

        return valid_depths[mid];
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HumanDetector>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

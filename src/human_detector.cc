#include <social_nav/human_detector.hh>

namespace SocialNav {
HumanDetector::HumanDetector() : Node("human_detector") {
    fixed_frame_ = declare_parameter<std::string>("fixed_frame", "odom");
    v_min_ = declare_parameter<double>("v_min", 0.3);
    kf_q_ = declare_parameter<double>("kf_q", 1.0);
    kf_r_ = declare_parameter<double>("kf_r", 0.0225);
    max_missed_ = declare_parameter<int>("max_missed", 5);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    intrinsics_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/oakd/rgb/preview/camera_info", rclcpp::QoS(10),
        [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr int_msg) {
            intrinsics_cb_(int_msg);
        });

    depth_img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/oakd/rgb/preview/depth", rclcpp::QoS(10),
        [this](sensor_msgs::msg::Image::ConstSharedPtr depth_img) {
            depth_img_cb_(depth_img);
        });

    yolo_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
        "/yolo_detections", rclcpp::QoS(10),
        [this](vision_msgs::msg::Detection2DArray::ConstSharedPtr yolo_array) {
            yolo_cb_(yolo_array);
        });

    spaam_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "/dr_spaam_detections", rclcpp::QoS(10),
        [this](geometry_msgs::msg::PoseArray::ConstSharedPtr spaam_array) {
            this->spaam_cb_(spaam_array);
        });

    human_poses_pub_ =
        this->create_publisher<human_state_msgs::msg::HumanStateArray>(
            "/human_detector/poses", rclcpp::QoS(10));

    timer_ = this->create_wall_timer(100ms, [this]() { this->timer_cb(); });
};

void HumanDetector::intrinsics_cb_(
    sensor_msgs::msg::CameraInfo::ConstSharedPtr int_msg) {
    if (!camera_model_initialized_) {
        cam_model_.fromCameraInfo(*int_msg);
        camera_model_initialized_ = true;
        RCLCPP_INFO(this->get_logger(), "CAMERA INTRINSICS RECEIVED!");
    }
}

void HumanDetector::depth_img_cb_(
    sensor_msgs::msg::Image::ConstSharedPtr depth_img) {
    this->depth_img_ = depth_img;
    if (!depth_received_) {
        depth_received_ = true;
        RCLCPP_INFO(this->get_logger(), "FIRST DEPTH IMAGE RECEIVED!");
    }
}

void HumanDetector::yolo_cb_(
    vision_msgs::msg::Detection2DArray::ConstSharedPtr yolo_array) {
    this->yolo_array_ = yolo_array;
    if (!yolo_received_) {
        yolo_received_ = true;
        RCLCPP_INFO(this->get_logger(),
                    "FIRST YOLO DETECTION RECEIVED! (Count: %zu)",
                    yolo_array->detections.size());
    }
}

void HumanDetector::spaam_cb_(
    geometry_msgs::msg::PoseArray::ConstSharedPtr spaam_array) {
    this->spaam_array_ = spaam_array;
    if (!spaam_received_) {
        spaam_received_ = true;
        RCLCPP_INFO(this->get_logger(),
                    "FIRST DR-SPAAM POSE RECEIVED! (Count: %zu)",
                    spaam_array->poses.size());
    }
}

void HumanDetector::timer_cb() {
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
            if (transform_pose_(lidar_pose, spaam_array_->header.frame_id,
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
        if (transform_pose_(optical_pose, depth_img_->header.frame_id,
                            "base_link", base_pose)) {
            yolo_in_base.push_back(base_pose);
        }
    }

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Transformed - YOLO: %zu, SPAAM: %zu",
                         yolo_in_base.size(), spaam_in_base.size());

    auto human_output = match_sensors_(yolo_in_base, spaam_in_base);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Final Fusion Count: %zu", human_output.humans.size());

    human_poses_pub_->publish(human_output);
}

void kf_predict(Track &t, double dt, double q) {
    t.px += t.vx * dt;
    t.py += t.vy * dt;
    double dt2 = dt * dt;
    t.P00 += 2.0 * dt * t.P01 + dt2 * t.P11 + q * dt2 * dt2 / 4.0;
    t.P01 += dt * t.P11 + q * dt2 * dt / 2.0;
    t.P11 += q * dt2;
}

void kf_update(Track &t, double zx, double zy, double r) {
    double S = t.P00 + r;
    double K0 = t.P00 / S;
    double K1 = t.P01 / S;
    double ex = zx - t.px, ey = zy - t.py;
    t.px += K0 * ex;
    t.vx += K1 * ex;
    t.py += K0 * ey;
    t.vy += K1 * ey;
    double p00 = t.P00, p01 = t.P01;
    t.P00 = p00 - K0 * p00;
    t.P01 = p01 - K0 * p01;
    t.P11 -= K1 * p01;
}

human_state_msgs::msg::HumanStateArray HumanDetector::match_sensors_(
    std::vector<geometry_msgs::msg::Pose> &yolo_in_base,
    std::vector<geometry_msgs::msg::Pose> &spaam_in_base) {

    human_state_msgs::msg::HumanStateArray human_output;

    human_output.header.stamp = this->now();
    human_output.header.frame_id = "base_link";

    const double MAX_DIST = 0.5;
    size_t num_yolo = yolo_in_base.size();
    size_t num_spaam = spaam_in_base.size();

    std::vector<int> assignments(num_yolo, -1);
    if (num_yolo > 0 && num_spaam > 0) {
        std::vector<std::vector<double>> cost_matrix(
            num_yolo, std::vector<double>(num_spaam, 0.0));

        for (size_t i = 0; i < num_yolo; ++i) {
            for (size_t j = 0; j < num_spaam; ++j) {
                double dx =
                    yolo_in_base[i].position.x - spaam_in_base[j].position.x;
                double dy =
                    yolo_in_base[i].position.y - spaam_in_base[j].position.y;
                cost_matrix[i][j] = std::sqrt(dx * dx + dy * dy);
            }
        }

        assignments = HungarianAlgorithm::solve(cost_matrix, MAX_DIST);
    }

    for (size_t i = 0; i < num_yolo; ++i) {
        int matched_spaam_idx = assignments[i];
        human_state_msgs::msg::HumanState final_pose;
        final_pose.pose.orientation.w = 1.0;

        if (matched_spaam_idx != -1) {
            const auto &sp = spaam_in_base[matched_spaam_idx];
            final_pose.pose.position.x =
                0.7 * sp.position.x + 0.3 * yolo_in_base[i].position.x;
            final_pose.pose.position.y =
                0.7 * sp.position.y + 0.3 * yolo_in_base[i].position.y;
        } else {
            final_pose.pose.position.x = yolo_in_base[i].position.x;
            final_pose.pose.position.y = yolo_in_base[i].position.y;
        }
        final_pose.pose.position.z = 0.0;
        human_output.humans.push_back(final_pose);
    }

    const auto now = this->now();
    const double MAX_TRACK_DIST = 0.8;
    const size_t M = human_output.humans.size();

    std::vector<geometry_msgs::msg::Pose> meas(M);
    std::vector<bool> ok(M, false);
    for (size_t i = 0; i < M; ++i)
        ok[i] = transform_pose_(human_output.humans[i].pose, "base_link",
                                fixed_frame_, meas[i]);

    for (auto &t : tracks_) {
        double dt = std::clamp((now - t.stamp).seconds(), 0.0, 1.0);
        kf_predict(t, dt, kf_q_);
        t.stamp = now;
    }

    std::vector<int> assign(M, -1);
    if (M > 0 && !tracks_.empty()) {
        std::vector<std::vector<double>> cost(
            M, std::vector<double>(tracks_.size(), 0.0));
        for (size_t i = 0; i < M; ++i)
            for (size_t j = 0; j < tracks_.size(); ++j)
                cost[i][j] = std::hypot(meas[i].position.x - tracks_[j].px,
                                        meas[i].position.y - tracks_[j].py);
        assign = HungarianAlgorithm::solve(cost, MAX_TRACK_DIST);
    }

    std::vector<bool> hit(tracks_.size(), false);
    for (size_t i = 0; i < M; ++i) {
        if (!ok[i])
            continue;
        int idx = assign[i];
        if (idx == -1) {
            Track t;
            t.px = meas[i].position.x;
            t.py = meas[i].position.y;
            t.P00 = kf_r_;
            t.P01 = 0.0;
            t.P11 = 1.0;
            t.stamp = now;
            tracks_.push_back(t);
            hit.push_back(true);
            idx = static_cast<int>(tracks_.size()) - 1;
        } else {
            kf_update(tracks_[idx], meas[i].position.x, meas[i].position.y,
                      kf_r_);
            tracks_[idx].missed = 0;
            hit[idx] = true;
        }

        Track &t = tracks_[idx];
        if (std::hypot(t.vx, t.vy) > v_min_) {
            t.yaw = std::atan2(t.vy, t.vx);
            t.has_heading = true;
        }
        if (t.has_heading) {
            tf2::Quaternion q;
            q.setRPY(0, 0, t.yaw);
            geometry_msgs::msg::Pose p = meas[i];
            p.orientation = tf2::toMsg(q);
            geometry_msgs::msg::Pose in_base;
            if (transform_pose_(p, fixed_frame_, "base_link", in_base))
                human_output.humans[i].pose.orientation = in_base.orientation;
        }
    }

    std::vector<Track> kept;
    for (size_t j = 0; j < tracks_.size(); ++j) {
        if (!hit[j])
            tracks_[j].missed++;
        if (tracks_[j].missed <= max_missed_)
            kept.push_back(tracks_[j]);
    }
    tracks_ = std::move(kept);

    return human_output;
}

bool HumanDetector::transform_pose_(const geometry_msgs::msg::Pose &in,
                                    const std::string &src,
                                    const std::string &dst,
                                    geometry_msgs::msg::Pose &out) {
    geometry_msgs::msg::PoseStamped pin, pout;
    pin.header.frame_id = src;
    pin.header.stamp = rclcpp::Time(0);
    pin.pose = in;
    try {
        pout = tf_buffer_->transform(pin, dst, tf2::durationFromSec(0.05));
        out = pout.pose;
        return true;
    } catch (const tf2::TransformException &) {
        return false;
    }
}

float HumanDetector::process_depth(
    const sensor_msgs::msg::Image::ConstSharedPtr &depth_msg, int u, int v,
    int window) {
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
} // namespace SocialNav

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SocialNav::HumanDetector>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

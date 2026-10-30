
#include "geometry_msgs/msg/pose_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <chrono>
#include <fstream>
#include <rclcpp/rclcpp.hpp>

class ProxemicsChecker : public rclcpp::Node {
  public:
    ProxemicsChecker() : Node("proxemics_checker") {
        RCLCPP_INFO(this->get_logger(), "Proxemics Checker Started.");

        csv_file_.open("proxemics_violations.csv", std::ios::out);
        if (csv_file_.is_open()) {
            csv_file_ << "timestamp,actor_id,distance,zone_violation\n";
        }

        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/actors/poses", rclcpp::QoS(10),
            [this](geometry_msgs::msg::PoseArray::SharedPtr pose_arr) {
                pose_cb_(pose_arr);
            });
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/sim_ground_truth_pose", rclcpp::QoS(10).best_effort(),
            [this](nav_msgs::msg::Odometry::SharedPtr odom_msg) {
                odom_cb_(odom_msg);
            });
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1), [this]() { this->proxemics_checker(); });
    }

    ~ProxemicsChecker() override {
        if (csv_file_.is_open()) {
            csv_file_.close();
        }
        RCLCPP_INFO(this->get_logger(), "--- DENEY ÖZETİ ---");
        RCLCPP_INFO(this->get_logger(), "Toplam İntim İhlal Ölçümü: %lu",
                    total_intimate_violations_);
        RCLCPP_INFO(this->get_logger(), "Toplam Kişisel İhlal Ölçümü: %lu",
                    total_personal_violations_);
    }

  private:
    void pose_cb_(geometry_msgs::msg::PoseArray::SharedPtr pose_arr) {
        poseArr_ = pose_arr;
    };

    void odom_cb_(nav_msgs::msg::Odometry::SharedPtr odom_msg) {
        odomMsg_ = odom_msg;
    };

    void proxemics_checker() {
        if (!poseArr_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "No Actor Pose Data Available Yet.");
            return;
        }
        if (!odomMsg_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "No Robot Pose Data Available Yet.");
            return;
        }

        const auto &rpose = odomMsg_->pose;
        double rx = rpose.pose.position.x;
        double ry = rpose.pose.position.y;

        size_t n = poseArr_->poses.size();
        double current_time = this->now().seconds();

        for (size_t i = 0; i < n; ++i) {
            const auto &apose = poseArr_->poses[i];

            double ax = apose.position.x;
            double ay = apose.position.y;

            double dx = rx - ax;
            double dy = ry - ay;

            double dist = std::hypot(dx, dy);

            std::string zone = "None";

            if (dist < 0.45) {
                zone = "Intimate";
                total_intimate_violations_++;
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(),
                                      1000,
                                      "CRITICAL VIOLATION: ACTOR %zu is in "
                                      "intimate area! Distance: %.2f m",
                                      i, dist);
            } else if (dist < 1.2) {
                zone = "Personal";
                total_personal_violations_++;
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 1000,
                    "Personal Space Violation: Actor %zu. Distance: %.2f m", i,
                    dist);
            }

            if (zone != "None" && csv_file_.is_open()) {
                csv_file_ << current_time << "," << i << "," << dist << ","
                          << zone << "\n";
            }
        }
    };
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr pose_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    geometry_msgs::msg::PoseArray::SharedPtr poseArr_;
    nav_msgs::msg::Odometry::SharedPtr odomMsg_;

    std::ofstream csv_file_;
    uint64_t total_intimate_violations_{0};
    uint64_t total_personal_violations_{0};
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ProxemicsChecker>();
    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}

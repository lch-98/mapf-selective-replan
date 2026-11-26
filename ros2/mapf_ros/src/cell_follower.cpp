// cell_follower — 로봇 한 대의 "돌고 → 직진 → 멈춤" 노드 (docs/31 31.7). 로봇 네임스페이스(/robot_i) 안에서 돈다.
//
//   받기: mapf/action (ActionCommand), odom (위치: Gazebo에서는 월드 좌표 정답 위치, 31.18), scan (비상 정지용)
//   보내기: cmd_vel, mapf/status (ActionStatus)
//
// 제어 계산은 mapf_exec::follow_step(순수 함수, gtest로 시험함)을 그대로 쓴다. 이 노드는 토픽 연결 + 가속 제한 + 비상 정지만.
// 시간은 시뮬레이터 시계(use_sim_time)를 따른다.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>

#include "geometry_msgs/msg/twist.hpp"
#include "mapf_exec/follower_control.hpp"
#include "mapf_ros_msgs/msg/action_command.hpp"
#include "mapf_ros_msgs/msg/action_status.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using mapf_ros_msgs::msg::ActionCommand;
using mapf_ros_msgs::msg::ActionStatus;

class CellFollower : public rclcpp::Node {
public:
    CellFollower() : Node("cell_follower") {
        params_.max_linear = declare_parameter("max_linear", 0.15);
        params_.max_angular = declare_parameter("max_angular", 1.0);
        linear_accel_ = declare_parameter("linear_accel", 0.5);    // m/s²
        angular_accel_ = declare_parameter("angular_accel", 3.0);  // rad/s²
        estop_distance_ = declare_parameter("estop_distance", 0.25);
        estop_half_angle_ = declare_parameter("estop_half_angle_deg", 30.0) * M_PI / 180.0;
        const double rate = declare_parameter("rate_hz", 20.0);
        dt_ = 1.0 / rate;

        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        status_pub_ = create_publisher<ActionStatus>("mapf/status", rclcpp::QoS(50).reliable());
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, [this](nav_msgs::msg::Odometry::ConstSharedPtr m) { on_odom(*m); });
        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::LaserScan::ConstSharedPtr m) { scan_ = m; });
        action_sub_ = create_subscription<ActionCommand>(
            "mapf/action", rclcpp::QoS(50).reliable(), [this](ActionCommand::ConstSharedPtr m) { on_action(*m); });
        timer_ = rclcpp::create_timer(this, get_clock(), std::chrono::duration<double>(dt_), [this] { tick(); });
        RCLCPP_INFO(get_logger(), "최고 선속도 %.2f m/s, 최고 각속도 %.2f rad/s", params_.max_linear, params_.max_angular);
    }

private:
    void on_odom(const nav_msgs::msg::Odometry& m) {
        const auto& q = m.pose.pose.orientation;
        pose_ = mapf_exec::Pose2{m.pose.pose.position.x, m.pose.pose.position.y,
                                 std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))};
    }

    void on_action(const ActionCommand& c) {
        if (last_seq_ && c.seq <= *last_seq_) return;  // 이미 받은 동작
        last_seq_ = c.seq;
        cmd_ = c;
        phase_ = mapf_exec::Phase::kRotate;
        wait_end_ = now() + rclcpp::Duration::from_seconds(c.duration_s);
        publish_status(ActionStatus::ACTIVE, 0.0);
    }

    bool front_blocked() const {
        if (!scan_) return false;
        for (std::size_t k = 0; k < scan_->ranges.size(); ++k) {
            const double a = mapf_exec::wrap_angle(scan_->angle_min + k * scan_->angle_increment);
            const float r = scan_->ranges[k];
            if (std::abs(a) <= estop_half_angle_ && std::isfinite(r) && r >= scan_->range_min && r < estop_distance_) {
                return true;
            }
        }
        return false;
    }

    void tick() {
        double v_cmd = 0.0, w_cmd = 0.0;
        if (cmd_ && pose_) {
            if (cmd_->kind == ActionCommand::WAIT) {
                if (now() >= wait_end_) finish(ActionStatus::DONE);
            } else {
                const mapf_exec::Segment seg{{cmd_->from_x, cmd_->from_y}, {cmd_->target_x, cmd_->target_y}};
                const auto out = mapf_exec::follow_step(*pose_, seg, phase_, params_);
                phase_ = out.phase;
                if (phase_ == mapf_exec::Phase::kDone) {
                    finish(ActionStatus::DONE);
                } else if (phase_ == mapf_exec::Phase::kDrive && front_blocked()) {
                    RCLCPP_ERROR(get_logger(), "비상 정지: 정면 %.2f m 안에 무언가 있음 (동작 %u)", estop_distance_, cmd_->seq);
                    finish(ActionStatus::ESTOP);
                } else {
                    v_cmd = out.v;
                    w_cmd = out.w;
                }
            }
        }
        // 할 동작이 없으면 cmd_vel을 보내지 않는다(멈춘 직후 한 번만 0을 보냄) — 사람이 teleop으로 조종할 때 싸우지 않게
        const bool active = cmd_.has_value();
        if (!active && !was_active_) return;
        was_active_ = active;
        // 가속 제한(멈출 때는 바로 0)
        if (v_cmd == 0.0 && w_cmd == 0.0) {
            v_ = w_ = 0.0;
        } else {
            v_ += std::clamp(v_cmd - v_, -linear_accel_ * dt_, linear_accel_ * dt_);
            w_ += std::clamp(w_cmd - w_, -angular_accel_ * dt_, angular_accel_ * dt_);
        }
        geometry_msgs::msg::Twist tw;
        tw.linear.x = v_;
        tw.angular.z = w_;
        cmd_pub_->publish(tw);
    }

    void finish(uint8_t state) {
        double err = 0.0;
        if (cmd_->kind == ActionCommand::MOVE && pose_) {
            err = std::hypot(cmd_->target_x - pose_->x, cmd_->target_y - pose_->y);
        }
        publish_status(state, err);
        cmd_.reset();
    }

    void publish_status(uint8_t state, double err) {
        ActionStatus s;
        s.seq = cmd_ ? cmd_->seq : 0;
        s.state = state;
        s.pos_err_m = err;
        status_pub_->publish(s);
    }

    mapf_exec::FollowerParams params_;
    double linear_accel_{0.5}, angular_accel_{3.0}, estop_distance_{0.25}, estop_half_angle_{0.5236}, dt_{0.05};
    double v_{0.0}, w_{0.0};
    bool was_active_{false};
    std::optional<mapf_exec::Pose2> pose_;
    std::optional<ActionCommand> cmd_;
    std::optional<uint32_t> last_seq_;
    mapf_exec::Phase phase_{mapf_exec::Phase::kDone};
    rclcpp::Time wait_end_{0, 0, RCL_ROS_TIME};
    sensor_msgs::msg::LaserScan::ConstSharedPtr scan_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<ActionStatus>::SharedPtr status_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<ActionCommand>::SharedPtr action_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CellFollower>());
    rclcpp::shutdown();
    return 0;
}

// cell_follower — 로봇 한 대의 "돌고 → 직진 → 멈춤" 노드 (docs/31 31.7). 로봇 네임스페이스(/robot_i) 안에서 돈다.
//
//   받기: mapf/action (ActionCommand), odom (위치: Gazebo에서는 월드 좌표 정답 위치, 31.18), scan (비상 정지용)
//   보내기: cmd_vel, mapf/status (ActionStatus)
//
// 제어 계산은 mapf_exec::follow_step(순수 함수, gtest로 시험함)을 그대로 쓴다. 이 노드는 토픽 연결 + 가속 제한 + 비상 정지만.
// 곧게 지나가기(31.24): 하는 중에 받은 동작은 다음 차례(한 개)로 두고, 그것이 같은 방향 MOVE면 지금 목표 칸 중심에서 서지 않고
// 지나가며 DONE을 보낸 뒤 바로 이어 간다. CANCEL로 시작 전의 다음 차례를 버릴 수 있다. 실행기가 미리 안 보내면 예전과 같다.
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
        // 비상 정지 v2(31.24 보충, 기본 꺼짐): 앞으로 갈 거리 안의 물체만 + 광선 estop_min_points개 이상.
        // 기준 = min(estop_distance, 남은 이동 + 센서→몸 앞 0.089 m + 여유 0.10 m). 벽을 보고 칸 중심에 설 때 잡음 한 점으로 서지 않게.
        estop_path_aware_ = declare_parameter("estop_path_aware", false);
        estop_min_points_ = declare_parameter("estop_min_points", 1);
        estop_front_ = declare_parameter("estop_front_from_sensor", 0.089);
        estop_margin_ = declare_parameter("estop_margin", 0.10);
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
        if (c.kind == ActionCommand::CANCEL) {  // 줄 서 있던(아직 시작 안 한) 동작 취소 — 이미 시작했으면 무시
            if (next_ && next_->seq == c.seq) {
                next_.reset();
                ActionStatus st;
                st.seq = c.seq;
                st.state = ActionStatus::CANCELED;
                status_pub_->publish(st);
            }
            return;
        }
        if (last_seq_ && c.seq <= *last_seq_) return;  // 이미 받은 동작
        last_seq_ = c.seq;
        if (cmd_) {
            next_ = c;  // 하는 중인 동작 다음 차례(곧게 지나가기, 31.24)
            return;
        }
        start(c, mapf_exec::Phase::kRotate);
    }

    void start(const ActionCommand& c, mapf_exec::Phase phase) {
        cmd_ = c;
        phase_ = phase;
        wait_end_ = now() + rclcpp::Duration::from_seconds(c.duration_s);
        publish_status(ActionStatus::ACTIVE, 0.0);
    }

    static mapf_exec::Segment segment_of(const ActionCommand& c) {
        return {{c.from_x, c.from_y}, {c.target_x, c.target_y}};
    }

    // 다음 차례가 지금 구간에 곧게 이어지는 MOVE면 그 길이(m), 아니면 0(지금 목표에서 섬)
    double carry() const {
        if (!cmd_ || !next_ || cmd_->kind != ActionCommand::MOVE || next_->kind != ActionCommand::MOVE) return 0.0;
        const auto a = segment_of(*cmd_), b = segment_of(*next_);
        return mapf_exec::continues_straight(a, b) ? mapf_exec::segment_length(b) : 0.0;
    }

    // remaining: 이 동작(+ 곧게 이어지는 다음 동작)에서 앞으로 더 갈 거리(m)
    bool front_blocked(double remaining) const {
        if (!scan_) return false;
        const double limit =
            estop_path_aware_ ? std::min(estop_distance_, std::max(0.0, remaining) + estop_front_ + estop_margin_) : estop_distance_;
        int hits = 0;
        for (std::size_t k = 0; k < scan_->ranges.size(); ++k) {
            const double a = mapf_exec::wrap_angle(scan_->angle_min + k * scan_->angle_increment);
            const float r = scan_->ranges[k];
            if (std::abs(a) <= estop_half_angle_ && std::isfinite(r) && r >= scan_->range_min && r < limit) ++hits;
        }
        if (hits >= std::max(1, estop_min_points_)) {
            last_estop_limit_ = limit;
            return true;
        }
        return false;
    }

    void tick() {
        double v_cmd = 0.0, w_cmd = 0.0;
        // 한 tick 안에서: 지금 동작이 끝나면 다음 차례를 바로 시작해 같은 tick에 속도를 계산한다(지나가기에서 0을 한 번 내지 않게)
        for (int guard = 0; guard < 2 && cmd_ && pose_; ++guard) {
            if (cmd_->kind == ActionCommand::WAIT) {
                if (now() >= wait_end_) {
                    finish(ActionStatus::DONE);
                    start_next(false);  // 다음 tick부터(예전처럼 한 번 0을 보냄)
                }
                break;
            }
            const double c = carry();
            const auto out = mapf_exec::follow_step(*pose_, segment_of(*cmd_), phase_, params_, c);
            phase_ = out.phase;
            if (phase_ == mapf_exec::Phase::kDone) {
                finish(ActionStatus::DONE);
                const bool started = start_next(c > 0.0);
                if (started && c > 0.0) continue;  // 곧게 이어 감: 같은 tick에 다음 구간 속도. 꺾으면 예전처럼 이번 tick은 섬
            } else if (phase_ == mapf_exec::Phase::kDrive &&
                       front_blocked(mapf_exec::segment_error(*pose_, segment_of(*cmd_)).along + c)) {
                RCLCPP_ERROR(get_logger(), "비상 정지: 정면 %.3f m 안에 무언가 있음 (동작 %u)", last_estop_limit_, cmd_->seq);
                finish(ActionStatus::ESTOP);
                next_.reset();
            } else {
                v_cmd = out.v;
                w_cmd = out.w;
            }
            break;
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

    // 줄 서 있던 동작을 시작. straight면 회전 없이 직진으로 이어 간다(방향이 30° 넘게 틀어져 있으면 follow_step이 다시 회전으로 바꿈)
    bool start_next(bool straight) {
        if (!next_) return false;
        const ActionCommand c = *next_;
        next_.reset();
        start(c, straight ? mapf_exec::Phase::kDrive : mapf_exec::Phase::kRotate);
        return true;
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
    double v_{0.0}, w_{0.0}, estop_front_{0.089}, estop_margin_{0.10};
    mutable double last_estop_limit_{0.0};
    bool estop_path_aware_{false};
    int estop_min_points_{1};
    bool was_active_{false};
    std::optional<mapf_exec::Pose2> pose_;
    std::optional<ActionCommand> cmd_, next_;
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

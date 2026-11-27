// mapf_executive — 저장된 처음 계획을 ADG로 실행하고, 장애물이 나타나면 다시 계획한다 (docs/31 31.4~31.6, 31.22, 31.23).
//
//   ros2 run mapf_ros mapf_executive --ros-args -p use_sim_time:=true -p scenario:=<시나리오.json> -p out_dir:=<폴더>
//        [-p method:=ours|full] [-p d_detect:=2.0] [-p t_wait:=3.0] [-p detection:=simulated|lidar] [-p pass_through:=false]
//
// 1. 시나리오 JSON의 initial_plan(make_scenario가 코어 plan()이 성공할 때까지 다시 뽑아 저장한 것)을 읽는다. 실행 중 plan()은 안 부른다.
// 2. 계획 검사 (가) 통과 → ADG → 내줄 수 있는 동작을 /robot_i/mapf/action으로 보냄 → DONE 받으면 다음.
// 3. 장애물(7-4, 31.5): 시나리오 obstacles[]의 시각에 상자 생성(그 칸을 누가 쓰고 있으면 비워질 때까지 미룸).
//    detection:=simulated(7-4, 기본): 어떤 로봇 중심이 칸 중심에서 d_detect 안이고 사이에 선반이 없으면 발견, 발견 + t_wait초 = 확정.
//    detection:=lidar(7-5, 31.25): 로봇마다 /robot_i/scan 점 → 지도에 없는 물체 칸(mapf_exec::unknown_cells, 다른 로봇 ± 0.2 m 점은 버림)
//      → 처음 보이면 발견, t_wait 동안 0.2초 칸의 90% 이상에서 보이면 확정, 모자라면 버림(ObstacleConfirmer). 시나리오에 없는 칸이 확정되면
//      "오탐"(phantom)으로 세고 그래도 재계획한다(실제 시스템처럼). 비교용으로 흉내 규칙의 발견 시각(sim_detect_s)도 같이 적는다.
//    발견 뒤에는 그 칸으로 들어가는 동작을 안 내줌(보류). 확정 → 새 동작 시작 금지 → 이동 중인 로봇이 다 칸 중심에 서면:
//    번호 다시 붙이기(mapf_exec::retime) → 코어 replan()(우리 방법) 또는 full_replan() → 계획 검사 → 새 ADG → 다시 출발.
// 4. 상태(로봇마다): MOVE 이동 / WORK 작업 / YIELD 약속 때문에 서서 기다림(양보) / HOLD 장애물 앞 보류 / STOP 재계획 대기 /
//    DONE 끝 / ESTOP 비상 정지. run.csv에 남기고, RViz 마커(/mapf/markers)로 보여 준다.
// 5. 기록(out_dir): run.csv, map.txt, executive.json(동작 시각, 기다린 시간, 접촉, 칸 사용 순서, 장애물·재계획 사건).
// 6. 곧게 지나가기(pass_through:=true, 31.24, 기본 꺼짐): 이동 동작을 하는 중인 로봇의 다음 동작이 같은 방향 이동이고 약속이 다 지켜졌으면
//    미리 보낸다 → follower가 칸 중심에서 서지 않고 지나감. 미리 보낸 동작의 칸에서 장애물이 발견되면 CANCEL(시작 전이면 follower가 버림).
// debug_drop_cross_edges=true는 변이 검사용(약속을 뺌). 실험에 쓰지 말 것.
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <deque>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "gazebo_msgs/msg/contacts_state.hpp"
#include "gazebo_msgs/srv/spawn_entity.hpp"
#include "mapf/prioritized_planner.hpp"
#include "mapf/task_schedule.hpp"
#include "mapf_exec/adg.hpp"
#include "mapf_exec/geometry.hpp"
#include "mapf_exec/lidar_detect.hpp"
#include "mapf_exec/plan_check.hpp"
#include "mapf_exec/retime.hpp"
#include "mapf_ros_msgs/msg/action_command.hpp"
#include "mapf_ros_msgs/msg/action_status.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using json = nlohmann::json;
using mapf_exec::AdgAction;
using mapf_exec::GridCell;
using mapf_ros_msgs::msg::ActionCommand;
using mapf_ros_msgs::msg::ActionStatus;
using visualization_msgs::msg::Marker;

namespace {

struct Robot {
    int id{0};
    std::string name;
    GridCell goal{};
    int service_steps{0};
    double service_factor{1.0};
    rclcpp::Publisher<ActionCommand>::SharedPtr pub;
    rclcpp::Subscription<ActionStatus>::SharedPtr status_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<gazebo_msgs::msg::ContactsState>::SharedPtr bump_sub;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
    std::optional<mapf_exec::Pose2> pose;
    std::deque<std::pair<double, mapf_exec::Pose2>> history;  // (시뮬레이터 시각, 자세) 최근 3초 — 스캔 시각의 자세를 찾으려고
    long scans{0};
    uint32_t next_seq{0};                // follower는 더 큰 seq만 받으므로 재계획 뒤에도 계속 늘린다
    int active{-1};                      // 지금 ADG에서 하는 동작
    uint32_t active_seq{0};
    int queued{-1};                      // 곧게 지나가기: 미리 보낸 다음 동작(아직 follower가 시작 전일 수 있음)
    uint32_t queued_seq{0};
    bool cancel_sent{false};
    int passes{0}, cancels{0};
    double service_end_s{-1.0};          // 하는 중인 작업이 끝나는 시각(시작 기준 초)
    std::optional<uint32_t> legacy_seq;  // 재계획 전 ADG에서 시작한 작업(끝날 때까지 다음 동작 안 내줌)
    GridCell at{};
    double last_done_s{0.0}, adg_wait_s{0.0}, finish_s{-1.0}, max_pos_err{0.0};
    std::map<std::string, double> state_time;  // 상태별 머문 시간(초)
    long contacts{0};
    std::string state{"IDLE"}, waiting_for;
    bool estop{false};
};

struct Obstacle {
    GridCell cell{};
    double spawn_at_s{0.0};
    double spawned_s{-1.0}, detected_s{-1.0}, confirmed_s{-1.0}, replanned_s{-1.0};
    double sim_detect_s{-1.0}, first_detect_s{-1.0};  // 흉내 규칙의 발견 시각(비교용), 맨 처음 발견(버려졌다 다시 보여도 유지)
    std::string detected_by, sim_detected_by;
    bool spawn_requested{false};
    bool phantom{false};  // LiDAR가 찾았지만 시나리오에 없는 칸(오탐)
    int drops{0}, bins_seen{0}, bins_total{0};
    json event = json::object();
};

std::string cell_str(GridCell c) { return std::to_string(c.x) + ":" + std::to_string(c.y); }

std::vector<GridCell> dedup(const std::vector<GridCell>& p) {
    std::vector<GridCell> out;
    for (const auto& c : p) {
        if (out.empty() || out.back() != c) out.push_back(c);
    }
    return out;
}

std::array<float, 3> robot_color(int i) {
    static const float c[10][3] = {{0.12f, 0.47f, 0.71f}, {1.0f, 0.5f, 0.05f}, {0.17f, 0.63f, 0.17f}, {0.84f, 0.15f, 0.16f},
                                   {0.58f, 0.4f, 0.74f},  {0.55f, 0.34f, 0.29f}, {0.89f, 0.47f, 0.76f}, {0.5f, 0.5f, 0.5f},
                                   {0.74f, 0.74f, 0.13f}, {0.09f, 0.75f, 0.81f}};
    return {c[i % 10][0], c[i % 10][1], c[i % 10][2]};
}

std::array<float, 3> state_color(const std::string& s) {
    if (s == "MOVE") return {0.2f, 0.5f, 1.0f};
    if (s == "WORK") return {0.1f, 0.1f, 0.1f};
    if (s == "YIELD") return {1.0f, 0.6f, 0.0f};
    if (s == "HOLD") return {0.9f, 0.1f, 0.1f};
    if (s == "STOP") return {0.6f, 0.2f, 0.8f};
    if (s == "DONE") return {0.2f, 0.75f, 0.2f};
    if (s == "ESTOP") return {1.0f, 0.0f, 1.0f};
    return {0.7f, 0.7f, 0.7f};
}

std::string fmt1(double v) {
    char b[32];
    std::snprintf(b, sizeof(b), "%.1f", v);
    return b;
}

}  // namespace

class MapfExecutive : public rclcpp::Node {
public:
    MapfExecutive() : Node("mapf_executive") {
        const std::string scenario_path = declare_parameter("scenario", std::string());
        out_dir_ = declare_parameter("out_dir", std::string());
        drop_ = declare_parameter("debug_drop_cross_edges", false);
        method_ = declare_parameter("method", std::string("ours"));
        d_detect_ = declare_parameter("d_detect", 2.0);
        t_wait_ = declare_parameter("t_wait", 3.0);
        pass_through_ = declare_parameter("pass_through", false);
        detection_ = declare_parameter("detection", std::string("simulated"));
        lidar_.d_detect = d_detect_;
        lidar_.min_points = declare_parameter("lidar_min_points", 3);
        lidar_.robot_filter_r = declare_parameter("lidar_robot_filter_r", 0.2);
        if (detection_ != "simulated" && detection_ != "lidar") throw std::runtime_error("detection은 simulated 또는 lidar");
        confirmer_ = std::make_unique<mapf_exec::ObstacleConfirmer>(t_wait_);
        start_wait_s_ = declare_parameter("start_wait_s", 60.0);
        total_timeout_s_ = declare_parameter("total_timeout_s", 1800.0);
        const double rate = declare_parameter("rate_hz", 20.0);
        if (scenario_path.empty() || out_dir_.empty()) throw std::runtime_error("scenario와 out_dir 파라미터가 필요함");
        if (method_ != "ours" && method_ != "full") throw std::runtime_error("method는 ours 또는 full");
        std::filesystem::create_directories(out_dir_);

        std::ifstream f(scenario_path);
        if (!f) throw std::runtime_error("시나리오를 못 엶: " + scenario_path);
        sc_ = json::parse(f);
        rows_ = sc_.at("map").get<std::vector<std::string>>();
        cell_ = sc_.at("cell_size").get<double>();
        dt_nom_ = sc_.at("dt_nom_s").get<double>();
        height_ = static_cast<int>(rows_.size());
        if (!sc_.contains("initial_plan")) {
            throw std::runtime_error("시나리오에 initial_plan이 없음 — make_scenario로 만든(처음 계획이 성공한) 시나리오가 필요함");
        }

        std::map<int, std::vector<GridCell>> paths;
        std::map<int, int> service;
        for (const auto& r : sc_.at("robots")) {
            Robot rb;
            rb.id = r.at("id");
            rb.name = r.at("name");
            rb.goal = {r.at("goal")[0].get<int>(), r.at("goal")[1].get<int>()};
            rb.service_steps = r.value("service_steps", 0);
            rb.service_factor = r.value("service_factor", 1.0);
            const auto& p = sc_["initial_plan"].at(std::to_string(rb.id));
            std::vector<GridCell> path;
            for (std::size_t t = 0; t < p.size(); ++t) {
                if (p[t].at(2).get<int>() != static_cast<int>(t)) throw std::runtime_error(rb.name + ": 계획 번호가 0부터 1씩이 아님");
                path.push_back({p[t].at(0).get<int>(), p[t].at(1).get<int>()});
            }
            rb.at = path.front();
            paths[rb.id] = path;
            service[rb.id] = rb.service_steps;
            robots_[rb.id] = std::move(rb);
        }
        for (const auto& o : sc_.value("obstacles", json::array())) {
            Obstacle ob;
            ob.cell = {o.at("cell")[0].get<int>(), o.at("cell")[1].get<int>()};
            ob.spawn_at_s = o.value("spawn_time_s", 0.0);
            obstacles_.push_back(ob);
        }
        for (const auto& [id, p] : paths) initial_plan_steps_ = std::max(initial_plan_steps_, static_cast<int>(p.size()) - 1);

        violations_ = mapf_exec::check_plan(rows_, paths);
        for (const auto& v : violations_) RCLCPP_ERROR(get_logger(), "계획 검사 실패: %s", v.describe().c_str());
        set_plan(paths, service, {});
        if (drop_) RCLCPP_WARN(get_logger(), "변이 검사용: 로봇 사이 약속을 모두 뺐다(부딪혀야 정상)");
        RCLCPP_INFO(get_logger(), "시나리오 %s: 로봇 %zu대, 동작 %zu개, 약속 %d개, 장애물 %zu개, 재계획 방법 %s, 곧게 지나가기 %s",
                    sc_.at("name").get<std::string>().c_str(), robots_.size(), adg_->actions().size(),
                    adg_->num_cross_edges(), obstacles_.size(), method_.c_str(), pass_through_ ? "켬" : "끔");
        RCLCPP_INFO(get_logger(), "장애물 발견: %s (d_detect %.1f m, t_wait %.1f초)", detection_.c_str(), d_detect_, t_wait_);

        const auto rel = rclcpp::QoS(50).reliable();
        for (auto& [id, rb] : robots_) {
            Robot* r = &rb;
            rb.pub = create_publisher<ActionCommand>("/" + rb.name + "/mapf/action", rel);
            rb.status_sub = create_subscription<ActionStatus>(
                "/" + rb.name + "/mapf/status", rel, [this, r](ActionStatus::ConstSharedPtr m) { on_status(*r, *m); });
            rb.odom_sub = create_subscription<nav_msgs::msg::Odometry>(
                "/" + rb.name + "/odom", 10, [r](nav_msgs::msg::Odometry::ConstSharedPtr m) {
                    const auto& q = m->pose.pose.orientation;
                    r->pose = mapf_exec::Pose2{m->pose.pose.position.x, m->pose.pose.position.y,
                                               std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))};
                    const double ts = rclcpp::Time(m->header.stamp).seconds();
                    r->history.emplace_back(ts, *r->pose);
                    while (!r->history.empty() && r->history.front().first < ts - 3.0) r->history.pop_front();
                });
            if (detection_ == "lidar") {
                rb.scan_sub = create_subscription<sensor_msgs::msg::LaserScan>(
                    "/" + rb.name + "/scan", rclcpp::SensorDataQoS(),
                    [this, r](sensor_msgs::msg::LaserScan::ConstSharedPtr m) { on_scan(*r, m); });
            }
            rb.bump_sub = create_subscription<gazebo_msgs::msg::ContactsState>(
                "/" + rb.name + "/bumper_states", rclcpp::SensorDataQoS(),
                [this, r](gazebo_msgs::msg::ContactsState::ConstSharedPtr m) { on_contact(*r, *m); });
        }
        spawn_cli_ = create_client<gazebo_msgs::srv::SpawnEntity>("/spawn_entity");
        marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/mapf/markers", 10);

        csv_.open(out_dir_ + "/run.csv");
        csv_ << "t,robot,x,y,yaw,kind,held,seg_sx,seg_sy,seg_gx,seg_gy,state,waiting_for\n";
        {
            std::ofstream mf(out_dir_ + "/map.txt");
            for (const auto& r : rows_) mf << r << "\n";
        }
        wall_start_ = std::chrono::steady_clock::now();
        timer_ = rclcpp::create_timer(this, get_clock(), std::chrono::duration<double>(1.0 / rate), [this] { tick(); });
    }

    bool finished() const { return finished_; }
    int exit_code() const { return ok_ ? 0 : 1; }

private:
    double rel_now() { return now().seconds() - t0_; }

    // 재계획을 마친 장애물은 지도에서 막힌 칸('#')으로. include_current면 지금 재계획 중인 장애물도.
    std::vector<std::string> rows_with_obstacles(bool include_current) const {
        auto rows = rows_;
        for (std::size_t k = 0; k < obstacles_.size(); ++k) {
            const auto& o = obstacles_[k];
            if (o.replanned_s >= 0 || (include_current && static_cast<int>(k) == replanning_)) rows[o.cell.y][o.cell.x] = '#';
        }
        return rows;
    }

    void set_plan(const std::map<int, std::vector<GridCell>>& paths, const std::map<int, int>& service,
                  const std::map<int, int>& service_start) {
        adg_ = std::make_unique<mapf_exec::Adg>(paths, service, !drop_, service_start);
        for (const auto& A : adg_->actions()) {
            if (A.kind == AdgAction::Kind::kMove) {
                planned_order_[cell_str(A.to)].push_back({static_cast<long>(plan_epoch_) * 100000 + A.plan_t, A.robot});
            }
        }
        plan_epoch_++;
    }

    bool ready() {
        for (auto& [id, rb] : robots_) {
            if (!rb.pose || rb.pub->get_subscription_count() == 0) return false;
        }
        return true;
    }

    // ── 로봇이 보내는 상태 ──
    void on_status(Robot& r, const ActionStatus& m) {
        if (!started_ || finished_ || m.state == ActionStatus::ACTIVE) return;
        const double t = rel_now();
        if (r.legacy_seq && m.seq == *r.legacy_seq && m.state == ActionStatus::DONE) {  // 재계획 전에 시작한 작업이 끝남
            r.legacy_seq.reset();
            r.service_end_s = -1;
            r.last_done_s = t;
            return;
        }
        if (m.state == ActionStatus::CANCELED) {  // 미리 보낸 동작을 follower가 시작 전에 버림 → ADG에서도 되돌림
            if (r.queued >= 0 && m.seq == r.queued_seq) {
                adg_->unrelease(r.queued);
                r.queued = -1;
                r.cancel_sent = false;
                r.cancels++;
            }
            return;
        }
        if (r.active < 0 || m.seq != r.active_seq) return;
        const int a = r.active;
        const auto& A = adg_->actions()[a];
        if (m.state == ActionStatus::DONE) {
            adg_->done(a);
            const bool passed = r.queued >= 0;
            r.active = -1;
            if (passed) {  // 곧게 지나감: follower가 이미 다음 동작을 시작했다
                if (r.cancel_sent && enters_detected_obstacle(adg_->actions()[r.queued])) {  // 재계획 대기 취소가 늦은 것은 괜찮음
                    RCLCPP_WARN(get_logger(), "%s: 취소가 늦어 장애물 칸으로 가는 동작을 이미 시작함", r.name.c_str());
                    late_cancels_++;
                }
                r.active = r.queued;
                r.active_seq = r.queued_seq;
                r.queued = -1;
                r.cancel_sent = false;
                r.passes++;
            }
            r.service_end_s = -1;
            r.at = A.to;
            r.last_done_s = t;
            r.max_pos_err = std::max(r.max_pos_err, static_cast<double>(m.pos_err_m));
            if (A.kind == AdgAction::Kind::kMove) entered_[cell_str(A.to)].push_back({t, r.id});
            action_log_.push_back({{"robot", r.name}, {"plan_epoch", plan_epoch_ - 1}, {"index", A.index},
                                   {"kind", A.kind == AdgAction::Kind::kMove ? "MOVE" : "SERVICE"},
                                   {"to", {A.to.x, A.to.y}}, {"plan_t", A.plan_t}, {"done_s", t}, {"pos_err_m", m.pos_err_m},
                                   {"passed_through", passed}});
        } else {
            r.estop = m.state == ActionStatus::ESTOP;
            abort_reason_ = r.name + (r.estop ? " 비상 정지" : " 보류(HELD)") + " — 동작 " + std::to_string(A.index);
            RCLCPP_ERROR(get_logger(), "%s", abort_reason_.c_str());
            stop(false);
        }
    }

    // 시각 ts의 자세(기록 사이를 직선으로 이음). 기록이 없으면 지금 자세
    static std::optional<mapf_exec::Pose2> pose_at(const Robot& r, double ts) {
        if (r.history.empty()) return r.pose;
        if (ts <= r.history.front().first) return r.history.front().second;
        if (ts >= r.history.back().first) return r.history.back().second;
        for (std::size_t i = 1; i < r.history.size(); ++i) {
            const auto& [t1, p1] = r.history[i];
            if (t1 < ts) continue;
            const auto& [t0, p0] = r.history[i - 1];
            const double u = t1 > t0 ? (ts - t0) / (t1 - t0) : 0.0;
            return mapf_exec::Pose2{p0.x + u * (p1.x - p0.x), p0.y + u * (p1.y - p0.y),
                                    mapf_exec::wrap_angle(p0.yaw + u * mapf_exec::wrap_angle(p1.yaw - p0.yaw))};
        }
        return r.history.back().second;
    }

    // 지금 알고 있는 지도: 선반 + 확정된 장애물(재계획 전이어도)
    std::vector<std::string> known_rows() const {
        auto rows = rows_;
        for (const auto& o : obstacles_) {
            if (o.confirmed_s >= 0) rows[o.cell.y][o.cell.x] = '#';
        }
        return rows;
    }

    // ── LiDAR 발견(7-5) ──
    // 스캔은 바로 쓰지 않고 모아 둔다: 스캔이 그 시각의 odom보다 먼저 도착하면 회전 중인 로봇의 옛 자세로 점을 놓아
    // 선반 점이 빈칸으로 간다(31.25, 실제로 겪음). 모든 로봇의 odom이 스캔 시각을 넘어선 뒤 사이를 이어(보간) 처리한다.
    void on_scan(Robot& r, sensor_msgs::msg::LaserScan::ConstSharedPtr m) {
        if (!started_ || finished_) return;
        pending_scans_.push_back({&r, m});
    }

    void process_scans() {
        double covered = 1e18;
        for (const auto& [id, o] : robots_) covered = std::min(covered, o.history.empty() ? -1.0 : o.history.back().first);
        while (!pending_scans_.empty()) {
            auto [r, m] = pending_scans_.front();
            const double ts = rclcpp::Time(m->header.stamp).seconds();
            if (ts > covered && now().seconds() - ts < 0.5) break;  // 아직 odom이 안 따라옴(0.5초 넘게 밀리면 그냥 처리)
            if (ts > covered) late_scans_++;
            pending_scans_.pop_front();
            scan_one(*r, *m, ts);
        }
    }

    void scan_one(Robot& r, const sensor_msgs::msg::LaserScan& m, double ts) {
        const auto me = pose_at(r, ts);
        if (!me) return;
        std::vector<mapf_exec::Vec2> others;
        for (const auto& [id, o] : robots_) {
            if (&o == &r) continue;
            if (const auto p = pose_at(o, ts)) others.push_back({p->x, p->y});
        }
        mapf_exec::ScanData sd;
        sd.angle_min = m.angle_min;
        sd.angle_increment = m.angle_increment;
        sd.range_min = m.range_min;
        sd.range_max = m.range_max;
        sd.ranges = m.ranges;
        const auto cells = mapf_exec::unknown_cells(*me, sd, others, known_rows(), cell_, lidar_);
        std::vector<GridCell> seen;
        for (const auto& [xy, n] : cells) {
            const GridCell g{xy.first, xy.second};
            if (cell_in_use(g)) {  // 로봇이 가진 칸은 장애물일 수 없음(생성 규칙) — 걸러진 로봇 점이 남은 것. 세기만 함
                held_cell_hits_++;
                continue;
            }
            seen.push_back(g);
        }
        r.scans++;
        confirmer_->observe(ts - t0_, seen, r.name);
    }

    void on_contact(Robot& r, const gazebo_msgs::msg::ContactsState& m) {
        if (!started_ || finished_ || m.states.empty()) return;
        r.contacts++;
        if (contact_examples_.size() < 10) {
            contact_examples_.push_back({{"t", rel_now()}, {"robot", r.name}, {"a", m.states.front().collision1_name},
                                         {"b", m.states.front().collision2_name}});
        }
    }

    // ── 장애물 ──
    bool cell_in_use(GridCell c) const {
        for (const auto& [id, r] : robots_) {
            if (r.at == c) return true;
            if (r.active >= 0) {
                const auto& A = adg_->actions()[r.active];
                if (A.from == c || A.to == c) return true;
            }
            if (r.queued >= 0 && adg_->actions()[r.queued].to == c) return true;
        }
        return false;
    }

    bool line_of_sight(const mapf_exec::Pose2& p, GridCell target) const {
        const auto c = mapf_exec::cell_center(target, height_, cell_);
        const double d = std::hypot(c.x - p.x, c.y - p.y);
        const int n = std::max(1, static_cast<int>(d / 0.05));
        for (int i = 1; i < n; ++i) {
            const GridCell g =
                mapf_exec::world_to_cell({p.x + (c.x - p.x) * i / n, p.y + (c.y - p.y) * i / n}, height_, cell_);
            if (g == target) continue;
            if (g.y < 0 || g.y >= height_ || g.x < 0 || g.x >= static_cast<int>(rows_[0].size())) return false;
            if (rows_[g.y][g.x] == '#') return false;
        }
        return true;
    }

    void update_obstacles() {
        const double t = rel_now();
        for (std::size_t k = 0; k < obstacles_.size(); ++k) {
            auto& o = obstacles_[k];
            if (!o.phantom && !o.spawn_requested && t >= o.spawn_at_s) {
                if (cell_in_use(o.cell)) continue;  // 로봇 위에 상자를 만들 수는 없음 → 비워질 때까지 미룸
                auto req = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
                req->name = "obstacle_" + std::to_string(k);
                req->xml =
                    "<?xml version='1.0'?><sdf version='1.6'><model name='box'><static>true</static><link name='l'>"
                    "<collision name='c'><geometry><box><size>0.4 0.4 0.4</size></box></geometry></collision>"
                    "<visual name='v'><geometry><box><size>0.4 0.4 0.4</size></box></geometry>"
                    "<material><ambient>0.8 0.1 0.1 1</ambient><diffuse>0.8 0.1 0.1 1</diffuse></material></visual>"
                    "</link></model></sdf>";
                const auto c = mapf_exec::cell_center(o.cell, height_, cell_);
                req->initial_pose.position.x = c.x;
                req->initial_pose.position.y = c.y;
                req->initial_pose.position.z = 0.2;
                spawn_cli_->async_send_request(req);
                o.spawn_requested = true;
                o.spawned_s = t;
                RCLCPP_INFO(get_logger(), "장애물 %zu 생성: %d행 %d열 (%.1f초)", k, o.cell.y, o.cell.x, t);
            }
            if (o.spawned_s >= 0 && o.sim_detect_s < 0) {  // 흉내 규칙(7-4). lidar 모드에서는 비교용 기록만
                for (auto& [id, r] : robots_) {
                    if (!r.pose) continue;
                    const auto c = mapf_exec::cell_center(o.cell, height_, cell_);
                    if (std::hypot(c.x - r.pose->x, c.y - r.pose->y) <= d_detect_ && line_of_sight(*r.pose, o.cell)) {
                        o.sim_detect_s = t;
                        o.sim_detected_by = r.name;
                        if (detection_ == "simulated") {
                            o.detected_s = o.first_detect_s = t;
                            o.detected_by = r.name;
                            RCLCPP_INFO(get_logger(), "장애물 %zu 발견: %s (%.1f초)", k, r.name.c_str(), t);
                        }
                        break;
                    }
                }
            }
            if (detection_ == "simulated" && o.detected_s >= 0 && o.confirmed_s < 0 && t >= o.detected_s + t_wait_ &&
                replanning_ < 0) {
                o.confirmed_s = t;
                replanning_ = static_cast<int>(k);
                RCLCPP_INFO(get_logger(), "장애물 %zu 확정(%.1f초): 새 동작 멈추고 이동 중인 로봇을 기다림", k, t);
            }
        }
        if (detection_ == "lidar") {
            process_scans();
            lidar_events(t);
        }
    }

    int find_obstacle(GridCell c, bool want_detected) const {
        for (std::size_t k = 0; k < obstacles_.size(); ++k) {
            const auto& o = obstacles_[k];
            if (o.cell == c && o.confirmed_s < 0 && (o.detected_s >= 0) == want_detected && (o.phantom || o.spawned_s >= 0)) {
                return static_cast<int>(k);
            }
        }
        return -1;
    }

    void lidar_events(double t) {
        confirmer_->set_hold_confirm(replanning_ >= 0);  // 다른 장애물 재계획 중이면 확정을 미룸
        for (const auto& ev : confirmer_->update(t)) {
            using K = mapf_exec::ObstacleConfirmer::Event::Kind;
            if (ev.kind == K::kDetected) {
                int k = find_obstacle(ev.cell, false);
                if (k < 0) {  // 시나리오에 없는(또는 아직 안 생긴) 칸 → 오탐 후보
                    Obstacle ph;
                    ph.cell = ev.cell;
                    ph.phantom = true;
                    obstacles_.push_back(ph);
                    k = static_cast<int>(obstacles_.size()) - 1;
                }
                auto& o = obstacles_[k];
                o.detected_s = ev.t;
                if (o.phantom) phantom_detections_++;
                if (o.first_detect_s < 0) o.first_detect_s = ev.t;
                o.detected_by = ev.by;
                RCLCPP_INFO(get_logger(), "장애물 %d 발견(LiDAR%s): %s, %d행 %d열 (%.1f초)", k, o.phantom ? ", 시나리오에 없음" : "",
                            ev.by.c_str(), ev.cell.y, ev.cell.x, ev.t);
                continue;
            }
            const int k = find_obstacle(ev.cell, true);
            if (k < 0) continue;
            auto& o = obstacles_[k];
            o.bins_seen = ev.bins_seen;
            o.bins_total = ev.bins_total;
            if (ev.kind == K::kDropped) {
                o.detected_s = -1;
                o.drops++;
                RCLCPP_INFO(get_logger(), "장애물 %d 후보 버림: %d/%d칸만 보임 (%.1f초)", k, ev.bins_seen, ev.bins_total, t);
            } else {
                o.confirmed_s = t;
                replanning_ = k;
                if (o.phantom) phantoms_confirmed_++;
                RCLCPP_INFO(get_logger(), "장애물 %d 확정(LiDAR %d/%d칸, %.1f초)%s: 새 동작 멈추고 이동 중인 로봇을 기다림", k,
                            ev.bins_seen, ev.bins_total, t, o.phantom ? " — 오탐!" : "");
                confirmer_->set_hold_confirm(true);  // 한 번에 하나씩
            }
        }
    }

    bool enters_detected_obstacle(const AdgAction& A) const {
        if (A.kind != AdgAction::Kind::kMove) return false;
        for (const auto& o : obstacles_) {
            if (o.detected_s >= 0 && o.cell == A.to) return true;
        }
        return false;
    }

    // ── 재계획 ──
    void do_replan() {
        auto& o = obstacles_[replanning_];
        const double t = rel_now();
        // ① 지금 모습: 칸, 하는 중인 작업의 남은 스텝(초 → 스텝은 Δt_nom으로, 올림)
        std::map<int, GridCell> current;
        std::map<int, int> inflight;
        for (auto& [id, r] : robots_) {
            current[id] = r.at;
            if (r.service_end_s >= 0) inflight[id] = std::max(1, static_cast<int>(std::ceil((r.service_end_s - t) / dt_nom_)));
        }
        // ② 번호 다시 붙이기(31.6, 31.17)
        mapf_exec::RetimeResult rt = mapf_exec::retime(*adg_, current, inflight);
        // ③ 코어 입력: 지도(앞서 재계획한 장애물은 막힌 칸), 로봇, 지금까지의 시간표, 근무표, 새 장애물
        mapf::Map map(rows_with_obstacles(false));
        std::vector<mapf::Agent> agents;
        mapf::PlanResult prev;
        mapf::TaskSchedule schedule;
        for (auto& [id, r] : robots_) {
            agents.push_back(mapf::Agent{id, {r.at.x, r.at.y}, {r.goal.x, r.goal.y}, r.service_steps});
            mapf::Path p;
            const auto& cells = rt.paths.at(id);
            for (std::size_t s = 0; s < cells.size(); ++s) p.push_back({cells[s].x, cells[s].y, static_cast<int>(s)});
            prev[id] = p;
            if (rt.service_start.count(id)) {
                schedule[id] = {rt.service_start.at(id), rt.service_start.at(id) + rt.service_steps.at(id)};
            } else {
                int arrive = static_cast<int>(cells.size()) - 1;
                while (arrive > 0 && cells[arrive - 1] == cells.back()) --arrive;
                schedule[id] = {arrive, arrive};  // 남은 작업 없음: 도착 전 = 이동, 도착 후 = 주차
            }
        }
        const std::vector<mapf::Cell> new_obs{{o.cell.x, o.cell.y}};
        mapf::ReplanConfig cfg;  // 우리 방법(CLAUDE.md): 고정 먼저, 도달 확인, 길 위 로봇 구조, Tier 2, 안전망 ② 3번
        cfg.order = mapf::ReplanOrder::kFixedFirst;
        cfg.check_reachability = true;
        cfg.rescue_selection = mapf::RescueSelection::kShortestPathConflicts;
        cfg.max_escalation_tiers = 2;
        cfg.order_restarts = 3;
        mapf::PrioritizedPlanner planner(map, mapf::AStarConfig{}, cfg);

        const auto c0 = std::chrono::steady_clock::now();
        std::optional<mapf::PlanResult> new_paths;
        mapf::TaskSchedule new_schedule;
        json info = json::object();
        if (method_ == "ours") {
            auto res = planner.replan(agents, prev, schedule, new_obs, 0);
            if (res) {
                new_paths = res->paths;
                new_schedule = res->schedule;
                info = {{"escalation_tier", res->escalation_tier}, {"order_restart", res->order_restart},
                        {"replanned_ids", res->replanned_ids}, {"rescued_ids", res->rescued_ids}};
            }
        } else {
            new_paths = planner.full_replan(agents, prev, schedule, new_obs, 0);
            if (new_paths) new_schedule = mapf::update_schedule(agents, schedule, *new_paths, 0);
        }
        const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - c0).count();

        o.event = {{"cell", {o.cell.x, o.cell.y}}, {"spawn_s", o.spawned_s}, {"detect_s", o.detected_s},
                   {"detected_by", o.detected_by}, {"confirm_s", o.confirmed_s}, {"replan_s", t},
                   {"detection", detection_}, {"sim_detect_s", o.sim_detect_s}, {"sim_detected_by", o.sim_detected_by},
                   {"first_detect_s", o.first_detect_s}, {"drops", o.drops}, {"bins_seen", o.bins_seen},
                   {"bins_total", o.bins_total}, {"phantom", o.phantom},
                   {"method", method_}, {"compute_ms", ms}, {"success", static_cast<bool>(new_paths)}};
        o.event.update(info);
        {  // ②~④ 입력·출력 저장(31.6): 같은 입력을 2D 파이썬(mapf_py)으로 다시 돌려 같은 결과인지 확인(check_replan_input)
            json in_agents = json::array(), in_prev = json::object(), in_sched = json::object(), out_paths = json::object();
            for (const auto& a : agents) {
                in_agents.push_back({{"id", a.id}, {"start", {a.start.x, a.start.y}}, {"goal", {a.goal.x, a.goal.y}},
                                     {"service_time", a.service_time}});
            }
            for (const auto& [id, p] : prev) {
                json q = json::array();
                for (const auto& c : p) q.push_back({c.x, c.y, c.t});
                in_prev[std::to_string(id)] = q;
            }
            for (const auto& [id, w] : schedule) in_sched[std::to_string(id)] = {w.start, w.end};
            if (new_paths) {
                for (const auto& [id, p] : *new_paths) {
                    json q = json::array();
                    for (const auto& c : p) q.push_back({c.x, c.y, c.t});
                    out_paths[std::to_string(id)] = q;
                }
            }
            const json dump = {{"format", "mapf_replan_input_v1"}, {"method", method_}, {"map", rows_with_obstacles(false)},
                               {"agents", in_agents}, {"previous_paths", in_prev}, {"previous_schedule", in_sched},
                               {"new_obstacles", {{o.cell.x, o.cell.y}}}, {"current_time", 0},
                               {"config", {{"order", "fixed_first"}, {"check_reachability", true},
                                           {"rescue_selection", "shortest_path_conflicts"}, {"max_escalation_tiers", 2},
                                           {"order_restarts", 3}}},
                               {"result", {{"success", static_cast<bool>(new_paths)}, {"paths", out_paths}}}};
            std::ofstream(out_dir_ + "/replan_" + std::to_string(replans_) + ".json") << dump.dump(1);
        }
        if (!new_paths) {
            abort_reason_ = "재계획 실패(" + method_ + ")";
            RCLCPP_ERROR(get_logger(), "%s", abort_reason_.c_str());
            stop(false);
            return;
        }
        // ④ 새 계획 검사 (가): 이번 장애물까지 막힌 지도로
        std::map<int, std::vector<GridCell>> np;
        for (const auto& [id, p] : *new_paths) {
            for (const auto& c : p) np[id].push_back({c.x, c.y});
        }
        const auto v = mapf_exec::check_plan(rows_with_obstacles(true), np);
        std::vector<std::string> changed;
        for (auto& [id, r] : robots_) {
            if (dedup(np.at(id)) != dedup(rt.paths.at(id))) changed.push_back(r.name);
        }
        o.event["changed_robots"] = changed;
        o.event["plan_check_violations"] = v.size();
        if (!v.empty()) {
            for (const auto& x : v) RCLCPP_ERROR(get_logger(), "새 계획 검사 실패: %s", x.describe().c_str());
            abort_reason_ = "새 계획 검사 실패";
            stop(false);
            return;
        }
        // ⑤ 새 ADG. 하는 중인 작업은 옛 ADG의 동작으로 끝까지 하고(legacy), 새 ADG에는 안 넣는다
        std::map<int, int> service, start;
        for (auto& [id, r] : robots_) {
            const auto& w = new_schedule.at(id);
            if (inflight.count(id) || w.end <= 0 || w.end - w.start <= 0) continue;
            service[id] = w.end - w.start;
            start[id] = w.start;
        }
        for (auto& [id, r] : robots_) {
            if (r.active >= 0) {  // 작업 동작만 남아 있을 수 있음(이동은 다 끝나기를 기다렸음)
                r.legacy_seq = r.active_seq;
                r.active = -1;
            }
        }
        int steps = 0;
        for (const auto& [id, p] : np) steps = std::max(steps, static_cast<int>(p.size()) - 1);
        o.event["new_plan_steps"] = steps;
        set_plan(np, service, start);
        o.replanned_s = t;
        replanning_ = -1;
        replans_++;
        char buf[64];
        std::snprintf(buf, sizeof(buf), "%s %.2f ms, changed %zu", method_.c_str(), ms, changed.size());
        last_replan_text_ = buf;
        RCLCPP_INFO(get_logger(), "재계획 완료(%s): %.2f ms, 길이 바뀐 로봇 %zu대", method_.c_str(), ms, changed.size());
    }

    // ── 동작 내주기 ──
    void release_ready() {
        if (replanning_ >= 0) return;  // 확정 뒤에는 새 동작 시작 금지
        for (int a : adg_->releasable()) {
            const auto& A = adg_->actions()[a];
            Robot& r = robots_.at(A.robot);
            if (r.legacy_seq) continue;                 // 재계획 전 작업이 아직 안 끝남
            if (enters_detected_obstacle(A)) continue;  // 장애물 앞 보류
            ActionCommand c;
            c.seq = r.next_seq++;
            c.cell_x = A.to.x;
            c.cell_y = A.to.y;
            const auto fa = mapf_exec::cell_center(A.from, height_, cell_), fb = mapf_exec::cell_center(A.to, height_, cell_);
            c.from_x = fa.x;
            c.from_y = fa.y;
            c.target_x = fb.x;
            c.target_y = fb.y;
            const double t = rel_now();
            if (A.kind == AdgAction::Kind::kMove) {
                c.kind = ActionCommand::MOVE;
            } else {
                c.kind = ActionCommand::WAIT;
                c.duration_s = A.service_steps * dt_nom_ * r.service_factor;  // 실제 작업 시간 = 계획 × 흔들림(31.3)
                r.service_end_s = t + c.duration_s;
            }
            r.adg_wait_s += t - r.last_done_s;
            adg_->release(a);
            r.active = a;
            r.active_seq = c.seq;
            r.pub->publish(c);
        }
        if (!pass_through_) return;
        // 곧게 지나가기: 이동 중인 로봇의 다음 같은 방향 이동을 미리 보냄(약속은 이미 다 지켜짐 — Adg::straight_next)
        for (auto& [id, r] : robots_) {
            if (r.active < 0 || r.queued >= 0 || r.legacy_seq) continue;
            const int b = adg_->straight_next(id);
            if (b < 0 || enters_detected_obstacle(adg_->actions()[b])) continue;
            const auto& B = adg_->actions()[b];
            ActionCommand c;
            c.seq = r.next_seq++;
            c.kind = ActionCommand::MOVE;
            c.cell_x = B.to.x;
            c.cell_y = B.to.y;
            const auto fa = mapf_exec::cell_center(B.from, height_, cell_), fb = mapf_exec::cell_center(B.to, height_, cell_);
            c.from_x = fa.x;
            c.from_y = fa.y;
            c.target_x = fb.x;
            c.target_y = fb.y;
            adg_->release(b);
            r.queued = b;
            r.queued_seq = c.seq;
            r.pub->publish(c);
        }
    }

    // 미리 보낸 동작의 칸에서 장애물이 발견되면 취소 요청(follower가 아직 시작 전이면 CANCELED로 답함).
    // 장애물이 확정되어 재계획을 기다리는 중이면 미리 보낸 동작을 모두 취소 → 로봇이 지금 목표 칸에서 바로 섬(31.6 ①을 빨리)
    void cancel_into_obstacles() {
        for (auto& [id, r] : robots_) {
            if (r.queued < 0 || r.cancel_sent) continue;
            if (replanning_ < 0 && !enters_detected_obstacle(adg_->actions()[r.queued])) continue;
            ActionCommand c;
            c.seq = r.queued_seq;
            c.kind = ActionCommand::CANCEL;
            r.pub->publish(c);
            r.cancel_sent = true;
            RCLCPP_INFO(get_logger(), "%s: 미리 보낸 동작 취소 요청(%s)", r.name.c_str(), replanning_ >= 0 ? "재계획 대기" : "장애물 칸");
        }
    }

    // ── 상태 ──
    void update_states(double dt) {
        for (auto& [id, r] : robots_) {
            std::string s = "IDLE", wait;
            int next = -1;
            for (int a : adg_->robot_actions(id)) {
                if (!adg_->is_done(a)) {
                    next = a;
                    break;
                }
            }
            if (r.estop) {
                s = "ESTOP";
            } else if (r.legacy_seq || (r.active >= 0 && adg_->actions()[r.active].kind == AdgAction::Kind::kService)) {
                s = "WORK";
            } else if (r.active >= 0) {
                s = "MOVE";
            } else if (next < 0) {
                s = "DONE";
            } else if (replanning_ >= 0) {
                s = "STOP";
            } else if (enters_detected_obstacle(adg_->actions()[next])) {
                s = "HOLD";
            } else {
                for (int p : adg_->cross_preds(next)) {
                    if (!adg_->is_done(p)) {
                        s = "YIELD";
                        wait = robots_.at(adg_->actions()[p].robot).name;
                        break;
                    }
                }
            }
            if (s == "DONE" && r.state != "DONE") r.finish_s = rel_now();
            if (s != "DONE") r.finish_s = -1;
            r.state = s;
            r.waiting_for = wait;
            r.state_time[s] += dt;
        }
    }

    void log_row() {
        const double t = now().seconds();
        if (t <= last_log_t_) return;  // 같은 시뮬레이터 시각은 한 번만(31.22)
        last_log_t_ = t;
        for (auto& [id, r] : robots_) {
            if (!r.pose) continue;
            std::string kind = "IDLE", held = cell_str(r.at);
            mapf_exec::Vec2 s = mapf_exec::cell_center(r.at, height_, cell_), g = s;
            if (r.active >= 0) {
                const auto& A = adg_->actions()[r.active];
                s = mapf_exec::cell_center(A.from, height_, cell_);
                g = mapf_exec::cell_center(A.to, height_, cell_);
                kind = A.kind == AdgAction::Kind::kMove ? "MOVE" : "WAIT";
                if (A.kind == AdgAction::Kind::kMove) held = cell_str(A.from) + ";" + cell_str(A.to);
            }
            csv_ << std::fixed;
            csv_.precision(3);
            csv_ << t << "," << r.name << ",";
            csv_.precision(5);
            csv_ << r.pose->x << "," << r.pose->y << "," << r.pose->yaw << "," << kind << "," << held << ",";
            csv_.precision(4);
            csv_ << s.x << "," << s.y << "," << g.x << "," << g.y << "," << r.state << "," << r.waiting_for << "\n";
        }
    }

    // ── RViz 마커(/mapf/markers, frame world) ──
    Marker base(const std::string& ns, int id, int type) {
        Marker m;
        m.header.frame_id = "world";
        m.header.stamp = now();
        m.ns = ns;
        m.id = id;
        m.type = type;
        m.action = Marker::ADD;
        m.pose.orientation.w = 1.0;
        m.lifetime = rclcpp::Duration::from_seconds(1.0);
        return m;
    }

    static geometry_msgs::msg::Point pt(mapf_exec::Vec2 v, double z) {
        geometry_msgs::msg::Point p;
        p.x = v.x;
        p.y = v.y;
        p.z = z;
        return p;
    }

    static void color(Marker& m, std::array<float, 3> c, float a) {
        m.color.r = c[0];
        m.color.g = c[1];
        m.color.b = c[2];
        m.color.a = a;
    }

    void publish_markers() {
        visualization_msgs::msg::MarkerArray arr;
        Marker sh = base("shelves", 0, Marker::CUBE_LIST);  // 선반
        sh.scale.x = sh.scale.y = cell_;
        sh.scale.z = 0.5;
        color(sh, {0.85f, 0.55f, 0.2f}, 0.6f);
        for (int y = 0; y < height_; ++y) {
            for (int x = 0; x < static_cast<int>(rows_[y].size()); ++x) {
                if (rows_[y][x] == '#') sh.points.push_back(pt(mapf_exec::cell_center({x, y}, height_, cell_), 0.25));
            }
        }
        arr.markers.push_back(sh);
        int k = 0;
        for (const auto& o : obstacles_) {  // 장애물: 생성만 됨(분홍) / 발견됨(빨강)
            if (o.spawned_s < 0 && !(o.phantom && (o.detected_s >= 0 || o.confirmed_s >= 0))) continue;
            Marker m = base("obstacles", k++, Marker::CUBE);
            m.pose.position = pt(mapf_exec::cell_center(o.cell, height_, cell_), 0.2);
            m.scale.x = m.scale.y = m.scale.z = 0.4;
            if (o.phantom) {  // 오탐(시나리오에 없는 칸): 노랑 반투명
                color(m, {1.0f, 0.9f, 0.1f}, 0.5f);
            } else {
                color(m, o.detected_s < 0 && o.confirmed_s < 0 ? std::array<float, 3>{0.9f, 0.5f, 0.6f}
                                                                : std::array<float, 3>{0.9f, 0.05f, 0.05f},
                      0.9f);
            }
            arr.markers.push_back(m);
        }
        for (auto& [id, r] : robots_) {
            if (!r.pose) continue;
            const auto rc = robot_color(id);
            Marker held = base("held", id, Marker::CUBE_LIST);  // 가진 칸
            held.scale.x = held.scale.y = cell_ * 0.95;
            held.scale.z = 0.01;
            color(held, rc, 0.25f);
            held.points.push_back(pt(mapf_exec::cell_center(r.at, height_, cell_), 0.005));
            if (r.active >= 0 && adg_->actions()[r.active].kind == AdgAction::Kind::kMove) {
                held.points.push_back(pt(mapf_exec::cell_center(adg_->actions()[r.active].to, height_, cell_), 0.005));
            }
            arr.markers.push_back(held);
            Marker path = base("path", id, Marker::LINE_STRIP);  // 남은 경로
            path.scale.x = 0.03;
            color(path, rc, 0.9f);
            path.points.push_back(pt({r.pose->x, r.pose->y}, 0.02));
            for (int a : adg_->robot_actions(id)) {
                if (!adg_->is_done(a) && adg_->actions()[a].kind == AdgAction::Kind::kMove) {
                    path.points.push_back(pt(mapf_exec::cell_center(adg_->actions()[a].to, height_, cell_), 0.02));
                }
            }
            if (path.points.size() >= 2) arr.markers.push_back(path);
            Marker disc = base("state", id, Marker::CYLINDER);  // 상태 원판
            disc.pose.position = pt({r.pose->x, r.pose->y}, 0.3);
            disc.scale.x = disc.scale.y = 0.22;
            disc.scale.z = 0.03;
            color(disc, state_color(r.state), 1.0f);
            arr.markers.push_back(disc);
            Marker txt = base("label", id, Marker::TEXT_VIEW_FACING);  // 글자(RViz 글꼴에 한글이 없어 영어 약자)
            txt.pose.position = pt({r.pose->x, r.pose->y}, 0.5);
            txt.scale.z = 0.12;
            color(txt, {1.0f, 1.0f, 1.0f}, 1.0f);
            std::string label = "R" + std::to_string(id + 1) + " " + r.state;
            if (r.state == "WORK" && r.service_end_s >= 0) label += " " + fmt1(std::max(0.0, r.service_end_s - rel_now())) + "s";
            if (r.state == "YIELD") label += ">R" + r.waiting_for.substr(r.waiting_for.find('_') + 1);
            txt.text = label;
            arr.markers.push_back(txt);
            if (r.state == "YIELD") {  // 양보: 기다리는 로봇 → 기다리게 하는 로봇
                for (auto& [oid, other] : robots_) {
                    if (other.name != r.waiting_for || !other.pose) continue;
                    Marker ar = base("wait", id, Marker::ARROW);
                    ar.points.push_back(pt({r.pose->x, r.pose->y}, 0.3));
                    ar.points.push_back(pt({other.pose->x, other.pose->y}, 0.3));
                    ar.scale.x = 0.02;
                    ar.scale.y = 0.06;
                    ar.scale.z = 0.08;
                    color(ar, {1.0f, 0.6f, 0.0f}, 1.0f);
                    arr.markers.push_back(ar);
                }
            }
        }
        Marker info = base("info", 0, Marker::TEXT_VIEW_FACING);  // 맨 위 정보 줄
        info.pose.position = pt({rows_[0].size() * cell_ / 2.0, height_ * cell_ + 0.4}, 0.5);
        info.scale.z = 0.18;
        color(info, {1.0f, 1.0f, 1.0f}, 1.0f);
        info.text = "t=" + fmt1(started_ ? rel_now() : 0.0) + "s  method=" + method_ + "  replans=" + std::to_string(replans_) +
                    (last_replan_text_.empty() ? "" : "  last: " + last_replan_text_);
        arr.markers.push_back(info);
        marker_pub_->publish(arr);
    }

    void tick() {
        if (finished_) return;
        if (!started_) {
            if (!violations_.empty()) {
                abort_reason_ = "계획 검사 (가) 실패 " + std::to_string(violations_.size()) + "건 — 실행 안 함";
                stop(false);
                return;
            }
            if (!ready()) {
                if (std::chrono::duration<double>(std::chrono::steady_clock::now() - wall_start_).count() > start_wait_s_) {
                    abort_reason_ = "준비 안 됨(odom 또는 cell_follower 없음)";
                    stop(false);
                }
                return;
            }
            started_ = true;
            t0_ = now().seconds();
            RCLCPP_INFO(get_logger(), "실행 시작");
        }
        const double t = rel_now();
        update_obstacles();
        cancel_into_obstacles();
        if (replanning_ >= 0) {
            bool moving = false;
            for (auto& [id, r] : robots_) {
                moving = moving || (r.active >= 0 && adg_->actions()[r.active].kind == AdgAction::Kind::kMove);
            }
            if (!moving) do_replan();
            if (finished_) return;
        }
        release_ready();
        update_states(std::max(0.0, t - last_tick_));
        last_tick_ = t;
        log_row();
        if (++marker_tick_ % 2 == 0) publish_markers();
        bool all_done = adg_->finished();
        for (auto& [id, r] : robots_) all_done = all_done && !r.legacy_seq;
        // 생성됐는데 아직 발견·재계획이 안 끝난 장애물이 있으면 기다림(발견 못 한 장애물은 기다리지 않음)
        for (const auto& o : obstacles_) all_done = all_done && !(o.detected_s >= 0 && o.replanned_s < 0);
        if (all_done) {
            stop(true);
        } else if (t > total_timeout_s_) {
            abort_reason_ = "전체 시간 초과";
            stop(false);
        }
    }

    void stop(bool all_done) {
        finished_ = true;
        timer_->cancel();
        csv_.close();
        publish_markers();
        // 칸 사용 순서: 실제로 들어가기를 끝낸 순서가 계획 순서(재계획마다 새 계획을 이어 붙임)의 부분열인가.
        // 재계획으로 버려진 옛 계획의 남은 동작은 실제로 안 일어나므로 "같다" 대신 "부분열"로 본다.
        json mismatches = json::array();
        for (auto& [c, list] : planned_order_) {
            std::sort(list.begin(), list.end());
            auto actual = entered_[c];
            std::sort(actual.begin(), actual.end());
            std::vector<int> p, q;
            for (auto& x : list) p.push_back(x.second);
            for (auto& x : actual) q.push_back(x.second);
            std::size_t i = 0;
            for (int x : p) {
                if (i < q.size() && q[i] == x) ++i;
            }
            if (i != q.size()) mismatches.push_back({{"cell", c}, {"plan", p}, {"actual", q}});
        }
        json robots = json::object();
        double makespan = 0.0;
        long contacts = 0;
        int estops = 0;
        for (auto& [id, r] : robots_) {
            robots[r.name] = {{"finish_s", r.finish_s}, {"adg_wait_s", r.adg_wait_s}, {"max_pos_err_m", r.max_pos_err},
                              {"contacts", r.contacts}, {"state_time_s", r.state_time}, {"estop", r.estop},
                              {"passes", r.passes}, {"cancels", r.cancels}, {"scans", r.scans}};
            makespan = std::max(makespan, r.finish_s);
            contacts += r.contacts;
            estops += r.estop ? 1 : 0;
        }
        json obs = json::array();
        for (const auto& o : obstacles_) {
            obs.push_back(o.event.empty() ? json{{"cell", {o.cell.x, o.cell.y}}, {"spawn_s", o.spawned_s}, {"detect_s", o.detected_s},
                                                 {"sim_detect_s", o.sim_detect_s}, {"first_detect_s", o.first_detect_s},
                                                 {"drops", o.drops}, {"phantom", o.phantom}, {"detection", detection_}}
                                          : o.event);
        }
        ok_ = all_done && violations_.empty() && contacts == 0 && mismatches.empty() && estops == 0 && late_cancels_ == 0 &&
              phantoms_confirmed_ == 0;
        json out = {{"scenario", sc_.at("name")}, {"method", method_}, {"all_done", all_done}, {"abort_reason", abort_reason_},
                    {"plan_check_violations", violations_.size()}, {"makespan_s", makespan},
                    {"plan_makespan_steps", initial_plan_steps_}, {"plan_makespan_s_at_dt_nom", initial_plan_steps_ * dt_nom_},
                    {"contacts", contacts}, {"contact_examples", contact_examples_}, {"estops", estops},
                    {"order_mismatches", mismatches}, {"replans", replans_}, {"obstacles", obs}, {"d_detect", d_detect_},
                    {"t_wait", t_wait_}, {"pass_through", pass_through_}, {"late_cancels", late_cancels_},
                    {"detection", detection_}, {"phantoms_confirmed", phantoms_confirmed_}, {"held_cell_hits", held_cell_hits_},
                    {"late_scans", late_scans_}, {"phantom_detections", phantom_detections_},
                    {"cross_edges", adg_->num_cross_edges()}, {"robots", robots},
                    {"action_log", action_log_}};
        std::ofstream(out_dir_ + "/executive.json") << out.dump(1);
        RCLCPP_INFO(get_logger(), "%s: 다 끝난 시간 %.1f초, 재계획 %d번, 접촉 %ld, 순서 불일치 %zu",
                    all_done ? "모두 완료" : ("중단: " + abort_reason_).c_str(), makespan, replans_, contacts, mismatches.size());
    }

    json sc_;
    std::vector<std::string> rows_;
    std::string out_dir_, abort_reason_, method_, last_replan_text_;
    double cell_{0.5}, dt_nom_{3.5}, d_detect_{2.0}, t_wait_{3.0}, start_wait_s_{60}, total_timeout_s_{1800};
    double t0_{0}, last_log_t_{-1.0}, last_tick_{0.0};
    int height_{0}, initial_plan_steps_{0}, replanning_{-1}, replans_{0}, plan_epoch_{0};
    long marker_tick_{0};
    bool drop_{false}, started_{false}, finished_{false}, ok_{false}, pass_through_{false};
    int late_cancels_{0}, phantoms_confirmed_{0};
    long held_cell_hits_{0}, late_scans_{0}, phantom_detections_{0};
    std::deque<std::pair<Robot*, sensor_msgs::msg::LaserScan::ConstSharedPtr>> pending_scans_;
    std::string detection_{"simulated"};
    mapf_exec::LidarParams lidar_;
    std::unique_ptr<mapf_exec::ObstacleConfirmer> confirmer_;
    std::map<int, Robot> robots_;
    std::vector<Obstacle> obstacles_;
    std::vector<mapf_exec::Violation> violations_;
    std::unique_ptr<mapf_exec::Adg> adg_;
    std::map<std::string, std::vector<std::pair<long, int>>> planned_order_;
    std::map<std::string, std::vector<std::pair<double, int>>> entered_;
    json action_log_ = json::array(), contact_examples_ = json::array();
    std::ofstream csv_;
    std::chrono::steady_clock::time_point wall_start_;
    rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr spawn_cli_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapfExecutive>();
    rclcpp::executors::SingleThreadedExecutor ex;
    ex.add_node(node);
    while (rclcpp::ok() && !node->finished()) ex.spin_once(std::chrono::milliseconds(50));
    const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(300);  // 마지막 마커가 나가도록
    while (rclcpp::ok() && std::chrono::steady_clock::now() < end) ex.spin_once(std::chrono::milliseconds(50));
    const int code = node->exit_code();
    rclcpp::shutdown();
    return code;
}

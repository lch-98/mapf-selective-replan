// mapf_executive — 저장된 처음 계획을 ADG로 여러 대에게 실행시킨다 (docs/31 31.4, 31.22, 7-3).
//
//   ros2 run mapf_ros mapf_executive --ros-args -p use_sim_time:=true -p scenario:=<시나리오.json> -p out_dir:=<폴더>
//
// 1. 시나리오 JSON의 initial_plan(make_scenario가 코어 plan()으로 미리 만들어 저장한 것)을 읽는다.
//    실행 중에는 plan()을 부르지 않는다 — PP는 처음 계획부터 실패할 수 있어서, 성공한 계획만 시나리오에 저장해 둔다.
//    우리 연구도 "처음 계획은 무엇으로 만들었든 로봇별 경로만 받는다"(26.7)이므로 같은 설정이다.
// 2. 계획 검사 (가)(mapf_exec::check_plan)를 통과해야만 실행한다.
// 3. ADG(mapf_exec::Adg)를 만들고, 내줄 수 있는 동작을 로봇마다 /robot_i/mapf/action으로 보낸다. DONE을 받으면 다음 계산.
// 4. 기록(out_dir): run.csv(같은 시각마다 모든 로봇 자세, analyze_run 형식), map.txt, executive.json(동작 시각, ADG 대기,
//    접촉 센서, 칸 사용 순서 비교, 계획 vs 실제 시간).
// debug_drop_cross_edges=true는 변이 검사용(로봇 사이 약속을 뺌 → 부딪혀야 정상). 실험에 쓰지 말 것.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "gazebo_msgs/msg/contacts_state.hpp"
#include "mapf_exec/adg.hpp"
#include "mapf_exec/geometry.hpp"
#include "mapf_exec/plan_check.hpp"
#include "mapf_ros_msgs/msg/action_command.hpp"
#include "mapf_ros_msgs/msg/action_status.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

using json = nlohmann::json;
using mapf_exec::AdgAction;
using mapf_exec::GridCell;
using mapf_ros_msgs::msg::ActionCommand;
using mapf_ros_msgs::msg::ActionStatus;

namespace {

struct Robot {
    int id{0};
    std::string name;
    int service_steps{0};
    double service_factor{1.0};
    rclcpp::Publisher<ActionCommand>::SharedPtr pub;
    rclcpp::Subscription<ActionStatus>::SharedPtr status_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<gazebo_msgs::msg::ContactsState>::SharedPtr bump_sub;
    std::optional<mapf_exec::Pose2> pose;
    // 지금 하는 동작(기록용)
    int active{-1};
    GridCell at{};
    double last_done_s{0.0};
    double adg_wait_s{0.0};
    double finish_s{-1.0};
    double max_pos_err{0.0};
    long contacts{0};
};

std::string cell_str(GridCell c) { return std::to_string(c.x) + ":" + std::to_string(c.y); }

}  // namespace

class MapfExecutive : public rclcpp::Node {
public:
    MapfExecutive() : Node("mapf_executive") {
        const std::string scenario_path = declare_parameter("scenario", std::string());
        out_dir_ = declare_parameter("out_dir", std::string());
        const bool drop = declare_parameter("debug_drop_cross_edges", false);
        start_wait_s_ = declare_parameter("start_wait_s", 60.0);
        total_timeout_s_ = declare_parameter("total_timeout_s", 1800.0);
        const double rate = declare_parameter("rate_hz", 20.0);
        if (scenario_path.empty() || out_dir_.empty()) throw std::runtime_error("scenario와 out_dir 파라미터가 필요함");
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

        std::map<int, int> service;
        for (const auto& r : sc_.at("robots")) {
            Robot rb;
            rb.id = r.at("id");
            rb.name = r.at("name");
            rb.service_steps = r.value("service_steps", 0);
            rb.service_factor = r.value("service_factor", 1.0);
            const auto& p = sc_["initial_plan"].at(std::to_string(rb.id));
            std::vector<GridCell> path;
            for (std::size_t t = 0; t < p.size(); ++t) {
                if (p[t].at(2).get<int>() != static_cast<int>(t)) throw std::runtime_error(rb.name + ": 계획 번호가 0부터 1씩이 아님");
                path.push_back({p[t].at(0).get<int>(), p[t].at(1).get<int>()});
            }
            rb.at = path.front();
            paths_[rb.id] = path;
            service[rb.id] = rb.service_steps;
            robots_[rb.id] = std::move(rb);
        }

        // 계획 검사 (가)
        violations_ = mapf_exec::check_plan(rows_, paths_);
        for (const auto& v : violations_) RCLCPP_ERROR(get_logger(), "계획 검사 실패: %s", v.describe().c_str());

        adg_ = std::make_unique<mapf_exec::Adg>(paths_, service, !drop);
        if (drop) RCLCPP_WARN(get_logger(), "변이 검사용: 로봇 사이 약속을 모두 뺐다(부딪혀야 정상)");
        RCLCPP_INFO(get_logger(), "시나리오 %s: 로봇 %zu대, 동작 %zu개, 로봇 사이 약속 %d개", sc_.at("name").get<std::string>().c_str(),
                    robots_.size(), adg_->actions().size(), adg_->num_cross_edges());

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
                });
            rb.bump_sub = create_subscription<gazebo_msgs::msg::ContactsState>(
                "/" + rb.name + "/bumper_states", rclcpp::SensorDataQoS(),
                [this, r](gazebo_msgs::msg::ContactsState::ConstSharedPtr m) { on_contact(*r, *m); });
        }

        csv_.open(out_dir_ + "/run.csv");
        csv_ << "t,robot,x,y,yaw,kind,held,seg_sx,seg_sy,seg_gx,seg_gy\n";
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
    double now_s() { return now().seconds(); }

    bool ready() {
        for (auto& [id, rb] : robots_) {
            if (!rb.pose || rb.pub->get_subscription_count() == 0) return false;
        }
        return true;
    }

    void on_status(Robot& r, const ActionStatus& m) {
        if (!started_ || r.active < 0) return;
        const auto& acts = adg_->robot_actions(r.id);
        if (m.seq >= acts.size() || acts[m.seq] != r.active || m.state == ActionStatus::ACTIVE) return;
        const int a = r.active;
        const auto& A = adg_->actions()[a];
        const double t = now_s() - t0_;
        if (m.state == ActionStatus::DONE) {
            adg_->done(a);
            r.active = -1;
            r.at = A.to;
            r.last_done_s = t;
            r.max_pos_err = std::max(r.max_pos_err, static_cast<double>(m.pos_err_m));
            if (A.kind == AdgAction::Kind::kMove) entered_[cell_str(A.to)].push_back({t, r.id});
            action_log_.push_back({{"robot", r.name}, {"index", A.index}, {"kind", A.kind == AdgAction::Kind::kMove ? "MOVE" : "SERVICE"},
                                   {"to", {A.to.x, A.to.y}}, {"plan_t", A.plan_t}, {"done_s", t}, {"pos_err_m", m.pos_err_m}});
            if (acts.back() == a) r.finish_s = t;
        } else {
            abort_reason_ = r.name + (m.state == ActionStatus::ESTOP ? " 비상 정지" : " 보류(HELD)") + " — 동작 " +
                            std::to_string(A.index);
            RCLCPP_ERROR(get_logger(), "%s", abort_reason_.c_str());
            stop(false);
        }
    }

    void on_contact(Robot& r, const gazebo_msgs::msg::ContactsState& m) {
        if (!started_ || finished_ || m.states.empty()) return;
        r.contacts++;
        if (contact_examples_.size() < 10) {
            contact_examples_.push_back({{"t", now_s() - t0_}, {"robot", r.name},
                                         {"a", m.states.front().collision1_name}, {"b", m.states.front().collision2_name}});
        }
    }

    void release_ready() {
        for (int a : adg_->releasable()) {
            const auto& A = adg_->actions()[a];
            Robot& r = robots_.at(A.robot);
            ActionCommand c;
            c.seq = static_cast<uint32_t>(A.index);
            c.cell_x = A.to.x;
            c.cell_y = A.to.y;
            const auto fa = mapf_exec::cell_center(A.from, height_, cell_), fb = mapf_exec::cell_center(A.to, height_, cell_);
            c.from_x = fa.x;
            c.from_y = fa.y;
            c.target_x = fb.x;
            c.target_y = fb.y;
            if (A.kind == AdgAction::Kind::kMove) {
                c.kind = ActionCommand::MOVE;
            } else {
                c.kind = ActionCommand::WAIT;
                c.duration_s = A.service_steps * dt_nom_ * r.service_factor;  // 실제 작업 시간 = 계획 × 흔들림(31.3)
            }
            const double t = now_s() - t0_;
            r.adg_wait_s += t - r.last_done_s;  // 앞 동작이 끝난 뒤 약속 때문에 기다린 시간
            adg_->release(a);
            r.active = a;
            r.pub->publish(c);
        }
    }

    void log_row() {
        const double t = now_s();
        // 시뮬레이터 시계가 타이머보다 느리게 갱신되면 같은 시각에 타이머가 두 번 돈다 → 같은 시각은 한 번만 기록
        if (t <= last_log_t_) return;
        last_log_t_ = t;
        for (auto& [id, r] : robots_) {
            if (!r.pose) continue;
            std::string kind = "IDLE", held = cell_str(r.at);
            mapf_exec::Vec2 s = mapf_exec::cell_center(r.at, height_, cell_), g = s;
            if (r.active >= 0) {
                const auto& A = adg_->actions()[r.active];
                s = mapf_exec::cell_center(A.from, height_, cell_);
                g = mapf_exec::cell_center(A.to, height_, cell_);
                if (A.kind == AdgAction::Kind::kMove) {
                    kind = "MOVE";
                    held = cell_str(A.from) + ";" + cell_str(A.to);
                } else {
                    kind = "WAIT";
                }
            }
            csv_ << std::fixed;
            csv_.precision(3);
            csv_ << t << "," << r.name << ",";
            csv_.precision(5);
            csv_ << r.pose->x << "," << r.pose->y << "," << r.pose->yaw << "," << kind << "," << held << ",";
            csv_.precision(4);
            csv_ << s.x << "," << s.y << "," << g.x << "," << g.y << "\n";
        }
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
            t0_ = now_s();
            RCLCPP_INFO(get_logger(), "실행 시작");
        }
        release_ready();
        log_row();
        if (adg_->finished()) {
            stop(true);
        } else if (now_s() - t0_ > total_timeout_s_) {
            abort_reason_ = "전체 시간 초과";
            stop(false);
        }
    }

    void stop(bool all_done) {
        finished_ = true;
        timer_->cancel();
        csv_.close();
        // 칸 사용 순서: 계획에서 칸에 들어간 순서(계획 시각) vs 실제로 들어가기를 끝낸 순서(DONE 시각)
        std::map<std::string, std::vector<std::pair<int, int>>> planned;
        for (const auto& A : adg_->actions()) {
            if (A.kind == AdgAction::Kind::kMove) planned[cell_str(A.to)].push_back({A.plan_t, A.robot});
        }
        json mismatches = json::array();
        for (auto& [c, list] : planned) {
            std::sort(list.begin(), list.end());
            auto actual = entered_[c];
            std::sort(actual.begin(), actual.end());
            std::vector<int> p, q;
            for (auto& x : list) p.push_back(x.second);
            for (auto& x : actual) q.push_back(x.second);
            // 다 끝나지 않았으면 실제 쪽이 짧다 → 앞부분만 비교
            p.resize(std::min(p.size(), q.size()));
            if (p != q) mismatches.push_back({{"cell", c}, {"plan", p}, {"actual", q}});
        }
        int plan_makespan = 0;
        for (const auto& [id, path] : paths_) plan_makespan = std::max(plan_makespan, static_cast<int>(path.size()) - 1);
        json robots = json::object();
        double makespan = 0.0;
        long contacts = 0;
        int estops = abort_reason_.find("비상 정지") != std::string::npos ? 1 : 0;
        for (auto& [id, r] : robots_) {
            robots[r.name] = {{"finish_s", r.finish_s}, {"adg_wait_s", r.adg_wait_s}, {"max_pos_err_m", r.max_pos_err},
                              {"contacts", r.contacts}, {"actions", adg_->robot_actions(id).size()},
                              {"plan_steps", paths_.at(id).size() - 1}};
            makespan = std::max(makespan, r.finish_s);
            contacts += r.contacts;
        }
        ok_ = all_done && violations_.empty() && contacts == 0 && mismatches.empty();
        json out = {
            {"scenario", sc_.at("name")}, {"all_done", all_done}, {"abort_reason", abort_reason_},
            {"plan_check_violations", violations_.size()}, {"actions", adg_->actions().size()},
            {"cross_edges", adg_->num_cross_edges()}, {"makespan_s", makespan}, {"plan_makespan_steps", plan_makespan},
            {"plan_makespan_s_at_dt_nom", plan_makespan * dt_nom_}, {"contacts", contacts},
            {"contact_examples", contact_examples_}, {"estops", estops}, {"order_mismatches", mismatches},
            {"robots", robots}, {"action_log", action_log_}};
        std::ofstream(out_dir_ + "/executive.json") << out.dump(1);
        RCLCPP_INFO(get_logger(), "%s: 다 끝난 시간 %.1f초(계획 %d스텝), 접촉 %ld, 순서 불일치 %zu",
                    all_done ? "모두 완료" : ("중단: " + abort_reason_).c_str(), makespan, plan_makespan, contacts,
                    mismatches.size());
    }

    json sc_;
    std::vector<std::string> rows_;
    double cell_{0.5}, dt_nom_{3.5}, start_wait_s_{60}, total_timeout_s_{1800}, t0_{0}, last_log_t_{-1.0};
    int height_{0};
    std::string out_dir_, abort_reason_;
    std::map<int, Robot> robots_;
    std::map<int, std::vector<GridCell>> paths_;
    std::vector<mapf_exec::Violation> violations_;
    std::unique_ptr<mapf_exec::Adg> adg_;
    std::map<std::string, std::vector<std::pair<double, int>>> entered_;
    json action_log_ = json::array(), contact_examples_ = json::array();
    std::ofstream csv_;
    std::chrono::steady_clock::time_point wall_start_;
    bool started_{false}, finished_{false}, ok_{false};
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapfExecutive>();
    rclcpp::executors::SingleThreadedExecutor ex;
    ex.add_node(node);
    while (rclcpp::ok() && !node->finished()) ex.spin_once(std::chrono::milliseconds(50));
    const int code = node->exit_code();
    rclcpp::shutdown();
    return code;
}

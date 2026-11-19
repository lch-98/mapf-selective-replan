// ─────────────────────────────────────────────────────────────────
// bindings/mapf_bindings.cpp
//
// C++ 코어(mapf::Cell/Agent/Map/PrioritizedPlanner 등)를 파이썬 확장 모듈 mapf_py로
// 그대로 노출한다. GUI가 순수 파이썬으로 알고리즘을 다시 짜지 않고,
// 실제로 컴파일된 C++ PrioritizedPlanner::plan/replan/full_replan을 직접 호출하게
// 하기 위한 얇은 바인딩 계층이다.
//
// 바인딩 순서가 곧 의존 순서다 — pybind11은 어떤 타입을 시그니처에서
// 참조하려면 그 타입이 먼저 등록되어 있어야 한다(Agent가 Cell을 쓰고,
// PrioritizedPlanner 생성자가 Map/AStarConfig/ReplanConfig를 쓰므로).
//
// PlanResult(= std::unordered_map<int, Path>)는 별도 바인딩이 필요 없다 —
// SpaceTimeCell만 등록되면 pybind11/stl.h의 제네릭 map/vector caster가
// dict[int, list[SpaceTimeCell]]로 자동 변환해준다. "바인딩된 PlanResult
// 클래스"는 따로 존재하지 않는다.
// ─────────────────────────────────────────────────────────────────
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "mapf/agent.hpp"
#include "mapf/map.hpp"
#include "mapf/prioritized_planner.hpp"
#include "mapf/space_time_astar.hpp"
#include "mapf/task_schedule.hpp"

namespace py = pybind11;
using namespace mapf;

PYBIND11_MODULE(mapf_py, m) {
    m.doc() = "mapf C++ 코어(Map, Agent, PrioritizedPlanner)를 그대로 노출하는 pybind11 바인딩";

    // ── Cell ────────────────────────────────────────────────────
    // C++에는 해시 함수가 없다(std::map/set 키로만 쓰였으므로 operator<로
    // 충분했음). 파이썬 GUI에서는 "장애물 칸 집합"을 set()/dict 키로
    // 다루고 싶으므로, __hash__는 C++ API에 없는 것을 GUI 편의를 위해
    // 여기서 새로 추가하는 것이다(라이브러리 자체 기능이 아님).
    py::class_<Cell>(m, "Cell")
        .def(py::init<>())
        .def(py::init<int, int>(), py::arg("x"), py::arg("y"))
        .def_readwrite("x", &Cell::x)
        .def_readwrite("y", &Cell::y)
        .def(py::self == py::self)
        .def(py::self < py::self)
        .def("__hash__", [](const Cell& c) { return py::hash(py::make_tuple(c.x, c.y)); })
        .def("__repr__", [](const Cell& c) {
            return "Cell(x=" + std::to_string(c.x) + ", y=" + std::to_string(c.y) + ")";
        });

    // ── SpaceTimeCell ───────────────────────────────────────────
    py::class_<SpaceTimeCell>(m, "SpaceTimeCell")
        .def(py::init<>())
        .def(py::init<int, int, int>(), py::arg("x"), py::arg("y"), py::arg("t"))
        .def_readwrite("x", &SpaceTimeCell::x)
        .def_readwrite("y", &SpaceTimeCell::y)
        .def_readwrite("t", &SpaceTimeCell::t)
        .def(py::self == py::self)
        .def(py::self < py::self)
        .def("__hash__",
             [](const SpaceTimeCell& c) { return py::hash(py::make_tuple(c.x, c.y, c.t)); })
        .def("__repr__", [](const SpaceTimeCell& c) {
            return "SpaceTimeCell(x=" + std::to_string(c.x) + ", y=" + std::to_string(c.y) +
                   ", t=" + std::to_string(c.t) + ")";
        });

    // ── Agent ───────────────────────────────────────────────────
    // Agent(id, start, goal, service_time=0) 생성자를 바인딩해서, tools/benchmark.cpp의
    // Agent{i, shuffled_starts[i], shuffled_goals[i]} 패턴을 파이썬에서
    // 그대로 재현할 수 있게 한다. 우선순위는 이 객체들을 담은 리스트의
    // 순서로 정해진다(앞에 올수록 우선순위 높음) — 필드로 따로 없음.
    // service_time은 목적지에 도착한 뒤 작업하는 스텝 수다(17장). 생략하면 0이라 예전과 같다.
    py::class_<Agent>(m, "Agent")
        .def(py::init<>())
        .def(py::init<int, Cell, Cell, int>(), py::arg("id"), py::arg("start"), py::arg("goal"),
             py::arg("service_time") = 0)
        .def_readwrite("id", &Agent::id)
        .def_readwrite("start", &Agent::start)
        .def_readwrite("goal", &Agent::goal)
        .def_readwrite("service_time", &Agent::service_time)
        .def("__repr__", [](const Agent& a) {
            return "Agent(id=" + std::to_string(a.id) + ", start=Cell(" +
                   std::to_string(a.start.x) + "," + std::to_string(a.start.y) + "), goal=Cell(" +
                   std::to_string(a.goal.x) + "," + std::to_string(a.goal.y) +
                   "), service_time=" + std::to_string(a.service_time) + ")";
        });

    // ── Map ─────────────────────────────────────────────────────
    // 두 생성자 모두 바인딩한다: Map(width, height)와 Map(rows). rows는
    // list[str]을 넘기면 pybind11/stl.h가 std::vector<std::string>으로
    // 자동 변환한다. 두 생성자 모두 std::invalid_argument를 던질 수 있는데,
    // pybind11은 이걸 자동으로 파이썬 ValueError로 변환해준다(별도 예외
    // 변환기를 등록할 필요 없음 — smoke_test.py에서 직접 확인한다).
    py::class_<Map>(m, "Map")
        .def(py::init<int, int>(), py::arg("width"), py::arg("height"))
        .def(py::init<const std::vector<std::string>&>(), py::arg("rows"))
        .def("width", &Map::width)
        .def("height", &Map::height)
        .def("is_passable", &Map::is_passable, py::arg("x"), py::arg("y"))
        .def("neighbors", &Map::neighbors, py::arg("cell"))
        .def("set_obstacle", &Map::set_obstacle, py::arg("x"), py::arg("y"));

    // ── AStarConfig / ReplanConfig ─────────────────────────────────
    py::enum_<TieBreak>(m, "TieBreak")
        .value("LARGER_G", TieBreak::kLargerG)
        .value("SMALLER_G", TieBreak::kSmallerG)
        .value("RANDOM", TieBreak::kRandom);

    py::class_<AStarConfig>(m, "AStarConfig")
        .def(py::init<>())
        .def_readwrite("max_timestep", &AStarConfig::max_timestep)
        .def_readwrite("tie_break", &AStarConfig::tie_break)
        .def_readwrite("tie_seed", &AStarConfig::tie_seed);

    // 재계획 스위치(12·13·19장). 21장 5-1에서 전부 노출 — 전에는 max_escalation_tiers만 있어서
    // 파이썬(GUI)에서 "우리 방법"(고정 먼저 + 도달 확인 + 경로 구조)을 고를 수 없었다.
    py::enum_<ReplanOrder>(m, "ReplanOrder")
        .value("PRIORITY", ReplanOrder::kPriority)
        .value("FIXED_FIRST", ReplanOrder::kFixedFirst);

    py::enum_<RescueSelection>(m, "RescueSelection")
        .value("BLOCKED_CELLS", RescueSelection::kBlockedCells)
        .value("SHORTEST_PATH_CONFLICTS", RescueSelection::kShortestPathConflicts);

    py::class_<ReplanConfig>(m, "ReplanConfig")
        .def(py::init<>())
        .def_readwrite("max_escalation_tiers", &ReplanConfig::max_escalation_tiers)
        .def_readwrite("order", &ReplanConfig::order)
        .def_readwrite("check_reachability", &ReplanConfig::check_reachability)
        .def_readwrite("rescue_selection", &ReplanConfig::rescue_selection)
        .def_readwrite("request_yield", &ReplanConfig::request_yield)
        .def_readwrite("yield_min_delay", &ReplanConfig::yield_min_delay)
        .def_readwrite("affected_window", &ReplanConfig::affected_window)
        .def("__repr__", [](const ReplanConfig& c) {
            return std::string("ReplanConfig(max_escalation_tiers=") +
                   std::to_string(c.max_escalation_tiers) + ", order=" +
                   (c.order == ReplanOrder::kFixedFirst ? "FIXED_FIRST" : "PRIORITY") +
                   ", check_reachability=" + (c.check_reachability ? "True" : "False") +
                   ", rescue_selection=" +
                   (c.rescue_selection == RescueSelection::kShortestPathConflicts
                        ? "SHORTEST_PATH_CONFLICTS"
                        : "BLOCKED_CELLS") +
                   ", request_yield=" + (c.request_yield ? "True" : "False") +
                   ", yield_min_delay=" + std::to_string(c.yield_min_delay) +
                   ", affected_window=" + std::to_string(c.affected_window) + ")";
        });

    // ── 근무표 (TaskWindow, TaskState, 근무표 함수) ───────────────
    // 17장 결정 3·9. TaskSchedule(= std::unordered_map<int, TaskWindow>)은 PlanResult처럼
    // 따로 바인딩하지 않는다 — TaskWindow만 등록되면 dict[int, TaskWindow]로 자동 변환된다.
    py::class_<TaskWindow>(m, "TaskWindow")
        .def(py::init<>())
        .def(py::init([](int start, int end) { return TaskWindow{start, end}; }),
             py::arg("start"), py::arg("end"))
        .def_readwrite("start", &TaskWindow::start)
        .def_readwrite("end", &TaskWindow::end)
        .def(py::self == py::self)
        .def("__repr__", [](const TaskWindow& w) {
            return "TaskWindow(start=" + std::to_string(w.start) + ", end=" +
                   std::to_string(w.end) + ")";
        });

    py::enum_<TaskState>(m, "TaskState")
        .value("MOVING", TaskState::kMoving)
        .value("WORKING", TaskState::kWorking)
        .value("PARKED", TaskState::kParked);

    m.def("task_state_at", &task_state_at, py::arg("window"), py::arg("t"));
    m.def("window_from_path", &window_from_path, py::arg("agent"), py::arg("path"));
    m.def("schedule_from_plan", &schedule_from_plan, py::arg("agents"), py::arg("paths"));
    m.def("update_schedule", &update_schedule, py::arg("agents"), py::arg("previous"),
          py::arg("new_paths"), py::arg("current_time"));

    // ── ReplanResult ────────────────────────────────────────────
    // paths는 PlanResult(= std::unordered_map<int, Path>) 그대로 반환된다 —
    // 파이썬에서는 dict[int, list[SpaceTimeCell]]로 자연스럽게 보인다.
    py::class_<ReplanResult>(m, "ReplanResult")
        .def_readonly("paths", &ReplanResult::paths)
        .def_readonly("replanned_ids", &ReplanResult::replanned_ids)
        .def_readonly("escalation_tier", &ReplanResult::escalation_tier)
        .def_readonly("rescued_ids", &ReplanResult::rescued_ids)
        .def_readonly("schedule", &ReplanResult::schedule)
        .def_readonly("yielded_ids", &ReplanResult::yielded_ids)
        .def_readonly("deferred_ids", &ReplanResult::deferred_ids)
        .def_readonly("event_times", &ReplanResult::event_times);

    // ── PrioritizedPlanner ─────────────────────────────────────────────────────
    // PrioritizedPlanner는 생성자 인자로 받은 const Map&를 멤버로 그대로 저장한다(복사가
    // 아니라 참조). 파이썬에서 Map 객체가 PrioritizedPlanner보다 먼저 가비지 컬렉션되면
    // PrioritizedPlanner가 댕글링 참조를 갖게 되어 크래시가 난다 — py::keep_alive<1,2>()로
    // "이 생성자가 만드는 PrioritizedPlanner 객체(인덱스 1)가 살아있는 동안 map 인자
    // (인덱스 2)도 반드시 함께 살아있게" 강제한다. 이 한 줄을 빠뜨리면
    // 안 되는 이유가 바로 이것 — 절대 생략하지 말 것.
    py::class_<PrioritizedPlanner>(m, "PrioritizedPlanner")
        .def(py::init<const Map&, AStarConfig, ReplanConfig>(), py::arg("map"),
             py::arg("config") = AStarConfig{}, py::arg("replan_config") = ReplanConfig{},
             py::keep_alive<1, 2>())
        .def("plan", &PrioritizedPlanner::plan, py::arg("agents"))
        .def("replan",
             py::overload_cast<const std::vector<Agent>&, const PlanResult&,
                               const std::vector<Cell>&, int>(&PrioritizedPlanner::replan),
             py::arg("agents"), py::arg("previous_paths"),
             py::arg("new_obstacles"), py::arg("current_time"))
        .def("full_replan",
             py::overload_cast<const std::vector<Agent>&, const PlanResult&,
                               const std::vector<Cell>&, int>(&PrioritizedPlanner::full_replan),
             py::arg("agents"), py::arg("previous_paths"),
             py::arg("new_obstacles"), py::arg("current_time"))
        // 근무표를 받는 새 버전(17장). 같은 이름의 예전 버전(근무표 없음)과 인자 개수가 달라서
        // pybind11이 호출할 때 인자를 보고 알맞은 쪽을 고른다.
        .def("replan",
             py::overload_cast<const std::vector<Agent>&, const PlanResult&, const TaskSchedule&,
                               const std::vector<Cell>&, int>(&PrioritizedPlanner::replan),
             py::arg("agents"), py::arg("previous_paths"), py::arg("previous_schedule"),
             py::arg("new_obstacles"), py::arg("current_time"))
        .def("full_replan",
             py::overload_cast<const std::vector<Agent>&, const PlanResult&, const TaskSchedule&,
                               const std::vector<Cell>&, int>(&PrioritizedPlanner::full_replan),
             py::arg("agents"), py::arg("previous_paths"), py::arg("previous_schedule"),
             py::arg("new_obstacles"), py::arg("current_time"))
        // 6번 Park식(27장 P1): affected_window를 쓸 때 미룬 로봇이 창에 들어올 때마다 replan을 다시 부른다.
        .def("replan_windowed", &PrioritizedPlanner::replan_windowed, py::arg("agents"),
             py::arg("previous_paths"), py::arg("previous_schedule"), py::arg("new_obstacles"),
             py::arg("current_time"))
        .def_static("path_hits_obstacle", &PrioritizedPlanner::path_hits_obstacle, py::arg("path"),
                    py::arg("new_obstacles"), py::arg("current_time"))
        .def_static("first_hit_time", &PrioritizedPlanner::first_hit_time, py::arg("path"),
                    py::arg("new_obstacles"), py::arg("current_time"));
}

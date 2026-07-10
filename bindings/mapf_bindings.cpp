// ─────────────────────────────────────────────────────────────────
// bindings/mapf_bindings.cpp
//
// C++ 코어(mapf::Cell/Agent/Map/PBS 등)를 파이썬 확장 모듈 mapf_py로
// 그대로 노출한다. GUI가 순수 파이썬으로 알고리즘을 다시 짜지 않고,
// 실제로 컴파일된 C++ PBS::plan/replan/full_replan을 직접 호출하게
// 하기 위한 얇은 바인딩 계층이다.
//
// 바인딩 순서가 곧 의존 순서다 — pybind11은 어떤 타입을 시그니처에서
// 참조하려면 그 타입이 먼저 등록되어 있어야 한다(Agent가 Cell을 쓰고,
// PBS 생성자가 Map/AStarConfig/PBSConfig를 쓰므로).
//
// PBSResult(= std::unordered_map<int, Path>)는 별도 바인딩이 필요 없다 —
// SpaceTimeCell만 등록되면 pybind11/stl.h의 제네릭 map/vector caster가
// dict[int, list[SpaceTimeCell]]로 자동 변환해준다. "바인딩된 PBSResult
// 클래스"는 따로 존재하지 않는다.
// ─────────────────────────────────────────────────────────────────
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "mapf/agent.hpp"
#include "mapf/map.hpp"
#include "mapf/pbs.hpp"
#include "mapf/space_time_astar.hpp"

namespace py = pybind11;
using namespace mapf;

PYBIND11_MODULE(mapf_py, m) {
    m.doc() = "mapf C++ 코어(Map, Agent, PBS)를 그대로 노출하는 pybind11 바인딩";

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
    // Agent(id, start, goal) 생성자를 바인딩해서, tools/benchmark.cpp의
    // Agent{i, shuffled_starts[i], shuffled_goals[i]} 패턴을 파이썬에서
    // 그대로 재현할 수 있게 한다. 우선순위는 이 객체들을 담은 리스트의
    // 순서로 정해진다(앞에 올수록 우선순위 높음) — 필드로 따로 없음.
    py::class_<Agent>(m, "Agent")
        .def(py::init<>())
        .def(py::init<int, Cell, Cell>(), py::arg("id"), py::arg("start"), py::arg("goal"))
        .def_readwrite("id", &Agent::id)
        .def_readwrite("start", &Agent::start)
        .def_readwrite("goal", &Agent::goal)
        .def("__repr__", [](const Agent& a) {
            return "Agent(id=" + std::to_string(a.id) + ", start=Cell(" +
                   std::to_string(a.start.x) + "," + std::to_string(a.start.y) + "), goal=Cell(" +
                   std::to_string(a.goal.x) + "," + std::to_string(a.goal.y) + "))";
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

    // ── AStarConfig / PBSConfig ─────────────────────────────────
    py::class_<AStarConfig>(m, "AStarConfig")
        .def(py::init<>())
        .def_readwrite("max_timestep", &AStarConfig::max_timestep);

    py::class_<PBSConfig>(m, "PBSConfig")
        .def(py::init<>())
        .def_readwrite("max_escalation_tiers", &PBSConfig::max_escalation_tiers);

    // ── ReplanResult ────────────────────────────────────────────
    // paths는 PBSResult(= std::unordered_map<int, Path>) 그대로 반환된다 —
    // 파이썬에서는 dict[int, list[SpaceTimeCell]]로 자연스럽게 보인다.
    py::class_<ReplanResult>(m, "ReplanResult")
        .def_readonly("paths", &ReplanResult::paths)
        .def_readonly("replanned_ids", &ReplanResult::replanned_ids)
        .def_readonly("escalation_tier", &ReplanResult::escalation_tier)
        .def_readonly("rescued_ids", &ReplanResult::rescued_ids);

    // ── PBS ─────────────────────────────────────────────────────
    // PBS는 생성자 인자로 받은 const Map&를 멤버로 그대로 저장한다(복사가
    // 아니라 참조). 파이썬에서 Map 객체가 PBS보다 먼저 가비지 컬렉션되면
    // PBS가 댕글링 참조를 갖게 되어 크래시가 난다 — py::keep_alive<1,2>()로
    // "이 생성자가 만드는 PBS 객체(인덱스 1)가 살아있는 동안 map 인자
    // (인덱스 2)도 반드시 함께 살아있게" 강제한다. 이 한 줄을 빠뜨리면
    // 안 되는 이유가 바로 이것 — 절대 생략하지 말 것.
    py::class_<PBS>(m, "PBS")
        .def(py::init<const Map&, AStarConfig, PBSConfig>(), py::arg("map"),
             py::arg("config") = AStarConfig{}, py::arg("replan_config") = PBSConfig{},
             py::keep_alive<1, 2>())
        .def("plan", &PBS::plan, py::arg("agents"))
        .def("replan", &PBS::replan, py::arg("agents"), py::arg("previous_paths"),
             py::arg("new_obstacles"), py::arg("current_time"))
        .def("full_replan", &PBS::full_replan, py::arg("agents"), py::arg("previous_paths"),
             py::arg("new_obstacles"), py::arg("current_time"))
        .def_static("path_hits_obstacle", &PBS::path_hits_obstacle, py::arg("path"),
                    py::arg("new_obstacles"), py::arg("current_time"));
}

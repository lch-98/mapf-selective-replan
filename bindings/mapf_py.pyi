# ─────────────────────────────────────────────────────────────────
# bindings/mapf_py.pyi
#
# mapf_py는 C++로 컴파일된 확장 모듈(.pyd/.so)이라 소스 코드가 없다 —
# IDE가 자동완성/타입 힌트를 보여줄 수 있도록 이 스텁 파일을 둔다.
# 실제 동작은 전부 bindings/mapf_bindings.cpp -> core/include/mapf/*.hpp를
# 참고할 것. 이 파일은 "무엇을 어떻게 호출할 수 있는지"만 설명하고,
# 알고리즘 자체의 동작(Tier 0/1/안전망 등)은 core/include/mapf/prioritized_planner.hpp의
# 주석과 docs/05_pbs.md, docs/06_selective_replan.md를 참고해야 한다.
# ─────────────────────────────────────────────────────────────────
from typing import ClassVar, Dict, List, Optional, overload

class Cell:
    """격자 위의 한 칸. x=열(가로), y=행(세로). 왼쪽 위가 (0,0)."""

    x: int
    y: int

    def __init__(self, x: int = 0, y: int = 0) -> None: ...
    def __eq__(self, other: object) -> bool: ...
    def __lt__(self, other: "Cell") -> bool: ...
    def __hash__(self) -> int:
        """C++ Cell에는 원래 해시가 없다 — 파이썬에서 set()/dict 키로
        쓸 수 있도록(예: 장애물 칸 집합) 바인딩 레이어에서 추가한 것."""
        ...
    def __repr__(self) -> str: ...

class SpaceTimeCell:
    """칸 + 시각(x, y, t). 로봇 경로(Path)를 이루는 기본 단위."""

    x: int
    y: int
    t: int

    def __init__(self, x: int = 0, y: int = 0, t: int = 0) -> None: ...
    def __eq__(self, other: object) -> bool: ...
    def __lt__(self, other: "SpaceTimeCell") -> bool: ...
    def __hash__(self) -> int: ...
    def __repr__(self) -> str: ...

# 한 로봇의 시공간 경로. path[i].t는 i와 같은 값으로 단조 증가하고,
# path[0]이 출발 시각의 위치다.
Path = List[SpaceTimeCell]

# 로봇 id -> 그 로봇의 경로. PrioritizedPlanner.plan()/full_replan()의 반환값이자
# PrioritizedPlanner.replan()/full_replan()이 받는 previous_paths의 타입이기도 하다.
# 커스텀 바인딩된 클래스가 아니라 순수 파이썬 dict다(pybind11/stl.h의
# 제네릭 map/vector 변환).
PlanResult = Dict[int, Path]

class Agent:
    """로봇 한 대. 우선순위는 필드가 아니라, PrioritizedPlanner에 넘기는 리스트 안에서의
    "순서"로 정해진다 — 리스트 앞쪽일수록 우선순위가 높다."""

    id: int
    start: Cell
    goal: Cell
    service_time: int
    """목적지에 도착한 뒤 작업하는 스텝 수(17장). 작업 중에는 절대 움직이지 않고, 작업이
    끝나면 주차 상태라 구조 로봇으로 불리면 비켜 줄 수 있다. 0이면 "도착 = 작업 완료"(예전과 같음)."""

    def __init__(
        self, id: int = 0, start: Cell = ..., goal: Cell = ..., service_time: int = 0
    ) -> None: ...
    def __repr__(self) -> str: ...

class Map:
    """격자 전체(가로x세로 크기 + 어느 칸이 벽인지). 동적 장애물(다른
    로봇 등 시간에 따라 막히는 것)은 다루지 않는다 — 그건 PrioritizedPlanner 내부의
    ReservationTable 책임이다."""

    @overload
    def __init__(self, width: int, height: int) -> None:
        """빈 맵(벽 없음)을 만든다. width/height가 음수면 ValueError."""
        ...
    @overload
    def __init__(self, rows: List[str]) -> None:
        """ASCII 그림으로 맵을 만든다. '#' = 벽, 그 외 문자 = 빈 칸.
        rows[y][x]가 칸 (x, y)에 대응한다. 행마다 길이가 다르면 ValueError."""
        ...
    def width(self) -> int: ...
    def height(self) -> int: ...
    def is_passable(self, x: int, y: int) -> bool:
        """격자 범위 밖이거나 벽이면 False."""
        ...
    def neighbors(self, cell: Cell) -> List[Cell]:
        """한 스텝에 갈 수 있는 칸들: 상/하/좌/우 + 대기(자기 자신)."""
        ...
    def set_obstacle(self, x: int, y: int) -> None:
        """런타임에 정적 장애물(영원히 막힌 벽)을 추가한다."""
        ...

class TieBreak:
    """A*에서 f가 같은 노드 중 무엇을 먼저 꺼낼지(21장 5-5). 경로 길이는 안 바뀐다."""

    LARGER_G: ClassVar["TieBreak"]
    """기본: g가 큰 쪽 → 먼저 넣은 쪽."""
    SMALLER_G: ClassVar["TieBreak"]
    """g가 작은 쪽 → 먼저 넣은 쪽."""
    RANDOM: ClassVar["TieBreak"]
    """g가 큰 쪽 → tie_seed로 정한 무작위 순."""

class AStarConfig:
    """SpaceTimeAStar 탐색의 설정. PrioritizedPlanner 생성자에 선택적으로 넘긴다."""

    max_timestep: int  # 기본값 256. 이 시각을 넘으면 탐색을 포기한다.

    def __init__(self) -> None: ...

class ReplanOrder:
    """한 번의 재계획 시도에서 로봇을 장부에 등록하는 순서(12장)."""

    PRIORITY: ClassVar["ReplanOrder"]
    """원래 방식: agents 순서(우선순위) 그대로."""
    FIXED_FIRST: ClassVar["ReplanOrder"]
    """고정 로봇 먼저 → 영향받은 로봇 → 구조 로봇(우리 방법)."""

class RescueSelection:
    """A*가 실패했을 때 구조 로봇을 고르는 방법(13장)."""

    BLOCKED_CELLS: ClassVar["RescueSelection"]
    """원래 방식: A*가 막혀 본 모든 칸의 주인."""
    SHORTEST_PATH_CONFLICTS: ClassVar["RescueSelection"]
    """다른 로봇이 없을 때의 가장 짧은 길과 부딪히는 로봇만(우리 방법)."""

class ReplanConfig:
    """PrioritizedPlanner.replan()의 설정. 기본값은 원래 방식이다.
    우리 방법 = order=FIXED_FIRST, check_reachability=True,
    rescue_selection=SHORTEST_PATH_CONFLICTS."""

    max_escalation_tiers: int
    """기본값 1. Tier 1 이후 몇 단계까지 확장할지."""
    order: ReplanOrder
    """기본값 PRIORITY."""
    check_reachability: bool
    """기본값 False. 로봇이 아무도 없어도 못 가면 바로 실패(13장)."""
    rescue_selection: RescueSelection
    """기본값 BLOCKED_CELLS."""
    request_yield: bool
    """기본값 False. 크게 돌아가게 됐으면 서 있는 로봇에게 비켜 달라고 한 번 더 시도(19장)."""
    yield_min_delay: int
    """기본값 1. 가장 빠른 도착보다 이만큼 이상 늦은 로봇이 있을 때만 비켜 달라고 한다."""
    affected_window: int
    """기본값 -1(끔). 0 이상이면 6번 비교 기준 Park식: 지금부터 이 스텝 안에 장애물에 닿는 로봇만 지금 다시
    계획하고, 더 늦게 닿는 로봇은 미룬다(ReplanResult.deferred_ids). replan_windowed()가 창에 들어올 때 다시 계획한다."""

    def __init__(self) -> None: ...

class TaskWindow:
    """로봇 한 대의 작업 구간(17장 결정 3). [start, end) 동안 작업하고, end 시각부터 움직일 수 있다.
    예: t2에 도착, 작업 3 -> TaskWindow(start=2, end=5). 작업 시간이 0이면 start == end."""

    start: int
    """작업 시작 = 목적지 도착 시각."""
    end: int
    """작업 끝 = 이 시각부터 움직일 수 있다(start + service_time)."""

    def __init__(self, start: int = 0, end: int = 0) -> None: ...
    def __eq__(self, other: object) -> bool: ...
    def __repr__(self) -> str: ...

# 로봇 id -> 작업 구간. 근무표. PlanResult처럼 순수 파이썬 dict다.
TaskSchedule = Dict[int, TaskWindow]

class TaskState:
    """어떤 시각에 로봇이 어떤 상태인지(17장 17.3절)."""

    MOVING: "TaskState"   # t < start: 아직 도착 전
    WORKING: "TaskState"  # start <= t < end: 작업 중, 절대 안 움직임
    PARKED: "TaskState"   # end <= t: 작업 끝, 주차 중, 불리면 비켜 줄 수 있음

def task_state_at(window: TaskWindow, t: int) -> TaskState:
    """시각 t에 이 작업 구간의 로봇이 이동 중 / 작업 중 / 주차 중 중 무엇인지."""
    ...

def window_from_path(agent: Agent, path: Path) -> TaskWindow:
    """새로 계획한 경로에서 작업 구간을 읽는다(end = 경로 끝, start = end - service_time).
    처음 계획 결과에만 쓴다. 경로 끝의 (service_time + 1)칸이 목적지가 아니면 ValueError."""
    ...

def schedule_from_plan(agents: List[Agent], paths: PlanResult) -> TaskSchedule:
    """처음 계획(plan)의 결과에서 근무표를 만든다."""
    ...

def update_schedule(
    agents: List[Agent],
    previous: TaskSchedule,
    new_paths: PlanResult,
    current_time: int,
) -> TaskSchedule:
    """재계획 후의 근무표. current_time에 아직 이동 중이던 로봇만 새 경로로 다시 계산하고,
    이미 도착한(작업 중, 주차 중) 로봇은 그대로 둔다. full_replan 결과에 쓴다
    (replan은 결과의 schedule에 이미 담아 준다)."""
    ...

class ReplanResult:
    """PrioritizedPlanner.replan() 한 번의 결과. 최종 경로 외에 "얼마나 선택적으로
    풀렸는지"를 보여주는 디버깅·연구용 정보를 함께 담는다."""

    paths: PlanResult
    """최종 전체 경로. 영향 안 받은 로봇은 기존 경로 그대로."""

    replanned_ids: List[int]
    """실제로 다시 탐색된 로봇 id 전부."""

    escalation_tier: int
    """0=Tier 0에서 끝남, 1 이상=그 단계까지 확장해서 성공,
    -1=전체 재계획(안전망)으로 폴백해서 성공."""

    rescued_ids: List[int]
    """Tier 1+에서 새로 추가된 "구조 로봇" id 목록. 비어있으면 Tier 0만으로
    충분했다는 뜻."""

    schedule: TaskSchedule
    """재계획 후의 근무표. 다음 replan()에 previous_schedule로 그대로 넘긴다."""

    yielded_ids: List[int]
    """비켜 달라고 해서 결과에 쓰인 로봇 id(19장, ReplanConfig.request_yield를 켰을 때만)."""

    deferred_ids: List[int]
    """ReplanConfig.affected_window 때문에 이번에 미룬 로봇 id. 옛 경로가 아직 장애물을 밟는다."""

    event_times: List[int]
    """replan_windowed()에서 replan()을 부른 시각들(첫 값 = current_time). replan()에서는 빈 리스트."""

class PrioritizedPlanner:
    """고정 우선순위 계획(Prioritized Planning, PP). 우선순위는 agents 리스트의 순서
    그대로다. 예전 이름은 PBS였다(순서를 탐색하는 진짜 PBS와 달라서 이름을 바꿈)."""

    def __init__(
        self,
        map: Map,
        config: AStarConfig = ...,
        replan_config: ReplanConfig = ...,
    ) -> None:
        """주의: 이 PrioritizedPlanner 객체는 map을 참조로 저장한다(복사가 아님). map
        객체가 살아있는 동안만 이 PrioritizedPlanner 객체를 안전하게 쓸 수 있다 —
        pybind11의 keep_alive로 파이썬 GC가 map을 먼저 수거하지 못하도록
        막아뒀으므로 일반적인 사용에서는 신경 쓸 필요 없다."""
        ...

    def plan(self, agents: List[Agent]) -> Optional[PlanResult]:
        """agents를 순서대로(=우선순위 순으로) 전부 계획한다. 한 로봇이라도
        경로를 못 찾으면 None을 반환한다. agents에 중복 id가 있거나 service_time이
        음수면 ValueError. agents가 빈 리스트면 빈 dict를 성공으로 반환한다.
        각 경로는 도착 후 작업 칸(service_time개)까지 포함해서 끝난다. 근무표는
        schedule_from_plan(agents, 결과)로 만든다."""
        ...

    @overload
    def replan(
        self,
        agents: List[Agent],
        previous_paths: PlanResult,
        previous_schedule: TaskSchedule,
        new_obstacles: List[Cell],
        current_time: int,
    ) -> Optional[ReplanResult]:
        """근무표를 받는 버전(17장). 지금 작업 중인 로봇은 작업이 끝날 때까지 절대 움직이지
        않는다(구조 로봇으로 불려도 작업이 끝난 뒤에 비킨다). 결과의 schedule에 새 근무표가 담긴다."""
        ...
    @overload
    def replan(
        self,
        agents: List[Agent],
        previous_paths: PlanResult,
        new_obstacles: List[Cell],
        current_time: int,
    ) -> Optional[ReplanResult]:
        """이미 진행 중인 계획(previous_paths)에 new_obstacles가 새로
        생겼을 때, 영향받은 로봇만 선택적으로 다시 계획한다. Tier 0(영향받은
        로봇만) -> Tier 1..max_escalation_tiers(막은 원인 로봇을 추적해서
        확장) -> 안전망(전체 재계획)을 순서대로 시도한다. 안전망까지도
        실패하면 None. 근무표 없이 부르는 예전 버전이라, service_time이 있는 로봇이
        하나라도 있으면 ValueError(근무표를 받는 버전을 써야 한다)."""
        ...

    @overload
    def full_replan(
        self,
        agents: List[Agent],
        previous_paths: PlanResult,
        previous_schedule: TaskSchedule,
        new_obstacles: List[Cell],
        current_time: int,
    ) -> Optional[PlanResult]:
        """근무표를 받는 버전(17장). 작업 중인 로봇은 여기서도 작업이 끝날 때까지 안 움직인다.
        새 근무표는 update_schedule(agents, previous_schedule, 결과, current_time)로 만든다."""
        ...
    @overload
    def full_replan(
        self,
        agents: List[Agent],
        previous_paths: PlanResult,
        new_obstacles: List[Cell],
        current_time: int,
    ) -> Optional[PlanResult]:
        """"현재 위치 기준 전체 재계획" — replan()이 안전망으로 쓰는 것과
        동일한 로직. working_ids 구분 없이 agents 전체를, current_time
        시점 위치를 새 출발점으로 삼아 처음부터 다시 계획한다(이미 지나온
        길은 보존). 선택적 재계획과 공정하게 비교할 "베이스라인"으로 쓴다.
        예전 버전이라 service_time이 있는 로봇이 있으면 ValueError."""
        ...

    def replan_windowed(
        self,
        agents: List[Agent],
        previous_paths: PlanResult,
        previous_schedule: TaskSchedule,
        new_obstacles: List[Cell],
        current_time: int,
    ) -> Optional[ReplanResult]:
        """6번 Park식(affected_window). current_time에 replan()을 부르고, 미룬 로봇이 남으면 그중 가장 먼저
        창에 들어오는 시각(장애물에 닿는 시각 - affected_window)에 다시 부른다. 미룬 로봇이 없어질 때까지.
        replanned_ids는 모든 회차의 합, escalation_tier는 한 번이라도 안전망이면 -1, event_times는 부른 시각들."""
        ...

    @staticmethod
    def first_hit_time(
        path: Path, new_obstacles: List[Cell], current_time: int
    ) -> Optional[int]:
        """path가 current_time 이후 처음으로 new_obstacles 중 한 칸에 서는 시각. 없으면 None."""
        ...

    @staticmethod
    def path_hits_obstacle(
        path: Path, new_obstacles: List[Cell], current_time: int
    ) -> bool:
        """path가 current_time 이후 시점에 new_obstacles 중 한 칸과 같은
        (칸,시각)에서 겹치는지 확인한다. replan()이 "영향받는 로봇"을
        판별할 때 쓰는 것과 정확히 같은 기준이므로, 재계획을 실제로
        호출하기 전에 "이 장애물이 의미가 있는지" 미리 확인할 때 쓴다."""
        ...

# ─────────────────────────────────────────────────────────────────
# bindings/mapf_py.pyi
#
# mapf_py는 C++로 컴파일된 확장 모듈(.pyd/.so)이라 소스 코드가 없다 —
# IDE가 자동완성/타입 힌트를 보여줄 수 있도록 이 스텁 파일을 둔다.
# 실제 동작은 전부 bindings/mapf_bindings.cpp -> core/include/mapf/*.hpp를
# 참고할 것. 이 파일은 "무엇을 어떻게 호출할 수 있는지"만 설명하고,
# 알고리즘 자체의 동작(Tier 0/1/안전망 등)은 core/include/mapf/pbs.hpp의
# 주석과 docs/05_pbs.md, docs/06_selective_replan.md를 참고해야 한다.
# ─────────────────────────────────────────────────────────────────
from typing import Dict, List, Optional, overload

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

# 로봇 id -> 그 로봇의 경로. PBS.plan()/full_replan()의 반환값이자
# PBS.replan()/full_replan()이 받는 previous_paths의 타입이기도 하다.
# 커스텀 바인딩된 클래스가 아니라 순수 파이썬 dict다(pybind11/stl.h의
# 제네릭 map/vector 변환).
PBSResult = Dict[int, Path]

class Agent:
    """로봇 한 대. 우선순위는 필드가 아니라, PBS에 넘기는 리스트 안에서의
    "순서"로 정해진다 — 리스트 앞쪽일수록 우선순위가 높다."""

    id: int
    start: Cell
    goal: Cell

    def __init__(self, id: int = 0, start: Cell = ..., goal: Cell = ...) -> None: ...
    def __repr__(self) -> str: ...

class Map:
    """격자 전체(가로x세로 크기 + 어느 칸이 벽인지). 동적 장애물(다른
    로봇 등 시간에 따라 막히는 것)은 다루지 않는다 — 그건 PBS 내부의
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

class AStarConfig:
    """SpaceTimeAStar 탐색의 설정. PBS 생성자에 선택적으로 넘긴다."""

    max_timestep: int  # 기본값 256. 이 시각을 넘으면 탐색을 포기한다.

    def __init__(self) -> None: ...

class PBSConfig:
    """PBS.replan()의 계층적 확장(Tiered Escalation) 설정."""

    max_escalation_tiers: int  # 기본값 1. Tier 1 이후 몇 단계까지 확장할지.

    def __init__(self) -> None: ...

class ReplanResult:
    """PBS.replan() 한 번의 결과. 최종 경로 외에 "얼마나 선택적으로
    풀렸는지"를 보여주는 디버깅·연구용 정보를 함께 담는다."""

    paths: PBSResult
    """최종 전체 경로. 영향 안 받은 로봇은 기존 경로 그대로."""

    replanned_ids: List[int]
    """실제로 다시 탐색된 로봇 id 전부."""

    escalation_tier: int
    """0=Tier 0에서 끝남, 1 이상=그 단계까지 확장해서 성공,
    -1=전체 재계획(안전망)으로 폴백해서 성공."""

    rescued_ids: List[int]
    """Tier 1+에서 새로 추가된 "구조 로봇" id 목록. 비어있으면 Tier 0만으로
    충분했다는 뜻."""

class PBS:
    """Priority-Based Search. 우선순위는 agents 리스트의 순서 그대로다."""

    def __init__(
        self,
        map: Map,
        config: AStarConfig = ...,
        replan_config: PBSConfig = ...,
    ) -> None:
        """주의: 이 PBS 객체는 map을 참조로 저장한다(복사가 아님). map
        객체가 살아있는 동안만 이 PBS 객체를 안전하게 쓸 수 있다 —
        pybind11의 keep_alive로 파이썬 GC가 map을 먼저 수거하지 못하도록
        막아뒀으므로 일반적인 사용에서는 신경 쓸 필요 없다."""
        ...

    def plan(self, agents: List[Agent]) -> Optional[PBSResult]:
        """agents를 순서대로(=우선순위 순으로) 전부 계획한다. 한 로봇이라도
        경로를 못 찾으면 None을 반환한다. agents에 중복 id가 있으면
        ValueError. agents가 빈 리스트면 빈 dict를 성공으로 반환한다."""
        ...

    def replan(
        self,
        agents: List[Agent],
        previous_paths: PBSResult,
        new_obstacles: List[Cell],
        current_time: int,
    ) -> Optional[ReplanResult]:
        """이미 진행 중인 계획(previous_paths)에 new_obstacles가 새로
        생겼을 때, 영향받은 로봇만 선택적으로 다시 계획한다. Tier 0(영향받은
        로봇만) -> Tier 1..max_escalation_tiers(막은 원인 로봇을 추적해서
        확장) -> 안전망(전체 재계획)을 순서대로 시도한다. 안전망까지도
        실패하면 None."""
        ...

    def full_replan(
        self,
        agents: List[Agent],
        previous_paths: PBSResult,
        new_obstacles: List[Cell],
        current_time: int,
    ) -> Optional[PBSResult]:
        """"현재 위치 기준 전체 재계획" — replan()이 안전망으로 쓰는 것과
        동일한 로직. working_ids 구분 없이 agents 전체를, current_time
        시점 위치를 새 출발점으로 삼아 처음부터 다시 계획한다(이미 지나온
        길은 보존). 선택적 재계획과 공정하게 비교할 "베이스라인"으로 쓴다."""
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

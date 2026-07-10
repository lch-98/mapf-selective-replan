# ─────────────────────────────────────────────────────────────────
# python/gui/app.pyi
#
# app.py의 공개 인터페이스. 이 파일이 GUI의 메인 루프 전체를 담당한다 —
# 시나리오 생성, 클릭 이벤트로 장애물을 놓았을 때 full_replan과
# replan(선택적 재계획)을 동시에 호출해 좌우 패널로 비교하는 것,
# SPACE/R/ESC 키 처리, 실패 상태에서의 조작 잠금까지 전부 여기에 있다.
#
# 실행 방법(둘 다 지원됨):
#   python python/gui/app.py --map open --agents 10 --seed 43   (리포 루트에서)
#   python app.py --map open --agents 10 --seed 43              (python/gui 폴더 안에서)
# ─────────────────────────────────────────────────────────────────
import argparse
from typing import Dict, List, Optional

import mapf_py

def parse_args() -> argparse.Namespace:
    """--map(open|corridor, 기본 corridor), --agents(기본 8),
    --seed(기본 None=매번 무작위)를 파싱한다."""
    ...

class ReplanSideResult:
    """좌우 패널 중 한쪽(full_replan 또는 selective replan)의 최신 계산
    결과를 담는 간단한 값 객체. 알고리즘 로직은 없고 결과 보관 용도다."""

    paths: "mapf_py.PBSResult"
    elapsed_ms: float
    ok: bool
    escalation_tier: Optional[int]
    """selective 쪽만 의미 있는 값(0=Tier0, 1+=확장, -1=안전망).
    full_replan 쪽은 이 개념이 없으므로 항상 None."""
    note: str
    """부가 안내 문구. 예: "(영향받은 로봇 없음 — 두 방법 다 변화 없이
    성공)", "(재계획 실패 — 로봇이 현재 위치에 멈춘 상태로 표시됨)"."""

    def __init__(self) -> None: ...

class App:
    """GUI 애플리케이션 본체. pygame 창 하나, 좌우 두 패널(왼쪽=Full
    Replan, 오른쪽=Selective Replan), 무작위로 배치된 로봇들을 관리한다."""

    rng: object  # random.Random
    game_map: "mapf_py.Map"
    num_agents: int
    agents: List["mapf_py.Agent"]
    initial_paths: "mapf_py.PBSResult"
    obstacles: List["mapf_py.Cell"]
    full_side: ReplanSideResult
    selective_side: ReplanSideResult

    def __init__(self, args: argparse.Namespace) -> None: ...

    def _start_new_scenario(self) -> None:
        """새 무작위 배치로 처음부터 다시 시작한다(__init__과 R키 재시작이
        공용으로 쓴다). rng는 새로 만들지 않고 이어서 쓴다 — --seed로
        고정했다면 재시작할 때마다 그 seed에서 이어지는 결정론적 다음
        배치가 나오고, --seed 없이 시작했다면 매번 다른 배치가 나온다."""
        ...

    def current_positions(self) -> Dict[int, "mapf_py.Cell"]:
        """각 agent id -> 현재 시각(clock_sim.current_time)의 위치.
        장애물 클릭이 로봇 위치와 겹치는지 검증할 때 쓴다."""
        ...

    @property
    def is_failed(self) -> bool:
        """full_side/selective_side 중 하나라도 재계획에 실패했으면 True.
        이 상태에서는 SPACE(시간 진행)와 마우스 클릭(장애물 배치)이 모두
        무시되고, R키로만 새로 시작할 수 있다."""
        ...

    def show_message(self, text: str, duration_sec: float = 2.0) -> None:
        """화면 하단 상태 바에 몇 초간 뜨는 일시 메시지를 설정한다."""
        ...

    def handle_click(self, mx: int, my: int) -> None:
        """마우스 클릭 픽셀 좌표를 받아 장애물을 배치한다. 다음 경우
        클릭이 무시된다: (1) 실패 상태일 때, (2) 격자 밖/구분선을
        클릭했을 때, (3) 벽을 클릭했을 때, (4) 현재 시각에 어떤 로봇이
        서 있는 칸을 클릭했을 때(그 상태로 재계획하면 "로봇이 이미
        장애물 안에 서 있다"는 모순이 재현되어 반드시 실패하므로 사전
        차단). 유효한 클릭이면 장애물 목록에 추가하고 recompute()를
        호출한다."""
        ...

    def _frozen_paths_at_current_time(self) -> "mapf_py.PBSResult":
        """재계획 실패 시 보여줄 경로: 각 로봇의 과거 이동 이력은 그대로
        보존하고, current_time 시점부터는 같은 자리에 멈춰 선 것으로
        표시한다. 이게 없으면 실패해도 화면에 예전 성공 경로가 계속
        그려져 "장애물을 뚫고 지나가는" 것처럼 보이는 착시가 생긴다."""
        ...

    def recompute(self) -> None:
        """현재 obstacles/current_time을 가지고 full_replan과 replan을
        모두 호출해서 full_side/selective_side를 갱신한다. 아무 로봇도
        영향받지 않은 경우(PBS.path_hits_obstacle이 전부 False)는 그
        사실을 note로 알려준다."""
        ...

    def total_path_len(self, paths: "mapf_py.PBSResult") -> int:
        """모든 로봇의 경로 길이 합(단순 통계 표시용)."""
        ...

    def draw(self) -> None:
        """현재 상태를 한 프레임 그린다: 격자, 로봇, 경로, 장애물,
        통계, 하단 상태 바(시계/조작안내/실패경고)."""
        ...

    def run(self) -> None:
        """pygame 이벤트 루프. SPACE=시간 진행(실패 상태면 무시),
        마우스 좌클릭=handle_click, R=_start_new_scenario, ESC/창닫기=종료."""
        ...

def main() -> None:
    """parse_args() -> App(args) -> app.run()."""
    ...

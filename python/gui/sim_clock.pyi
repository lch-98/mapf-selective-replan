# ─────────────────────────────────────────────────────────────────
# python/gui/sim_clock.pyi
#
# sim_clock.py의 공개 인터페이스. GUI의 "현재 시각"을 관리하는 아주 작은
# 클래스 — SPACE 키 입력마다 정수 한 스텝씩 진행한다(자동 재생 없음).
#
# current_time 시맨틱에 대한 중요한 참고: tools/benchmark.cpp의
# make_obstacles_that_actually_block이 쓰는 "chosen.t - 1" 역산은 합성
# 시나리오 전용 트릭이고, 이 GUI의 current_time과는 무관하다 — 여기서는
# current_time이 사용자가 SPACE로 직접 통제하는 독립 변수다. 자세한 설명은
# sim_clock.py 상단 주석 참고.
# ─────────────────────────────────────────────────────────────────
from typing import List

import mapf_py

class SimClock:
    current_time: int  # 0부터 시작, advance() 호출마다 1씩 증가.

    def __init__(self) -> None: ...
    def advance(self) -> None:
        """current_time을 1 증가시킨다(SPACE 키 1회에 대응)."""
        ...
    def position_at(self, path: List["mapf_py.SpaceTimeCell"]) -> "mapf_py.Cell":
        """path[current_time]의 위치를 반환한다. current_time이 경로
        길이를 넘으면 마지막 칸(목적지)에 계속 머무는 것으로 본다 —
        PBS::position_at의 Tail Reservation 개념과 동일하다."""
        ...

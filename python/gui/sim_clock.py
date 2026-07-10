# ─────────────────────────────────────────────────────────────────
# python/gui/sim_clock.py
#
# 정수 current_time 하나만 관리한다. SPACE 키 입력 시 1씩 증가.
#
# current_time 시맨틱의 핵심: 벤치마크(tools/benchmark.cpp)의
# chosen.t - 1 역산은 "이미 정해진 경로 중 한 칸을 골라 그 시각을 거꾸로
# 계산"하는 합성 시나리오 전용 트릭이다. 이 GUI에서는 current_time이
# 사용자가 SPACE로 직접 통제하는 독립 변수이므로 그 역산이 필요 없다 —
# "장애물을 클릭한 그 순간의 시계 값"을 그대로 쓰면 된다.
#
# 다만 벤치마크에서 얻은 교훈 하나는 그대로 적용된다: 사용자가 클릭한 칸이
# 바로 그 순간(current_time) 어떤 로봇이 서 있는 칸과 정확히 같으면,
# "로봇이 이미 장애물 안에 서 있다"는 모순이 재현되어 register_path가
# 거절한다(PBS::register_path의 reserve_if_unowned 참고). 이 경우는
# app.py의 클릭 핸들러에서 사전에 거부한다(포지션이 겹치면 클릭 무시).
# ─────────────────────────────────────────────────────────────────
import mapf_py


class SimClock:
    def __init__(self) -> None:
        self.current_time = 0

    def advance(self) -> None:
        self.current_time += 1

    def position_at(self, path: list) -> "mapf_py.Cell":
        """path[current_time]의 위치. 범위를 넘으면 마지막 칸(목적지)에
        계속 머무는 것으로 본다 — PBS::position_at의 Tail Reservation
        개념과 동일하다."""
        if self.current_time < len(path):
            cell = path[self.current_time]
        else:
            cell = path[-1]
        return mapf_py.Cell(cell.x, cell.y)

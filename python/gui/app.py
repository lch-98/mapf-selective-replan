# ─────────────────────────────────────────────────────────────────
# python/gui/app.py
#
# 진입점. 맵 생성 -> 무작위 시나리오(초기 plan() 성공 보장) -> pygame
# 이벤트 루프(SPACE=시간 진행, 마우스 클릭=장애물 배치 -> full_replan과
# replan을 동시에 호출해서 좌우 패널에 비교 표시, ESC=종료).
#
# 실행: 이 폴더(python/gui) 안에서 바로 python app.py [--map open|corridor] [--agents N]
#
# gui 폴더 자체를 sys.path에 넣고 render/maps/scenario/sim_clock을 패키지
# 접두어 없이(import render) 불러온다 — python.gui 패키지를 거치지 않으므로
# 리포 루트가 아니라 이 폴더 안에서 실행해도 그대로 동작한다.
# ─────────────────────────────────────────────────────────────────
import argparse
import copy
import random
import sys
import time
from pathlib import Path

_GUI_DIR = Path(__file__).resolve().parent
_REPO_ROOT = _GUI_DIR.parent.parent
sys.path.insert(0, str(_GUI_DIR))          # render/maps/scenario/sim_clock용
sys.path.insert(0, str(_REPO_ROOT / "python"))  # _pathsetup용
from _pathsetup import add_mapf_py_to_path  # noqa: E402

add_mapf_py_to_path()

import mapf_py  # noqa: E402
import pygame  # noqa: E402

import render  # noqa: E402
from maps import MAP_FACTORIES  # noqa: E402
from scenario import find_solvable_scenario  # noqa: E402
from sim_clock import SimClock  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="MAPF 선택적 재계획 데모 GUI")
    parser.add_argument("--map", choices=list(MAP_FACTORIES.keys()), default="corridor")
    parser.add_argument("--agents", type=int, default=30)
    parser.add_argument("--seed", type=int, default=None)
    return parser.parse_args()


def _cell_at(path: list, t: int) -> tuple:
    """path에서 시각 t의 칸 (x, y). 경로의 i번째 칸이 시각 i이고, 경로가 끝난
    뒤에는 마지막 칸(목적지)에 머문다고 본다 — SimClock.position_at과 같은 규칙."""
    cell = path[t] if t < len(path) else path[-1]
    return (cell.x, cell.y)


class ReplanSideResult:
    """한쪽 패널(full_replan 또는 replan)의 최신 결과를 담는다."""

    def __init__(self):
        self.paths: dict = {}
        self.elapsed_ms: float = 0.0
        self.ok: bool = True
        self.escalation_tier = None  # full_replan 쪽은 항상 None(N/A)
        self.note: str = ""


class App:
    def __init__(self, args: argparse.Namespace):
        self.rng = random.Random(args.seed)
        self.game_map = MAP_FACTORIES[args.map]()
        self.num_agents = args.agents

        self.layout = render.PanelLayout(self.game_map.width(), self.game_map.height())
        pygame.init()
        pygame.display.set_caption("MAPF 데모 — full_replan vs replan")
        pygame.display.set_icon(render.make_app_icon())
        self.screen = pygame.display.set_mode(self.layout.window_size())
        # Consolas는 한글 글리프가 없어서 한글 텍스트가 깨진다(네모/물음표로
        # 표시됨) — 맑은 고딕(malgungothic, Windows 기본 한글 폰트)으로 교체.
        self.font = pygame.font.SysFont("malgungothic", 15)
        self.header_font = pygame.font.SysFont("malgungothic", 18, bold=True)
        self.pygame_clock = pygame.time.Clock()

        self.message = ""
        self.message_until = 0.0
        # W키로 켜고 끄는 "기다리는(양보 중인) 로봇" 강조. R로 재시작해도 유지한다.
        self.show_waiting = True

        self._start_new_scenario()

    def _start_new_scenario(self) -> None:
        """새 무작위 배치로 처음부터 다시 시작한다(R키 재시작, 최초 시작 공용).

        rng 자체는 새로 만들지 않고 계속 진행 중인 것을 그대로 쓴다 —
        --seed로 고정했다면 "그 seed로부터 이어지는" 다음 배치가 결정론적으로
        나오고, --seed 없이 매번 다르게 시작한다면 R을 눌러도 계속 다른
        배치가 나온다(완전한 무작위 재시작).
        """
        self.agents, self.initial_paths = find_solvable_scenario(
            self.game_map, self.num_agents, self.rng
        )
        self.colors = render.agent_colors(self.num_agents)

        self.clock_sim = SimClock()
        self.obstacles: list = []

        # 왼쪽=full_replan, 오른쪽=replan(선택적). 장애물이 없는 초기
        # 상태에서는 두 패널 다 초기 경로를 그대로 보여준다.
        self.full_side = ReplanSideResult()
        self.full_side.paths = self.initial_paths
        self.selective_side = ReplanSideResult()
        self.selective_side.paths = self.initial_paths

        # 한 스텝 되돌리기(←)용: SPACE를 누를 때마다 그 직전 상태를 쌓아 둔다.
        self.history: list = []

    def current_positions(self) -> list:
        """현재 시각에 로봇들이 서 있는 칸(Cell) 목록 — 두 패널 모두. 장애물 클릭 검증에 쓴다.

        첫 장애물 이후 왼쪽(full_replan)과 오른쪽(replan) 패널의 로봇은 서로 다른
        경로를 따라가므로, 어느 패널이든 로봇이 서 있는 칸이면 막아야 한다.
        처음 경로(initial_paths)로 계산하면 경로가 한 번 바뀐 뒤에는 실제 위치와 어긋난다.
        """
        positions = []
        for side in (self.full_side, self.selective_side):
            for agent in self.agents:
                positions.append(self.clock_sim.position_at(side.paths[agent.id]))
        return positions

    def waiting_agent_ids(self, paths: dict) -> set:
        """이번 스텝(t -> t+1)에 제자리에 머물다가 나중에 다시 움직일 로봇 id —
        즉 누군가에게 길을 양보하며 기다리는 로봇. 목적지에 도착해서 계속 머무는
        로봇(그 뒤로 더 안 움직임)과, 재계획 실패로 멈춰 선 로봇은 제외된다."""
        t = self.clock_sim.current_time
        waiting = set()
        for agent in self.agents:
            path = paths[agent.id]
            here = _cell_at(path, t)
            if _cell_at(path, t + 1) != here:
                continue
            if any((c.x, c.y) != here for c in path[t + 1:]):
                waiting.add(agent.id)
        return waiting

    def _snapshot(self) -> tuple:
        """한 스텝 되돌리기용 상태 저장. recompute()는 경로 dict를 제자리에서
        고치지 않고 통째로 새로 바꿔 끼우고, 장애물 목록만 append로 늘어나므로
        패널 결과는 얕은 복사, 장애물 목록은 리스트 복사로 충분하다."""
        return (self.clock_sim.current_time, list(self.obstacles),
                copy.copy(self.full_side), copy.copy(self.selective_side))

    def step_forward(self) -> None:
        """시간을 한 스텝 진행한다(SPACE). 되돌릴 수 있게 직전 상태를 쌓아 둔다."""
        if self.is_failed:
            self.show_message("재계획 실패 상태입니다 — ←로 되돌리거나 R키로 새로 시작하세요.")
            return
        self.history.append(self._snapshot())
        self.clock_sim.advance()

    def step_back(self) -> None:
        """시간을 한 스텝 되돌린다(← / Backspace): 마지막 SPACE 직전 상태로 복원한다.

        그 시각 이후에 놓은 장애물과 재계획 결과도 함께 취소된다 — 과거로
        돌아가면서 "나중에 생긴 장애물"을 그대로 두면 그 장애물이 원래보다 일찍
        있었던 셈이 되어 시나리오 자체가 달라지기 때문이다. 재계획 실패 상태에서도
        동작해서, 실패 직전으로 돌아가 다른 칸에 장애물을 놓아볼 수 있다.
        """
        if not self.history:
            self.show_message("처음 시각입니다 — 더 되돌릴 수 없습니다.")
            return
        t, obstacles, full_side, selective_side = self.history.pop()
        removed = len(self.obstacles) - len(obstacles)
        self.clock_sim.current_time = t
        self.obstacles = obstacles
        self.full_side = full_side
        self.selective_side = selective_side
        note = f" (그 뒤에 놓은 장애물 {removed}개 취소)" if removed else ""
        self.show_message(f"t = {t} 로 되돌렸습니다{note}.")

    @property
    def is_failed(self) -> bool:
        """둘 중 하나라도 재계획에 실패했으면 True.

        이 상태에서는 "이 이후 상황"이 애초에 정의되지 않는다 — 로봇이
        갈 곳을 못 찾았는데 SPACE로 시간을 더 흘리거나 새 장애물을 얹으면,
        이미 의미가 불분명해진 시나리오 위에 계속 조작을 쌓는 꼴이 된다.
        그래서 실패하면 조작을 멈추고 R로 완전히 새로 시작하게 한다.
        """
        return not (self.full_side.ok and self.selective_side.ok)

    def show_message(self, text: str, duration_sec: float = 2.0) -> None:
        self.message = text
        self.message_until = time.monotonic() + duration_sec

    def handle_click(self, mx: int, my: int) -> None:
        if self.is_failed:
            self.show_message("재계획 실패 상태입니다 — R키를 눌러 새로 시작하세요.")
            return
        panel_index = self.layout.panel_index_at(mx)
        if panel_index not in (0, 1):
            return
        cell_xy = self.layout.pixel_to_cell(panel_index, mx, my)
        if cell_xy is None:
            return
        x, y = cell_xy
        if not self.game_map.is_passable(x, y):
            self.show_message("벽에는 장애물을 놓을 수 없습니다.")
            return

        clicked = mapf_py.Cell(x, y)

        # 현재 시각에 어떤 로봇이 서 있는 칸과 겹치면 거부한다 —
        # sim_clock.py 주석 참고: 이 상태로 replan/full_replan을 호출하면
        # "로봇이 이미 장애물 안에 서 있다"는 모순이 재현되어
        # register_path가 거절해버린다(벤치마크에서 확인된 사각지대와 동일).
        positions = self.current_positions()
        if any(pos == clicked for pos in positions):
            self.show_message("로봇이 있는 칸에는 장애물을 놓을 수 없습니다.")
            return

        if clicked not in self.obstacles:
            self.obstacles.append(clicked)
        self.recompute()

    def _frozen_paths_at_current_time(self, base_paths: dict) -> dict:
        """재계획이 실패했을 때 보여줄 경로: 각 로봇을 current_time 시점
        위치에 멈춰 세운 "제자리 경로"를 만든다. base_paths는 그 패널이
        실패 직전까지 따라가던 경로다(처음 경로가 아니다 — 두 번째 장애물부터는
        이미 바뀐 경로를 따라가고 있으므로).

        실패해도 self.full_side.paths/self.selective_side.paths를 그대로
        두면(=이전 성공 결과를 계속 들고 있으면), 화면에는 "장애물이 생기기
        전 계산됐던 예전 경로"가 계속 그려져서 마치 로봇이 장애물을 뚫고
        지나가는 것처럼 보이는 착시가 생긴다(실제로는 재계획 자체가 실패해서
        갱신이 안 된 것뿐). 실패를 정직하게 보여주려면, 그 시점 이후로는
        "갈 곳을 못 찾았다"는 의미로 로봇을 현재 위치에 멈춰 세워야 한다.
        """
        current_time = self.clock_sim.current_time
        frozen = {}
        for agent in self.agents:
            path = base_paths[agent.id]
            here = self.clock_sim.position_at(path)
            # 과거 구간(0~current_time-1)은 실제 이동 이력 그대로 보존하고,
            # 그 이후는 같은 칸에 계속 머무는 것으로 표시한다.
            past = [c for c in path if c.t < current_time]
            frozen[agent.id] = past + [mapf_py.SpaceTimeCell(here.x, here.y, current_time)]
        return frozen

    def recompute(self) -> None:
        current_time = self.clock_sim.current_time

        # 각 패널은 "지금 자기 로봇들이 따라가고 있는 경로"를 기준으로 다시
        # 계획한다. 첫 장애물 이후 두 패널의 경로는 서로 달라지므로(방법이
        # 다르니까) 각자의 현재 경로를 넘겨야 한다. 처음 경로(initial_paths)를
        # 넘기면, 경로가 한 번 바뀐 뒤 두 번째 장애물부터는 "처음 경로상의
        # 위치"에서 재계획이 시작되어 로봇이 그 칸으로 순간이동한 것처럼 보인다.
        prev_full = self.full_side.paths
        prev_sel = self.selective_side.paths

        planner_full = mapf_py.PrioritizedPlanner(self.game_map)
        t0 = time.perf_counter()
        full_result = planner_full.full_replan(self.agents, prev_full, self.obstacles, current_time)
        full_elapsed = (time.perf_counter() - t0) * 1000.0

        self.full_side.ok = full_result is not None
        self.full_side.elapsed_ms = full_elapsed
        self.full_side.escalation_tier = None
        self.full_side.paths = (
            full_result if full_result is not None else self._frozen_paths_at_current_time(prev_full)
        )
        self.full_side.note = "" if full_result is not None else "(재계획 실패 — 로봇이 현재 위치에 멈춘 상태로 표시됨)"

        planner_sel = mapf_py.PrioritizedPlanner(self.game_map)
        t0 = time.perf_counter()
        sel_result = planner_sel.replan(self.agents, prev_sel, self.obstacles, current_time)
        sel_elapsed = (time.perf_counter() - t0) * 1000.0

        self.selective_side.ok = sel_result is not None
        self.selective_side.elapsed_ms = sel_elapsed
        if sel_result is not None:
            self.selective_side.paths = sel_result.paths
            self.selective_side.escalation_tier = sel_result.escalation_tier
            self.selective_side.note = ""
        else:
            self.selective_side.paths = self._frozen_paths_at_current_time(prev_sel)
            self.selective_side.escalation_tier = None
            self.selective_side.note = "(재계획 실패 — 로봇이 현재 위치에 멈춘 상태로 표시됨)"

        # 아무도 영향받지 않은 클릭이면(장애물이 그 패널의 어떤 경로와도 안 겹침),
        # 그 패널은 사실상 변화가 없다는 걸 사용자가 알 수 있게 안내한다. 두 패널의
        # 경로가 서로 다를 수 있으므로 패널마다 따로 판정한다. 실패해서 이미
        # "재계획 실패" note가 붙은 쪽은 덮어쓰지 않는다.
        for side, prev in ((self.full_side, prev_full), (self.selective_side, prev_sel)):
            if not side.ok:
                continue
            any_hit = any(
                mapf_py.PrioritizedPlanner.path_hits_obstacle(prev[a.id], self.obstacles, current_time)
                for a in self.agents
            )
            if not any_hit:
                side.note = "(영향받은 로봇 없음 — 변화 없이 성공)"

    def total_path_len(self, paths: dict) -> int:
        return sum(len(p) for p in paths.values())

    def draw(self) -> None:
        self.screen.fill(render.BG_COLOR)

        render.draw_header(self.screen, self.layout, 0, self.header_font, "Full Replan")
        render.draw_header(self.screen, self.layout, 1, self.header_font, "Selective Replan")

        waiting_counts = []
        for panel_index, side in ((0, self.full_side), (1, self.selective_side)):
            render.draw_grid(self.screen, self.layout, panel_index, self.game_map)
            for agent in self.agents:
                path = side.paths.get(agent.id, self.initial_paths[agent.id])
                render.draw_path(self.screen, self.layout, panel_index, path, self.colors[agent.id])
            waiting = self.waiting_agent_ids(side.paths)
            waiting_counts.append(len(waiting))
            for agent in self.agents:
                path = side.paths.get(agent.id, self.initial_paths[agent.id])
                cell = self.clock_sim.position_at(path)
                render.draw_robot(self.screen, self.layout, panel_index, cell, self.colors[agent.id])
                if self.show_waiting and agent.id in waiting:
                    render.draw_wait_marker(self.screen, self.layout, panel_index, cell)
            render.draw_obstacles(self.screen, self.layout, panel_index, self.obstacles)
            render.draw_stats(
                self.screen,
                self.layout,
                panel_index,
                self.font,
                side.elapsed_ms,
                side.ok,
                side.escalation_tier,
                self.total_path_len(side.paths),
                note=side.note,
            )

        render.draw_divider(self.screen, self.layout)

        # 창 맨 아래 전용 상태 바(패널/헤더와 절대 안 겹침) — 시계+조작안내를
        # 첫 줄에, 그 아래에 실패 안내 또는 일시 메시지를 둘째 줄에 그린다.
        status_bar_y = self.layout.panel_h
        wait_text = (
            f"대기(양보) 중: 왼쪽 {waiting_counts[0]}대 / 오른쪽 {waiting_counts[1]}대"
            if self.show_waiting else "대기 표시 꺼짐"
        )
        clock_text = self.font.render(
            f"t = {self.clock_sim.current_time}   {wait_text}   "
            f"(SPACE: 다음 스텝, ←: 이전 스텝, W: 대기 표시, 클릭: 장애물, R: 재시작, ESC: 종료)",
            True,
            render.TEXT_COLOR,
        )
        self.screen.blit(clock_text, (render.MARGIN, status_bar_y + 4))

        if self.is_failed:
            # 실패 상태는 한 번 뜨고 사라지는 메시지가 아니라, R을 누르기
            # 전까지 계속 눈에 띄어야 한다 — 조작이 막혀 있다는 사실을
            # 사용자가 계속 인지할 수 있게 상태 바에 고정 표시한다.
            stopped_surf = self.font.render(
                "재계획 실패로 정지됨 — ←로 한 스텝 되돌리거나 R키로 새 시나리오를 시작하세요.",
                True,
                (180, 0, 0),
            )
            self.screen.blit(stopped_surf, (render.MARGIN, status_bar_y + 24))
        elif self.message and time.monotonic() < self.message_until:
            msg_surf = self.font.render(self.message, True, (180, 0, 0))
            self.screen.blit(msg_surf, (render.MARGIN, status_bar_y + 24))

        pygame.display.flip()

    def run(self) -> None:
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
                    elif event.key == pygame.K_r:
                        self._start_new_scenario()
                        self.show_message("새 시나리오로 재시작했습니다.")
                    elif event.key == pygame.K_SPACE:
                        self.step_forward()
                    elif event.key in (pygame.K_LEFT, pygame.K_BACKSPACE):
                        self.step_back()
                    elif event.key == pygame.K_w:
                        self.show_waiting = not self.show_waiting
                elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                    self.handle_click(*event.pos)

            self.draw()
            self.pygame_clock.tick(30)

        pygame.quit()


def main() -> None:
    args = parse_args()
    app = App(args)
    app.run()


if __name__ == "__main__":
    main()

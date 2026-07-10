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
    parser.add_argument("--agents", type=int, default=8)
    parser.add_argument("--seed", type=int, default=None)
    return parser.parse_args()


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

    def current_positions(self) -> dict:
        """각 agent id -> 현재 시각의 위치(Cell). 장애물 클릭 검증에 쓴다."""
        positions = {}
        for agent in self.agents:
            path = self.initial_paths[agent.id]
            positions[agent.id] = self.clock_sim.position_at(path)
        return positions

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
        if any(pos == clicked for pos in positions.values()):
            self.show_message("로봇이 있는 칸에는 장애물을 놓을 수 없습니다.")
            return

        if clicked not in self.obstacles:
            self.obstacles.append(clicked)
        self.recompute()

    def _frozen_paths_at_current_time(self) -> dict:
        """재계획이 실패했을 때 보여줄 경로: 각 로봇을 current_time 시점
        위치에 멈춰 세운 "제자리 경로"를 만든다.

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
            path = self.initial_paths[agent.id]
            here = self.clock_sim.position_at(path)
            # 과거 구간(0~current_time-1)은 실제 이동 이력 그대로 보존하고,
            # 그 이후는 같은 칸에 계속 머무는 것으로 표시한다.
            past = [c for c in path if c.t < current_time]
            frozen[agent.id] = past + [mapf_py.SpaceTimeCell(here.x, here.y, current_time)]
        return frozen

    def recompute(self) -> None:
        current_time = self.clock_sim.current_time

        pbs_full = mapf_py.PBS(self.game_map)
        t0 = time.perf_counter()
        full_result = pbs_full.full_replan(
            self.agents, self.initial_paths, self.obstacles, current_time
        )
        full_elapsed = (time.perf_counter() - t0) * 1000.0

        self.full_side.ok = full_result is not None
        self.full_side.elapsed_ms = full_elapsed
        self.full_side.escalation_tier = None
        self.full_side.paths = full_result if full_result is not None else self._frozen_paths_at_current_time()
        self.full_side.note = "" if full_result is not None else "(재계획 실패 — 로봇이 현재 위치에 멈춘 상태로 표시됨)"

        pbs_sel = mapf_py.PBS(self.game_map)
        t0 = time.perf_counter()
        sel_result = pbs_sel.replan(self.agents, self.initial_paths, self.obstacles, current_time)
        sel_elapsed = (time.perf_counter() - t0) * 1000.0

        self.selective_side.ok = sel_result is not None
        self.selective_side.elapsed_ms = sel_elapsed
        if sel_result is not None:
            self.selective_side.paths = sel_result.paths
            self.selective_side.escalation_tier = sel_result.escalation_tier
            self.selective_side.note = ""
        else:
            self.selective_side.paths = self._frozen_paths_at_current_time()
            self.selective_side.escalation_tier = None
            self.selective_side.note = "(재계획 실패 — 로봇이 현재 위치에 멈춘 상태로 표시됨)"

        # 아무도 영향받지 않은 클릭이면(장애물이 어떤 경로와도 안 겹침),
        # 두 패널 다 사실상 변화가 없다는 걸 사용자가 알 수 있게 안내한다.
        # 실패해서 이미 "재계획 실패" note가 붙은 쪽은 덮어쓰지 않는다.
        any_hit = any(
            mapf_py.PBS.path_hits_obstacle(self.initial_paths[a.id], self.obstacles, current_time)
            for a in self.agents
        )
        if not any_hit:
            note = "(영향받은 로봇 없음 — 두 방법 다 변화 없이 성공)"
            if self.full_side.ok:
                self.full_side.note = note
            if self.selective_side.ok:
                self.selective_side.note = note

    def total_path_len(self, paths: dict) -> int:
        return sum(len(p) for p in paths.values())

    def draw(self) -> None:
        self.screen.fill(render.BG_COLOR)

        render.draw_header(self.screen, self.layout, 0, self.header_font, "Full Replan")
        render.draw_header(self.screen, self.layout, 1, self.header_font, "Selective Replan")

        for panel_index, side in ((0, self.full_side), (1, self.selective_side)):
            render.draw_grid(self.screen, self.layout, panel_index, self.game_map)
            for agent in self.agents:
                path = side.paths.get(agent.id, self.initial_paths[agent.id])
                render.draw_path(self.screen, self.layout, panel_index, path, self.colors[agent.id])
            for agent in self.agents:
                path = side.paths.get(agent.id, self.initial_paths[agent.id])
                cell = self.clock_sim.position_at(path)
                render.draw_robot(self.screen, self.layout, panel_index, cell, self.colors[agent.id])
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
        clock_text = self.font.render(
            f"t = {self.clock_sim.current_time}  "
            f"(SPACE: 다음 스텝, 클릭: 장애물 배치, R: 재시작, ESC: 종료)",
            True,
            render.TEXT_COLOR,
        )
        self.screen.blit(clock_text, (render.MARGIN, status_bar_y + 4))

        if self.is_failed:
            # 실패 상태는 한 번 뜨고 사라지는 메시지가 아니라, R을 누르기
            # 전까지 계속 눈에 띄어야 한다 — 조작이 막혀 있다는 사실을
            # 사용자가 계속 인지할 수 있게 상태 바에 고정 표시한다.
            stopped_surf = self.font.render(
                "재계획 실패로 정지됨 — R키를 눌러 새 시나리오로 재시작하세요.",
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
                        if self.is_failed:
                            self.show_message("재계획 실패 상태입니다 — R키를 눌러 새로 시작하세요.")
                        else:
                            self.clock_sim.advance()
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

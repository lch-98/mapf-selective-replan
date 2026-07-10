# ─────────────────────────────────────────────────────────────────
# python/gui/render.pyi
#
# render.py의 공개 인터페이스 요약. pygame으로 격자/로봇/경로/장애물을
# 그리는 순수 그리기 함수들 — 여기 있는 함수들은 알고리즘과 무관하고,
# "이미 계산된 결과(경로, 성공여부, escalation_tier 등)를 화면 어디에
# 어떻게 그릴지"만 담당한다. 실제 재계획 로직은 app.py -> mapf_py.PBS를
# 참고할 것.
# ─────────────────────────────────────────────────────────────────
from typing import List, Tuple

import pygame

# ── 픽셀 단위 상수 ──────────────────────────────────────────────
CELL_SIZE: int  # 격자 한 칸의 픽셀 크기. 창 전체 크기가 이 값에 비례한다.
STATS_HEIGHT: int  # 패널 하단 통계 텍스트 영역 높이.
HEADER_HEIGHT: int  # 패널 상단 "Full Replan"/"Selective Replan" 제목 높이.
MARGIN: int  # 격자 주변 여백.
STATUS_BAR_HEIGHT: int  # 창 맨 아래, 시계/조작안내/실패경고 전용 줄 높이.

# ── 색상(RGB 튜플) ─────────────────────────────────────────────
WALL_COLOR: Tuple[int, int, int]
FREE_COLOR: Tuple[int, int, int]
GRID_LINE_COLOR: Tuple[int, int, int]
OBSTACLE_COLOR: Tuple[int, int, int]
BG_COLOR: Tuple[int, int, int]
OK_COLOR: Tuple[int, int, int]
FAIL_COLOR: Tuple[int, int, int]
TEXT_COLOR: Tuple[int, int, int]
DIVIDER_COLOR: Tuple[int, int, int]

def agent_colors(num_agents: int) -> List[Tuple[int, int, int]]:
    """에이전트 id별 고유 색을 HSV 색상환에서 균등 분배해서 생성한다.
    반환 리스트의 인덱스가 곧 agent id다."""
    ...

class PanelLayout:
    """좌우 두 패널(Full Replan / Selective Replan) 중 하나가 화면의
    어느 픽셀 영역을 차지하는지 계산한다. 맵 크기 하나만 알면 두 패널이
    동일한 크기로 나란히 배치된다고 가정한다."""

    grid_w: int
    grid_h: int
    panel_w: int
    panel_h: int

    def __init__(self, map_width: int, map_height: int) -> None: ...
    def window_size(self) -> Tuple[int, int]:
        """pygame.display.set_mode에 그대로 넘길 (width, height)."""
        ...
    def grid_origin(self, panel_index: int) -> Tuple[int, int]:
        """panel_index(0=왼쪽/Full Replan, 1=오른쪽/Selective Replan)의
        격자 시작 픽셀 좌표."""
        ...
    def cell_to_pixel(self, panel_index: int, x: int, y: int) -> Tuple[int, int]:
        """격자 좌표 (x,y) -> 그 칸의 좌상단 픽셀 좌표."""
        ...
    def cell_center(self, panel_index: int, x: int, y: int) -> Tuple[int, int]:
        """격자 좌표 (x,y) -> 그 칸의 중심 픽셀 좌표(로봇/경로 그릴 때 사용)."""
        ...
    def pixel_to_cell(self, panel_index: int, mx: int, my: int) -> "Tuple[int, int] | None":
        """마우스 클릭 픽셀 좌표 -> 격자 좌표. 격자 밖 클릭이면 None
        (app.py의 handle_click이 이 None을 보고 클릭을 무시한다)."""
        ...
    def panel_index_at(self, mx: int) -> int:
        """마우스 x좌표만 보고 어느 패널(0/1) 위인지 판정. divider 위
        (어느 패널도 아님)이면 -1."""
        ...

def draw_grid(surface: "pygame.Surface", layout: PanelLayout, panel_index: int, game_map) -> None:
    """벽/빈칸 격자를 그린다. game_map은 mapf_py.Map(is_passable/width/height
    메서드만 씀)."""
    ...

def draw_path(
    surface: "pygame.Surface", layout: PanelLayout, panel_index: int, path: list, color
) -> None:
    """path(SpaceTimeCell 리스트, t는 무시하고 x,y만 순서대로 연결)를
    꺾은선으로 그린다. 길이 2 미만이면 아무것도 안 그림."""
    ...

def draw_robot(surface: "pygame.Surface", layout: PanelLayout, panel_index: int, cell, color) -> None:
    """로봇 한 대를 cell 위치에 원으로 그린다."""
    ...

def draw_obstacles(
    surface: "pygame.Surface", layout: PanelLayout, panel_index: int, obstacles: list
) -> None:
    """장애물 칸들을 빨간 X 마커로 그린다."""
    ...

def draw_header(
    surface: "pygame.Surface", layout: PanelLayout, panel_index: int, font, title: str
) -> None:
    """패널 상단에 제목("Full Replan"/"Selective Replan")을 그린다."""
    ...

def draw_stats(
    surface: "pygame.Surface",
    layout: PanelLayout,
    panel_index: int,
    font,
    elapsed_ms: float,
    ok: bool,
    escalation_tier,
    total_path_len: int,
    note: str = "",
) -> None:
    """패널 하단 통계 텍스트(소요시간, 성공/실패, escalation_tier,
    전체 경로 길이, 부가 안내문)를 그린다. escalation_tier가 None이면
    "N/A"로 표시(Full Replan 쪽은 이 개념이 없으므로 항상 None)."""
    ...

def draw_divider(surface: "pygame.Surface", layout: PanelLayout) -> None:
    """두 패널 사이 세로 구분선을 창 전체 높이로 그린다."""
    ...

def make_app_icon(size: int = 32) -> "pygame.Surface":
    """외부 이미지 파일 없이 코드로 그리는 창 아이콘(격자+로봇 2대
    경로 모양). pygame.display.set_icon()에 넘긴다."""
    ...

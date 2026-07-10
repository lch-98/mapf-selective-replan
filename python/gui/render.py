# ─────────────────────────────────────────────────────────────────
# python/gui/render.py
#
# pygame으로 격자/로봇/경로/장애물을 그린다. 좌우 분할 화면으로
# full_replan 결과(왼쪽)와 replan(선택적 재계획) 결과(오른쪽)를 나란히
# 비교해서 보여주는 것이 이 파일의 핵심 목적이다.
# ─────────────────────────────────────────────────────────────────
import colorsys

import pygame

CELL_SIZE = 24
STATS_HEIGHT = 70
HEADER_HEIGHT = 30
MARGIN = 12
# 창 맨 아래, 시계/조작안내와 "재계획 실패로 정지됨" 안내를 위한 전용 줄.
# 예전에는 이 안내를 왼쪽 패널 헤더와 같은 y좌표(6px)에 그려서 "Full Replan"
# 헤더 글자와 겹쳐 보이는 문제가 있었다 — 아예 별도의 하단 바를 만들어서
# 어떤 패널 헤더와도 안 겹치게 한다.
STATUS_BAR_HEIGHT = 44

WALL_COLOR = (40, 40, 45)
FREE_COLOR = (235, 235, 240)
GRID_LINE_COLOR = (200, 200, 205)
OBSTACLE_COLOR = (200, 30, 30)
BG_COLOR = (250, 250, 252)
OK_COLOR = (30, 140, 30)
FAIL_COLOR = (200, 30, 30)
TEXT_COLOR = (20, 20, 20)
DIVIDER_COLOR = (150, 150, 155)


def agent_colors(num_agents: int) -> list:
    """에이전트 id별 고유 색을 HSV로 균등 분배해서 생성한다."""
    colors = []
    for i in range(max(num_agents, 1)):
        hue = i / max(num_agents, 1)
        r, g, b = colorsys.hsv_to_rgb(hue, 0.75, 0.85)
        colors.append((int(r * 255), int(g * 255), int(b * 255)))
    return colors


class PanelLayout:
    """맵 1개 크기를 기준으로, 패널 하나가 차지하는 픽셀 영역을 계산한다."""

    def __init__(self, map_width: int, map_height: int):
        self.grid_w = map_width * CELL_SIZE
        self.grid_h = map_height * CELL_SIZE
        self.panel_w = self.grid_w + 2 * MARGIN
        self.panel_h = HEADER_HEIGHT + self.grid_h + STATS_HEIGHT + 2 * MARGIN

    def window_size(self) -> tuple:
        divider = 4
        return (self.panel_w * 2 + divider, self.panel_h + STATUS_BAR_HEIGHT)

    def grid_origin(self, panel_index: int) -> tuple:
        divider = 4
        x0 = panel_index * (self.panel_w + divider) + MARGIN
        y0 = HEADER_HEIGHT + MARGIN
        return x0, y0

    def cell_to_pixel(self, panel_index: int, x: int, y: int) -> tuple:
        x0, y0 = self.grid_origin(panel_index)
        return x0 + x * CELL_SIZE, y0 + y * CELL_SIZE

    def cell_center(self, panel_index: int, x: int, y: int) -> tuple:
        px, py = self.cell_to_pixel(panel_index, x, y)
        return px + CELL_SIZE // 2, py + CELL_SIZE // 2

    def pixel_to_cell(self, panel_index: int, mx: int, my: int):
        x0, y0 = self.grid_origin(panel_index)
        gx, gy = mx - x0, my - y0
        if gx < 0 or gy < 0 or gx >= self.grid_w or gy >= self.grid_h:
            return None
        return gx // CELL_SIZE, gy // CELL_SIZE

    def panel_index_at(self, mx: int) -> int:
        divider = 4
        return 0 if mx < self.panel_w else (1 if mx >= self.panel_w + divider else -1)


def draw_grid(surface, layout: PanelLayout, panel_index: int, game_map) -> None:
    x0, y0 = layout.grid_origin(panel_index)
    for y in range(game_map.height()):
        for x in range(game_map.width()):
            color = FREE_COLOR if game_map.is_passable(x, y) else WALL_COLOR
            rect = pygame.Rect(x0 + x * CELL_SIZE, y0 + y * CELL_SIZE, CELL_SIZE, CELL_SIZE)
            pygame.draw.rect(surface, color, rect)
            pygame.draw.rect(surface, GRID_LINE_COLOR, rect, width=1)


def draw_path(surface, layout: PanelLayout, panel_index: int, path: list, color) -> None:
    if len(path) < 2:
        return
    points = [layout.cell_center(panel_index, cell.x, cell.y) for cell in path]
    pygame.draw.lines(surface, color, False, points, width=2)


def draw_robot(surface, layout: PanelLayout, panel_index: int, cell, color) -> None:
    cx, cy = layout.cell_center(panel_index, cell.x, cell.y)
    pygame.draw.circle(surface, color, (cx, cy), CELL_SIZE // 2 - 2)
    pygame.draw.circle(surface, (20, 20, 20), (cx, cy), CELL_SIZE // 2 - 2, width=1)


def draw_obstacles(surface, layout: PanelLayout, panel_index: int, obstacles: list) -> None:
    for cell in obstacles:
        cx, cy = layout.cell_center(panel_index, cell.x, cell.y)
        half = CELL_SIZE // 2 - 3
        pygame.draw.line(surface, OBSTACLE_COLOR, (cx - half, cy - half), (cx + half, cy + half), 3)
        pygame.draw.line(surface, OBSTACLE_COLOR, (cx - half, cy + half), (cx + half, cy - half), 3)


def draw_header(surface, layout: PanelLayout, panel_index: int, font, title: str) -> None:
    divider = 4
    px0 = panel_index * (layout.panel_w + divider)
    text = font.render(title, True, TEXT_COLOR)
    surface.blit(text, (px0 + MARGIN, 4))


def draw_stats(
    surface,
    layout: PanelLayout,
    panel_index: int,
    font,
    elapsed_ms,
    ok,
    escalation_tier,
    total_path_len,
    note: str = "",
) -> None:
    divider = 4
    px0 = panel_index * (layout.panel_w + divider)
    y = HEADER_HEIGHT + layout.grid_h + MARGIN + 4

    status_color = OK_COLOR if ok else FAIL_COLOR
    status_text = "OK" if ok else "FAILED"

    line1 = font.render(f"elapsed: {elapsed_ms:.2f} ms   status: ", True, TEXT_COLOR)
    surface.blit(line1, (px0 + MARGIN, y))
    status_surf = font.render(status_text, True, status_color)
    surface.blit(status_surf, (px0 + MARGIN + line1.get_width(), y))

    tier_text = "N/A" if escalation_tier is None else str(escalation_tier)
    line2 = font.render(
        f"escalation_tier: {tier_text}   total path len: {total_path_len}", True, TEXT_COLOR
    )
    surface.blit(line2, (px0 + MARGIN, y + 20))

    if note:
        line3 = font.render(note, True, (120, 120, 120))
        surface.blit(line3, (px0 + MARGIN, y + 40))


def draw_divider(surface, layout: PanelLayout) -> None:
    divider = 4
    x = layout.panel_w
    pygame.draw.rect(surface, DIVIDER_COLOR, pygame.Rect(x, 0, divider, layout.window_size()[1]))


def make_app_icon(size: int = 32) -> "pygame.Surface":
    """창 왼쪽 상단에 뜨는 pygame 기본 아이콘 대신 쓸 MAPF 테마 아이콘을
    코드로 직접 그린다(외부 이미지 파일 의존 없음) — 작은 격자 배경 위에
    서로 다른 색의 로봇 2대와 그 경로(꺾인 선)를 표현해서, 이 앱이 여러
    로봇의 경로를 다루는 도구임을 아이콘만 보고도 짐작할 수 있게 한다."""
    icon = pygame.Surface((size, size), pygame.SRCALPHA)
    icon.fill((0, 0, 0, 0))

    cell = size // 4
    grid_color = (210, 215, 225, 255)
    for i in range(5):
        pygame.draw.line(icon, grid_color, (i * cell, 0), (i * cell, size), 1)
        pygame.draw.line(icon, grid_color, (0, i * cell), (size, i * cell), 1)

    def center(cx: int, cy: int) -> tuple:
        return cx * cell + cell // 2, cy * cell + cell // 2

    path_a = [center(0, 0), center(1, 0), center(1, 2), center(3, 2)]
    path_b = [center(3, 0), center(3, 1), center(1, 1), center(0, 3)]

    color_a = (70, 130, 200, 255)
    color_b = (220, 120, 60, 255)

    pygame.draw.lines(icon, color_a, False, path_a, width=2)
    pygame.draw.lines(icon, color_b, False, path_b, width=2)

    pygame.draw.circle(icon, color_a, path_a[-1], cell // 3)
    pygame.draw.circle(icon, color_b, path_b[-1], cell // 3)

    return icon

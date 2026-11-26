"""격자(2D 코어) ↔ 월드(Gazebo, 미터) 변환과 지도 규칙 — 7-1 (docs/31 31.3, 31.19).

방향 약속 (여기 한 곳에서만 정한다)
- 2D 격자: rows[y][x], (0,0) = 왼쪽 위, x는 오른쪽, y는 **아래로** 커진다(core/include/mapf/map.hpp).
- Gazebo 월드: x는 오른쪽(동), y는 **위로**(북) 커진다. 원점 = 격자 왼쪽 **아래** 모서리.
- 그래서 위에서 내려다본 Gazebo 화면이 2D 그림과 같은 모양이 되도록 세로를 뒤집는다:
      월드 x = x × cell + cell/2
      월드 y = (높이 − 1 − y) × cell + cell/2
- 격자에서 한 칸 이동 → 로봇이 바라볼 방향(yaw): 오른쪽 0°, 위(y−1) +90°, 왼쪽 180°, 아래(y+1) −90°.
"""
import math

CELL_SIZE = 0.5


# ── 지도 규칙: tools/benchmark_maps.hpp, python/gui/maps.py와 같은 규칙(크기만 바꿔 쓸 수 있게) ──
def make_map_rows(kind, size):
    """'open'(넓은 맵: 3x3 기둥, 8칸 간격, 6에서 시작) / 'corridor'(좁은 통로: 3x3 선반, 4칸 간격, 2에서 시작)."""
    start, step = {'open': (6, 8), 'corridor': (2, 4)}[kind]
    rows = [['.'] * size for _ in range(size)]
    for by in range(start, size - 3, step):
        for bx in range(start, size - 3, step):
            for y in range(by, by + 3):
                for x in range(bx, bx + 3):
                    rows[y][x] = '#'
    return [''.join(r) for r in rows]


# ── 칸 ↔ 월드 ──
def cell_center(x, y, height, cell=CELL_SIZE):
    """격자 칸 (x, y)의 중심 → 월드 (wx, wy) 미터."""
    return x * cell + cell / 2.0, (height - 1 - y) * cell + cell / 2.0


def world_to_cell(wx, wy, height, cell=CELL_SIZE):
    """월드 (wx, wy) → 그 점이 들어 있는 격자 칸 (x, y). 격자 밖이어도 계산은 한다(호출하는 쪽이 범위 확인)."""
    return int(math.floor(wx / cell)), height - 1 - int(math.floor(wy / cell))


def move_yaw(dx, dy):
    """격자에서 (dx, dy)로 한 칸 갈 때 바라볼 방향(라디안). 제자리(0, 0)면 None."""
    if (dx, dy) == (0, 0):
        return None
    # 격자 y는 아래로 커지므로 월드 방향은 (dx, −dy)
    return math.atan2(-dy, dx)


def blocked_boxes(rows, cell=CELL_SIZE):
    """'#' 칸마다 월드 좌표 사각형 (xmin, xmax, ymin, ymax)."""
    h = len(rows)
    boxes = []
    for y, row in enumerate(rows):
        for x, ch in enumerate(row):
            if ch == '#':
                cx, cy = cell_center(x, y, h, cell)
                boxes.append((cx - cell / 2, cx + cell / 2, cy - cell / 2, cy + cell / 2))
    return boxes


WALL_THICKNESS = 0.05


def wall_boxes(width, height, cell=CELL_SIZE, t=WALL_THICKNESS):
    """격자 바깥 테두리 벽 4개 (xmin, xmax, ymin, ymax). 격자 [0, W·cell] × [0, H·cell] 바로 바깥에 붙인다."""
    w, h = width * cell, height * cell
    return [(-t, w + t, -t, 0.0), (-t, w + t, h, h + t),   # 아래, 위
            (-t, 0.0, 0.0, h), (w, w + t, 0.0, h)]           # 왼쪽, 오른쪽


# ── 광선 쏘기: LiDAR가 "격자대로라면" 잴 거리 ──
def ray_box(ox, oy, dx, dy, box):
    """점 (ox, oy)에서 방향 (dx, dy)(단위 벡터)로 쏜 광선이 사각형에 처음 닿는 거리. 안 닿으면 None."""
    xmin, xmax, ymin, ymax = box
    tmin, tmax = -math.inf, math.inf
    for o, d, lo, hi in ((ox, dx, xmin, xmax), (oy, dy, ymin, ymax)):
        if abs(d) < 1e-12:
            if o < lo or o > hi:
                return None
        else:
            t1, t2 = (lo - o) / d, (hi - o) / d
            tmin, tmax = max(tmin, min(t1, t2)), min(tmax, max(t1, t2))
    if tmax < max(tmin, 0.0):
        return None
    return tmin if tmin >= 0.0 else None  # 광선 시작점이 사각형 안이면 None(그런 배치는 없어야 함)


def expected_range(ox, oy, angle, boxes):
    """(ox, oy)에서 angle 방향으로 가장 먼저 닿는 사각형까지 거리. 없으면 inf."""
    dx, dy = math.cos(angle), math.sin(angle)
    best = math.inf
    for b in boxes:
        t = ray_box(ox, oy, dx, dy, b)
        if t is not None and t < best:
            best = t
    return best


def ray_passes_near(ox, oy, angle, dist, px, py, radius):
    """광선 (ox,oy)→angle 방향 0~dist 구간이 점 (px,py)에서 radius 안을 지나가나(다른 로봇에 가린 광선 거르기)."""
    dx, dy = math.cos(angle), math.sin(angle)
    t = (px - ox) * dx + (py - oy) * dy
    t = max(0.0, min(dist if math.isfinite(dist) else 10.0, t))
    return math.hypot(ox + t * dx - px, oy + t * dy - py) < radius

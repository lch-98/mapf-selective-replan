"""grid.py 손 계산 테스트 — python3 -m pytest ros2/mapf_gazebo/test -q"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from mapf_gazebo.grid import (blocked_boxes, cell_center, expected_range, make_map_rows,  # noqa: E402
                              move_yaw, ray_box, wall_boxes, world_to_cell)


def test_corridor_10_matches_benchmark_rule():
    # benchmark_maps.hpp make_corridor_map 규칙을 size=10에 적용한 결과(손으로 그림): 3x3 선반 4개
    assert make_map_rows('corridor', 10) == [
        '..........',
        '..........',
        '..###.###.',
        '..###.###.',
        '..###.###.',
        '..........',
        '..###.###.',
        '..###.###.',
        '..###.###.',
        '..........',
    ]


def test_open_32_has_nine_pillars():
    rows = make_map_rows('open', 32)
    assert sum(r.count('#') for r in rows) == 9 * 9
    assert rows[6][6] == '#' and rows[5][6] == '.' and rows[22][22] == '#'


def test_cell_center_flips_y():
    # 높이 10: 맨 윗줄(y=0)이 월드에서 가장 위(y가 큼), 맨 아랫줄(y=9)이 원점 쪽
    assert cell_center(0, 0, 10) == (0.25, 4.75)
    assert cell_center(0, 9, 10) == (0.25, 0.25)
    assert cell_center(3, 2, 10) == (1.75, 3.75)


def test_world_to_cell_round_trip():
    for x in range(10):
        for y in range(7):
            wx, wy = cell_center(x, y, 7)
            assert world_to_cell(wx, wy, 7) == (x, y)
            assert world_to_cell(wx + 0.24, wy - 0.24, 7) == (x, y)  # 칸 안쪽이면 같은 칸


def test_move_yaw():
    assert move_yaw(1, 0) == 0.0
    assert math.isclose(move_yaw(0, -1), math.pi / 2)    # 격자 위(y−1) = 월드 북쪽
    assert math.isclose(move_yaw(0, 1), -math.pi / 2)
    assert math.isclose(abs(move_yaw(-1, 0)), math.pi)
    assert move_yaw(0, 0) is None


def test_ray_box_and_expected_range():
    box = (1.0, 2.0, -0.5, 0.5)
    assert math.isclose(ray_box(0.0, 0.0, 1.0, 0.0, box), 1.0)
    assert ray_box(0.0, 0.0, -1.0, 0.0, box) is None
    assert ray_box(0.0, 1.0, 1.0, 0.0, box) is None
    # 3x3 격자, 가운데만 선반, 테두리 벽. (0,1) 칸 중심에서 오른쪽을 보면 선반 왼쪽 면(x=0.5)까지 0.25 m
    rows = ['...', '.#.', '...']
    boxes = blocked_boxes(rows) + wall_boxes(3, 3)
    ox, oy = cell_center(0, 1, 3)
    assert math.isclose(expected_range(ox, oy, 0.0, boxes), 0.25)
    # 위(북)를 보면 바깥 벽 아랫면(y = 1.5)까지: 1.5 − 0.75 = 0.75 m
    assert math.isclose(expected_range(ox, oy, math.pi / 2, boxes), 0.75)

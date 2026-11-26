"""시나리오 JSON의 격자 → Gazebo .world — 7-1 (docs/31 31.9, 31.19).

    ros2 run mapf_gazebo grid_to_world scenario.json --out scenario.world

- '#' 칸마다 칸을 꽉 채우는 상자(선반, 높이 SHELF_HEIGHT). 모두 정적 모델 "shelves" 하나의 링크 하나에 들어간다(가볍게).
- 격자 바깥 테두리 벽 "walls"(두께 grid.WALL_THICKNESS).
- 바닥, 해, 정답 위치 플러그인(gazebo_ros_state) — empty_check.world와 같다.
- 좌표는 grid.cell_center(세로를 뒤집음)로만 계산한다.
"""
import argparse
import json

from mapf_gazebo.grid import blocked_boxes, wall_boxes

SHELF_HEIGHT = 0.5


def _box(name, box, h, rgba):
    xmin, xmax, ymin, ymax = box
    cx, cy, sx, sy = (xmin + xmax) / 2, (ymin + ymax) / 2, xmax - xmin, ymax - ymin
    geom = f'<geometry><box><size>{sx:.4f} {sy:.4f} {h:.4f}</size></box></geometry>'
    pose = f'<pose>{cx:.4f} {cy:.4f} {h / 2:.4f} 0 0 0</pose>'
    return (f'<collision name="{name}_c">{pose}{geom}</collision>'
            f'<visual name="{name}_v">{pose}{geom}<material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse></material></visual>')


def world_xml(scenario, flip_y=True):
    """flip_y=False는 변이 검사용(세로를 안 뒤집은 틀린 월드)."""
    rows = scenario['map']
    cell = scenario['cell_size']
    if not flip_y:
        rows = list(reversed(rows))
    shelves = ''.join(_box(f'shelf_{i}', b, SHELF_HEIGHT, '0.85 0.55 0.2 1') for i, b in enumerate(blocked_boxes(rows, cell)))
    walls = ''.join(_box(f'wall_{i}', b, SHELF_HEIGHT, '0.5 0.5 0.5 1')
                    for i, b in enumerate(wall_boxes(len(rows[0]), len(rows), cell)))
    shelf_model = (f'<model name="shelves"><static>true</static><link name="link">{shelves}</link></model>'
                   if shelves else '')
    # 화면(gzclient)을 처음부터 "위에서 내려다보고, 화면 위 = +y(북), 오른쪽 = +x"로 연다 → 2D 그림(0번 줄이 위)과 같은 모양.
    # 이게 없으면 Gazebo 기본 카메라 방향이라 지도가 90° 돌아가 보인다(2026-11-26 사용자 확인).
    # 자세: yaw 90°(카메라 앞 = +y) 뒤 pitch 90° 가까이(아래를 봄) → 화면 위 = +y. pitch를 정확히 90°로 두면 orbit 조작이 튀어서 살짝 덜 숙인다.
    wm, hm = len(rows[0]) * cell, len(rows) * cell
    cam_h = 1.8 * max(wm, hm) + 0.5
    gui = (f'<gui fullscreen="0"><camera name="user_camera"><pose>{wm / 2:.3f} {hm / 2:.3f} {cam_h:.2f} 0 1.5697 1.5708</pose>'
           f'<view_controller>orbit</view_controller></camera></gui>')
    return f'''<?xml version="1.0"?>
<!-- mapf_gazebo/grid_to_world.py가 시나리오에서 만든 파일: "{scenario['name']}" — 손으로 고치지 말 것 -->
<sdf version="1.6">
  <world name="mapf_{scenario['name']}">
    {gui}
    <include><uri>model://ground_plane</uri></include>
    <include><uri>model://sun</uri></include>
    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
      <ros><namespace>/gazebo</namespace></ros>
      <update_rate>10.0</update_rate>
    </plugin>
    {shelf_model}
    <model name="walls"><static>true</static><link name="link">{walls}</link></model>
  </world>
</sdf>
'''


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('scenario')
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    with open(a.scenario) as f:
        sc = json.load(f)
    with open(a.out, 'w') as f:
        f.write(world_xml(sc))
    print(a.out)


if __name__ == '__main__':
    main()

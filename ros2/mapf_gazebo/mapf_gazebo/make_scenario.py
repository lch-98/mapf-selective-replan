"""3D 시나리오 JSON 만들기 — 7-1 (형식: docs/31 31.19).

    ros2 run mapf_gazebo make_scenario --map corridor --size 10 --robots 6 --seed 1 --out corridor10_r6_s1.json

- 지도: 벤치마크와 같은 규칙(grid.make_map_rows)을 작은 크기에 적용.
- 로봇 배치: 출발 칸끼리, 목적지 칸끼리 겹치지 않게 무작위(벤치마크·GUI와 같은 규칙). 코어 plan()이 성공하는 배치가 나올 때까지 다시 뽑는다.
- 로봇별 최고 선속도·각속도, 작업 시간 흔들림 배율: 31.13 D11 범위에서 무작위.
- 파이썬 random은 시드가 같으면 Windows·Ubuntu에서 같은 결과(C++ std::shuffle과 다름, PROGRESS 2026-11-25).
- 처음 방향: 처음 계획의 첫 이동 방향을 바라보게(없으면 0°).
"""
import argparse
import json
import math
import random

from mapf_gazebo.core import load_mapf_py
from mapf_gazebo.grid import CELL_SIZE, make_map_rows, move_yaw

FORMAT = 'mapf_scenario_v1'


def make(kind, size, n, seed, service=(0, 0), v_range=(0.12, 0.22), w_range=(0.8, 1.5),
         service_factor=(0.8, 1.2), dt_nom=3.5, max_attempts=200):
    mapf_py = load_mapf_py()
    rows = make_map_rows(kind, size)
    m = mapf_py.Map(rows)
    free = [mapf_py.Cell(x, y) for y in range(size) for x in range(size) if m.is_passable(x, y)]
    place_rng = random.Random(f'{seed}-place')     # 배치
    service_rng = random.Random(f'{seed}-service')  # 작업 스텝 (배치와 따로: 작업 시간을 바꿔도 배치는 같음)
    robot_rng = random.Random(f'{seed}-robot')      # 속도·흔들림 (배치와 따로)

    for attempt in range(1, max_attempts + 1):
        starts, goals = list(free), list(free)
        place_rng.shuffle(starts)
        place_rng.shuffle(goals)
        services = [service_rng.randint(*service) if service[1] > 0 else 0 for _ in range(n)]
        agents = [mapf_py.Agent(i, starts[i], goals[i], services[i]) for i in range(n)]
        plan = mapf_py.PrioritizedPlanner(m).plan(agents)
        if plan is not None:
            break
    else:
        raise RuntimeError(f'{max_attempts}번 안에 처음 계획이 되는 배치를 못 찾음')

    robots = []
    for a in agents:
        path = plan[a.id]
        yaw = 0.0
        for p, q in zip(path, path[1:]):
            y_ = move_yaw(q.x - p.x, q.y - p.y)
            if y_ is not None:
                yaw = y_
                break
        robots.append({
            'id': a.id, 'name': f'robot_{a.id + 1}',
            'start': [a.start.x, a.start.y], 'goal': [a.goal.x, a.goal.y],
            'start_yaw_deg': round(math.degrees(yaw), 1),
            'service_steps': a.service_time,
            'max_linear': round(robot_rng.uniform(*v_range), 2),
            'max_angular': round(robot_rng.uniform(*w_range), 2),
            'service_factor': round(robot_rng.uniform(*service_factor), 2),
        })
    return {
        'format': FORMAT,
        'name': f'{kind}{size}_r{n}_s{seed}',
        'map_kind': kind,
        'cell_size': CELL_SIZE,
        'dt_nom_s': dt_nom,
        'seed': seed,
        'placement_attempts': attempt,
        'map': rows,
        'robots': robots,
        'obstacles': [],
        'initial_plan': {str(a.id): [[c.x, c.y, c.t] for c in plan[a.id]] for a in agents},
    }


def main():
    ap = argparse.ArgumentParser(description='3D 시나리오 JSON 만들기 (docs/31 31.19)')
    ap.add_argument('--map', choices=['corridor', 'open'], default='corridor')
    ap.add_argument('--size', type=int, default=10)
    ap.add_argument('--robots', type=int, default=6)
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--service', type=int, nargs=2, default=[0, 0], metavar=('LO', 'HI'), help='작업 스텝 범위')
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    sc = make(a.map, a.size, a.robots, a.seed, tuple(a.service))
    with open(a.out, 'w') as f:
        json.dump(sc, f, ensure_ascii=False, indent=1)
    print(f"{a.out}: {sc['name']}, 배치 {sc['placement_attempts']}번째에 처음 계획 성공")


if __name__ == '__main__':
    main()

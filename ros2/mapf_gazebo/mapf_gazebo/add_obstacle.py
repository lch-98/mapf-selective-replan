"""시나리오에 장애물 하나를 넣은 새 시나리오를 만든다 — 7-4 (docs/31 31.5, 31.23).

    ros2 run mapf_gazebo add_obstacle corridor10_r6_s1 --spawn 2.0 --out scenarios/corridor10_r6_s1_ob1.json

고르는 규칙(2D 벤치마크의 "길 위 장애물"과 같은 뜻, 31.5):
  - 처음 계획(initial_plan)에서 로봇이 지나가는 칸 중, 어느 로봇의 출발 칸·목적지 칸도 아닌 칸
  - 그 칸을 쓰는 모든 로봇이 계획 번호 3 이후에야 들어옴(생성 시각에 로봇이 그 칸에 있거나 들어가는 중이 아니게)
  - 그중 지나가는 로봇이 가장 많은 칸(같으면 가장 먼저 쓰이는 칸, 그다음 행·열이 작은 칸)
생성 시각(spawn_time_s)에 그 칸을 누가 쓰고 있으면 실행기가 비워질 때까지 미룬다.
"""
import argparse
import json

from mapf_gazebo import scenario_io


def pick(sc, min_index=3):
    starts = {tuple(r['start']) for r in sc['robots']}
    goals = {tuple(r['goal']) for r in sc['robots']}
    first = {}  # 칸 → {로봇: 처음 들어오는 번호}
    for rid, path in sc['initial_plan'].items():
        for x, y, t in path:
            first.setdefault((x, y), {}).setdefault(rid, t)
    cands = []
    for c, users in first.items():
        if c in starts or c in goals or min(users.values()) < min_index:
            continue
        cands.append((-len(users), min(users.values()), c[1], c[0], c))
    if not cands:
        raise RuntimeError('조건에 맞는 칸이 없음')
    cands.sort()
    return cands[0][4], -cands[0][0]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('scenario')
    ap.add_argument('--spawn', type=float, default=2.0, help='생성 시각(실행 시작 뒤 초)')
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    sc = scenario_io.load(a.scenario)
    cell, users = pick(sc)
    sc['obstacles'] = [{'cell': [cell[0], cell[1]], 'spawn_time_s': a.spawn}]
    sc['name'] = sc['name'] + '_ob1'
    sc['note'] = (f"7-4: 처음 계획에서 로봇 {users}대가 지나가는 {cell[1]}행 {cell[0]}열에 {a.spawn}초에 상자 생성(add_obstacle). "
                  + sc.get('note', ''))
    with open(a.out, 'w') as f:
        json.dump(sc, f, ensure_ascii=False, indent=1)
    print(f"{a.out}: 장애물 {cell[1]}행 {cell[0]}열 (로봇 {users}대가 지나감), {a.spawn}초")


if __name__ == '__main__':
    main()

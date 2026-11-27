"""같은 재계획 입력에서 두 방법 비교 — 7-6, docs/32.

3D 실행은 매번 조금씩 달라서(LiDAR가 보인 칸 수, 로봇 도착 순서) 우리 방법 실행과 전체 재계획 실행의 재계획 입력이 다를 수 있다.
그래서 실행이 저장한 replan_<n>.json마다 같은 입력에 우리 방법(replan)과 전체 재계획(full_replan)을 둘 다 2D 파이썬으로 돌려
계획상 끝 스텝(로봇마다 작업까지 끝나는 번호)의 합·최댓값, 길이 바뀐 로봇 수, 성공을 비교한다.

    ros2 run mapf_gazebo same_input_compare results/7_6/base [다른 폴더 ...] [--csv 모음.csv]
"""
import argparse
import csv
import glob
import json
import os

from mapf_gazebo.core import load_mapf_py
from mapf_gazebo.check_replan_input import run


def finish_steps(mp, d, paths):
    """로봇마다 끝 번호: 마지막으로 칸이 바뀐 번호. 작업이 남은 로봇(근무표 start < end)은 작업 끝(end)까지.
    (경로 길이로 세면 안 된다 — 우리 방법은 고정된 로봇 경로를 계획 끝까지 채워 돌려주고 전체 재계획은 안 채운다.)"""
    agents = [mp.Agent(a['id'], mp.Cell(*a['start']), mp.Cell(*a['goal']), a['service_time']) for a in d['agents']]
    prev = {int(k): mp.TaskWindow(v[0], v[1]) for k, v in d['previous_schedule'].items()}
    p = {k: [mp.SpaceTimeCell(x, y, t) for x, y, t in v] for k, v in paths.items()}
    sched = mp.update_schedule(agents, prev, p, d['current_time'])
    out = {}
    for a in agents:
        path = paths[a.id]
        arrive = len(path) - 1
        while arrive > 0 and path[arrive - 1][:2] == path[-1][:2]:
            arrive -= 1
        w = sched[a.id]
        out[a.id] = max(arrive, w.end) if w.end > w.start else arrive  # 작업이 남은 로봇만 근무표 끝을 봄
    return out


def dedup(path):
    out = []
    for x, y, _ in path:
        if not out or out[-1] != (x, y):
            out.append((x, y))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('roots', nargs='+')
    ap.add_argument('--csv')
    a = ap.parse_args()
    mp = load_mapf_py()
    rows = []
    for root in a.roots:
        for f in sorted(glob.glob(os.path.join(root, '*', 'replan_0.json'))):
            d = json.load(open(f))
            prev = {int(k): v for k, v in d['previous_paths'].items()}
            row = {'group': os.path.basename(os.path.normpath(root)), 'run': os.path.basename(os.path.dirname(f))}
            for m in ('ours', 'full'):
                paths, info = run(mp, d, m)
                row[f'{m}_ok'] = paths is not None
                if paths is None:
                    row[f'{m}_sum'] = row[f'{m}_max'] = row[f'{m}_changed'] = ''
                    continue
                fs = finish_steps(mp, d, paths)
                row[f'{m}_sum'] = sum(fs.values())
                row[f'{m}_max'] = max(fs.values())
                row[f'{m}_changed'] = sum(1 for k in paths if dedup(paths[k]) != dedup(prev[k]))
                if m == 'ours':
                    row['ours_tier'] = info.get('escalation_tier')
            rows.append(row)
    cols = ['group', 'run', 'ours_ok', 'full_ok', 'ours_sum', 'full_sum', 'ours_max', 'full_max', 'ours_changed', 'full_changed', 'ours_tier']
    print('| ' + ' | '.join(cols) + ' |')
    print('|' + '---|' * len(cols))
    for r in rows:
        print('| ' + ' | '.join(str(r.get(c, '')) for c in cols) + ' |')
    if a.csv and rows:
        with open(a.csv, 'w', newline='') as fh:
            w = csv.DictWriter(fh, fieldnames=cols)
            w.writeheader()
            w.writerows(rows)


if __name__ == '__main__':
    main()

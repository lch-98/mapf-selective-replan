"""3D 실행이 저장한 재계획 입력(replan_<n>.json)을 2D 파이썬(mapf_py)으로 다시 돌려 같은 결과인지 확인 — 31.6, 31.12(7-4 기준).

    ros2 run mapf_gazebo check_replan_input results/7_5/hold/corridor10_r6_s1_hold_full/replan_0.json [--both]

--both면 우리 방법(replan)과 전체 재계획(full_replan)을 둘 다 돌려 결과를 보여 준다(어느 쪽으로 저장됐든).
같은 C++ 코어를 부르므로 경로가 칸·번호까지 같아야 정상이다(다르면 3D 연결이 틀림).
"""
import argparse
import json
import sys

from mapf_gazebo.core import load_mapf_py


def run(mp, d, method):
    m = mp.Map(d['map'])
    agents = [mp.Agent(a['id'], mp.Cell(*a['start']), mp.Cell(*a['goal']), a['service_time']) for a in d['agents']]
    prev = {int(k): [mp.SpaceTimeCell(x, y, t) for x, y, t in v] for k, v in d['previous_paths'].items()}
    sched = {int(k): mp.TaskWindow(v[0], v[1]) for k, v in d['previous_schedule'].items()}
    obs = [mp.Cell(x, y) for x, y in d['new_obstacles']]
    c = d['config']
    cfg = mp.ReplanConfig()
    cfg.order = mp.ReplanOrder.FIXED_FIRST if c['order'] == 'fixed_first' else mp.ReplanOrder.PRIORITY
    cfg.check_reachability = c['check_reachability']
    cfg.rescue_selection = (mp.RescueSelection.SHORTEST_PATH_CONFLICTS if c['rescue_selection'] == 'shortest_path_conflicts'
                            else mp.RescueSelection.BLOCKED_CELLS)
    cfg.max_escalation_tiers = c['max_escalation_tiers']
    cfg.order_restarts = c['order_restarts']
    pl = mp.PrioritizedPlanner(m, mp.AStarConfig(), cfg)
    if method == 'ours':
        r = pl.replan(agents, prev, sched, obs, d['current_time'])
        if r is None:
            return None, {}
        return {k: [[c.x, c.y, c.t] for c in v] for k, v in r.paths.items()}, {
            'escalation_tier': r.escalation_tier, 'replanned_ids': list(r.replanned_ids), 'rescued_ids': list(r.rescued_ids),
            'order_restart': r.order_restart}
    r = pl.full_replan(agents, prev, sched, obs, d['current_time'])
    return (None if r is None else {k: [[c.x, c.y, c.t] for c in v] for k, v in r.items()}), {}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('files', nargs='+')
    ap.add_argument('--both', action='store_true')
    a = ap.parse_args()
    mp = load_mapf_py()
    bad = 0
    for f in a.files:
        d = json.load(open(f))
        got, info = run(mp, d, d['method'])
        want = {int(k): v for k, v in d['result']['paths'].items()} if d['result']['success'] else None
        same = got == want
        bad += 0 if same else 1
        print(f"{f}: [{d['method']}] 3D {'성공' if want else '실패'} / 2D 파이썬 {'성공' if got else '실패'} → "
              f"{'같음' if same else '다름!'} {info if info else ''}")
        if a.both:
            other = 'full' if d['method'] == 'ours' else 'ours'
            g2, i2 = run(mp, d, other)
            print(f"   같은 입력을 [{other}]로: {'성공' if g2 else '실패'} {i2 if i2 else ''}")
    sys.exit(1 if bad else 0)


if __name__ == '__main__':
    main()

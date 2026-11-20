"""벤치마크 코드와 독립된 검사기.
시나리오를 새로 만들고(다른 난수·다른 장애물 규칙), 세 방법의 결과 경로를 직접 검사한다:
  - 지나온 길(current_time까지)이 그대로인가
  - 한 스텝에 제자리 또는 상하좌우 한 칸, 벽 아님
  - current_time 이후 새 장애물 칸을 밟지 않음
  - 같은 시각 같은 칸 / 자리 맞바꾸기 충돌 없음 (도착 후에는 목적지에 머문다고 봄)
  - 목적지에서 끝남
그리고 선택적 재계획이 안전망(escalation_tier == -1)을 썼는지 센다."""
import random
import sys
import time
from pathlib import Path

sys.path.insert(0, r"C:\Users\Lenovo\Desktop\Path_Planning_Core_\python")
from _pathsetup import add_mapf_py_to_path  # noqa: E402

add_mapf_py_to_path()
import mapf_py  # noqa: E402


def make_map(kind, size):
    rows = [["."] * size for _ in range(size)]
    start, step = (6, 8) if kind == "open" else (2, 4)
    for by in range(start, size - 3, step):
        for bx in range(start, size - 3, step):
            for y in range(by, by + 3):
                for x in range(bx, bx + 3):
                    rows[y][x] = "#"
    return mapf_py.Map(["".join(r) for r in rows])


def pos(path, t):
    c = path[t] if t < len(path) else path[-1]
    return (c.x, c.y)


def validate(game_map, agents, before, after, obstacles, t0):
    errors = []
    obs = {(c.x, c.y) for c in obstacles}
    horizon = max(len(p) for p in after.values()) + 1
    for a in agents:
        p = after[a.id]
        if any(c.t != i for i, c in enumerate(p)):
            errors.append(f"{a.id}: 시각이 0부터 이어지지 않음")
            continue
        for t in range(t0 + 1):
            if pos(p, t) != pos(before[a.id], t):
                errors.append(f"{a.id}: 지나온 길이 바뀜 t{t}")
                break
        for t in range(1, len(p)):
            (x0, y0), (x1, y1) = pos(p, t - 1), pos(p, t)
            if abs(x0 - x1) + abs(y0 - y1) > 1 or not game_map.is_passable(x1, y1):
                errors.append(f"{a.id}: 순간이동/벽 t{t}")
                break
        for t in range(t0, horizon):
            if pos(p, t) in obs:
                errors.append(f"{a.id}: 장애물 밟음 t{t}")
                break
        if pos(p, len(p) - 1) != (a.goal.x, a.goal.y):
            errors.append(f"{a.id}: 목적지에서 안 끝남")
    for t in range(t0, horizon):
        seen = {}
        for a in agents:
            here = pos(after[a.id], t)
            if here in seen:
                errors.append(f"충돌: {seen[here]}와 {a.id} t{t} {here}")
            seen[here] = a.id
            if t > t0:
                prev = pos(after[a.id], t - 1)
                for b in agents:
                    if b.id < a.id and pos(after[b.id], t - 1) == here and pos(after[b.id], t) == prev and here != prev:
                        errors.append(f"맞바꾸기: {a.id}와 {b.id} t{t}")
    return errors


def make_scenario(game_map, n, rng):
    free = [mapf_py.Cell(x, y) for y in range(game_map.height()) for x in range(game_map.width())
            if game_map.is_passable(x, y)]
    while True:
        starts, goals = rng.sample(free, n), rng.sample(free, n)
        agents = [mapf_py.Agent(i, starts[i], goals[i]) for i in range(n)]
        plan = mapf_py.PrioritizedPlanner(game_map).plan(agents)
        if plan is None:
            continue
        goal_set = {(a.goal.x, a.goal.y) for a in agents}
        start_set = {(a.start.x, a.start.y) for a in agents}
        # 장애물: 무작위 로봇 하나의 경로 중간 칸(누구의 출발·목적지도 아닌 칸), 그 직전 시각에 생김
        movers = [a for a in agents if len(plan[a.id]) > 3]
        if not movers:
            continue
        a = rng.choice(movers)
        cands = [c for c in plan[a.id][1:-1] if (c.x, c.y) not in goal_set and (c.x, c.y) not in start_set]
        if not cands:
            continue
        c = rng.choice(cands)
        t0 = c.t - 1
        if any(pos(plan[b.id], t0) == (c.x, c.y) for b in agents):
            continue
        return agents, plan, [mapf_py.Cell(c.x, c.y)], t0


def config(method):
    cfg = mapf_py.ReplanConfig()
    if method == "ours":
        # 2026-11-20부터 우리 방법 = Tier 2 + 안전망 ②(30장 30.6). 그 전 검사 결과(118/120 등)는 Tier 1 기준.
        cfg.order = mapf_py.ReplanOrder.FIXED_FIRST
        cfg.check_reachability = True
        cfg.rescue_selection = mapf_py.RescueSelection.SHORTEST_PATH_CONFLICTS
        cfg.max_escalation_tiers = 2
        cfg.order_restarts = 3
    return cfg


def main():
    rng = random.Random(20261117)  # 벤치마크와 다른 씨앗
    trials = int(sys.argv[1]) if len(sys.argv) > 1 else 25
    print("지도/로봇수      | 전체 재계획 성공 | 원래 방식: 스스로/안전망/실패 | 우리 방법: 스스로/안전망/실패 | 검사 오류 | 시간 중앙값 ms 전체/원래/우리")
    total_errors = 0
    for kind in ("open", "corridor"):
        game_map = make_map(kind, 64)
        for n in (100, 200):
            stat = {"full": 0, "orig": [0, 0, 0], "ours": [0, 0, 0]}
            times = {"full": [], "orig": [], "ours": []}
            errs = 0
            for _ in range(trials):
                agents, plan, obstacles, t0 = make_scenario(game_map, n, rng)
                sched = mapf_py.schedule_from_plan(agents, plan)
                s = time.perf_counter()
                full = mapf_py.PrioritizedPlanner(game_map).full_replan(agents, plan, sched, obstacles, t0)
                times["full"].append((time.perf_counter() - s) * 1000)
                if full is not None:
                    stat["full"] += 1
                    errs += len(validate(game_map, agents, plan, full, obstacles, t0))
                for key, method in (("orig", "legacy"), ("ours", "ours")):
                    planner = mapf_py.PrioritizedPlanner(game_map, replan_config=config(method))
                    s = time.perf_counter()
                    r = planner.replan(agents, plan, sched, obstacles, t0)
                    times[key].append((time.perf_counter() - s) * 1000)
                    if r is None:
                        stat[key][2] += 1
                        continue
                    stat[key][0 if r.escalation_tier >= 0 else 1] += 1
                    e = validate(game_map, agents, plan, r.paths, obstacles, t0)
                    if e and errs == 0:
                        print("   예:", e[:3])
                    errs += len(e)
            total_errors += errs
            med = lambda v: sorted(v)[len(v) // 2]
            print(f"64x64 {kind:8s} {n:3d} | {stat['full']:2d}/{trials} | "
                  f"{stat['orig'][0]:2d}/{stat['orig'][1]:2d}/{stat['orig'][2]:2d} | "
                  f"{stat['ours'][0]:2d}/{stat['ours'][1]:2d}/{stat['ours'][2]:2d} | {errs} | "
                  f"{med(times['full']):.0f}/{med(times['orig']):.0f}/{med(times['ours']):.0f}")
    print("검사 오류 합계:", total_errors)


if __name__ == "__main__":
    main()

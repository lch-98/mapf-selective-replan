"""우리 방법이 안전망을 쓰거나 실패한 시나리오의 원인 분석(2026-11-20, 30장).

벤치마크를 `--dump-failures DIR`로 돌리면 그런 시나리오가 DIR/*.json으로 저장된다. 이 스크립트는 같은 장면을
파이썬(mapf_py)으로 그대로 다시 돌려서 아래를 확인한다.

  1. 재현: 우리 방법(고정 먼저 + 도달 확인 + ③ + Tier 1)이 같은 결과(안전망/실패)를 내는가
  2. 길이 아예 끊겼나: 영향받은 로봇이 "다른 로봇이 하나도 없어도" 목적지에 갈 수 있는가(BFS)
  3. 더 넓히면 풀리나: Tier를 2~5로 늘리면 / 비켜 달라기(request_yield)를 켜면
  4. 순서만 바꾸면 풀리나: 전체 재계획을 무작위 우선순위로 여러 번(PP 무작위 재시작)
  5. 시간 상한 때문인가: max_timestep 256 → 512
  6. 길 위에 "작업 중"인 로봇이 서 있나(못 비킴): 막힌 로봇의 BFS 최단 길 위에 지금 작업 중인 로봇

사용: python tools/analyze_failures.py DIR [DIR ...]
"""
import glob
import json
import os
import random
import sys
from collections import deque

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "python"))
from _pathsetup import add_mapf_py_to_path  # noqa: E402

add_mapf_py_to_path()
import mapf_py  # noqa: E402

C, ST = mapf_py.Cell, mapf_py.SpaceTimeCell


def load(path):
    with open(path, encoding="utf-8") as f:
        d = json.load(f)
    m = mapf_py.Map([row.replace("@", "#") for row in d["map"]])  # 예전 덤프는 벽을 '@'로 적음
    agents = []
    for aid, sx, sy, gx, gy, svc in d["agents"]:
        a = mapf_py.Agent(aid, C(sx, sy), C(gx, gy))
        a.service_time = svc
        agents.append(a)
    paths = {int(k): [ST(x, y, t) for x, y, t in v] for k, v in d["paths"].items()}
    obstacles = [C(x, y) for x, y in d["obstacles"]]
    return d, m, agents, paths, obstacles, d["current_time"]


def pos_at(path, t):
    if t >= path[-1].t:
        return path[-1]
    for c in path:
        if c.t == t:
            return c
    return path[-1]


def bfs(m, start, goal, blocked):
    """벽 + blocked만 보고 start→goal 최단 길(칸 목록). 없으면 None."""
    blocked = {(c.x, c.y) for c in blocked}
    prev = {(start.x, start.y): None}
    q = deque([(start.x, start.y)])
    while q:
        x, y = q.popleft()
        if (x, y) == (goal.x, goal.y):
            route = []
            cur = (x, y)
            while cur is not None:
                route.append(cur)
                cur = prev[cur]
            return route[::-1]
        for nx, ny in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
            if 0 <= nx < m.width() and 0 <= ny < m.height() and m.is_passable(nx, ny) \
                    and (nx, ny) not in blocked and (nx, ny) not in prev:
                prev[(nx, ny)] = (x, y)
                q.append((nx, ny))
    return None


def ours(tiers=1, request_yield=False):
    c = mapf_py.ReplanConfig()
    c.order = mapf_py.ReplanOrder.FIXED_FIRST
    c.check_reachability = True
    c.rescue_selection = mapf_py.RescueSelection.SHORTEST_PATH_CONFLICTS
    c.max_escalation_tiers = tiers
    c.request_yield = request_yield
    return c


def analyze(path, shuffles=60):
    d, m, agents, paths, obstacles, t = load(path)
    sched = mapf_py.schedule_from_plan(agents, paths)
    by_id = {a.id: a for a in agents}
    out = {"file": os.path.basename(path), "n": len(agents), "t": t, "full_ok": d["full_replan_ok"],
           "sel_ok": d["selective_replan_ok"]}

    r = mapf_py.PrioritizedPlanner(m, replan_config=ours()).replan(agents, paths, sched, obstacles, t)
    out["reproduced"] = (r is not None) == bool(d["selective_replan_ok"]) and (r is None or r.escalation_tier == -1)

    affected = [a.id for a in agents if mapf_py.PrioritizedPlanner.path_hits_obstacle(paths[a.id], obstacles, t)]
    out["affected"] = affected
    working = [a.id for a in agents if mapf_py.task_state_at(sched[a.id], t) == mapf_py.TaskState.WORKING]
    out["working_n"] = len(working)

    # 2. 길이 끊겼나 + 6. 길 위의 작업 중 로봇
    unreachable, working_on_route = [], {}
    for aid in affected:
        here = pos_at(paths[aid], t)
        route = bfs(m, here, by_id[aid].goal, obstacles)
        if route is None:
            unreachable.append(aid)
            continue
        cells = set(route[1:])
        blockers = [w for w in working if (pos_at(paths[w], t).x, pos_at(paths[w], t).y) in cells]
        if blockers:
            working_on_route[aid] = blockers
    out["unreachable"] = unreachable
    out["working_on_route"] = working_on_route

    # 3. 더 넓히면
    for tiers in (2, 3, 5):
        rr = mapf_py.PrioritizedPlanner(m, replan_config=ours(tiers)).replan(agents, paths, sched, obstacles, t)
        out[f"tiers{tiers}"] = None if rr is None else rr.escalation_tier
    ry = mapf_py.PrioritizedPlanner(m, replan_config=ours(1, True)).replan(agents, paths, sched, obstacles, t)
    out["yield"] = None if ry is None else ry.escalation_tier

    # 4. 전체 재계획을 무작위 우선순위로
    rng = random.Random(1)
    solved = 0
    for _ in range(shuffles):
        order = agents[:]
        rng.shuffle(order)
        if mapf_py.PrioritizedPlanner(m).full_replan(order, paths, sched, obstacles, t) is not None:
            solved += 1
    out["full_random_order_solved"] = f"{solved}/{shuffles}"

    # 5. 시간 상한
    big = mapf_py.AStarConfig()
    big.max_timestep = 512
    out["full_t512"] = mapf_py.PrioritizedPlanner(m, big).full_replan(agents, paths, sched, obstacles, t) is not None
    rb = mapf_py.PrioritizedPlanner(m, big, ours()).replan(agents, paths, sched, obstacles, t)
    out["ours_t512"] = None if rb is None else rb.escalation_tier
    return out


def main():
    files = []
    for d in sys.argv[1:]:
        files += sorted(glob.glob(os.path.join(d, "*.json")))
    for f in files:
        res = analyze(f)
        print(os.path.basename(os.path.dirname(f)), json.dumps(res, ensure_ascii=False))


if __name__ == "__main__":
    main()

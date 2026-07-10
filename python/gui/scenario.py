# ─────────────────────────────────────────────────────────────────
# python/gui/scenario.py
#
# tools/benchmark.cpp의 collect_free_cells / make_random_agents /
# find_solvable_scenario_and_run(초기 plan() 재시도 부분만)을 파이썬으로
# 포팅한다. 실제 알고리즘(PBS.plan)은 여기서 다시 짜지 않고 바인딩된
# mapf_py.PBS를 그대로 호출한다 — 이 파일이 하는 일은 "무작위 시나리오를
# 만드는 것"뿐이다.
#
# make_obstacles_that_actually_block류의 "장애물이 실제로 막는지 보장하는"
# 로직은 포팅하지 않는다 — GUI의 장애물은 사람이 마우스로 직접 고르므로
# 합성할 필요가 없다(벤치마크는 통계 샘플링을 위해 "의미 있는" 장애물을
# 합성해야 했지만, GUI는 그럴 필요가 없다).
# ─────────────────────────────────────────────────────────────────
import random

import mapf_py


def collect_free_cells(game_map: "mapf_py.Map") -> list:
    """맵의 벽이 아닌 모든 칸을 모은다."""
    cells = []
    for y in range(game_map.height()):
        for x in range(game_map.width()):
            if game_map.is_passable(x, y):
                cells.append(mapf_py.Cell(x, y))
    return cells


def make_random_agents(free_cells: list, num_agents: int, rng: random.Random) -> list:
    """빈 칸 중에서 서로 겹치지 않는 (start, goal) 쌍을 num_agents개 뽑는다.

    시작점끼리는 서로 안 겹치고, 목적지끼리도 서로 안 겹치지만, 한 로봇
    자신의 start==goal은 허용한다(tools/benchmark.cpp와 동일한 규칙).
    """
    shuffled_starts = list(free_cells)
    shuffled_goals = list(free_cells)
    rng.shuffle(shuffled_starts)
    rng.shuffle(shuffled_goals)

    agents = []
    for i in range(num_agents):
        agents.append(mapf_py.Agent(i, shuffled_starts[i], shuffled_goals[i]))
    return agents


def find_solvable_scenario(
    game_map: "mapf_py.Map", num_agents: int, rng: random.Random, max_attempts: int = 200
):
    """초기 PBS.plan()이 성공하는 무작위 배치를 찾을 때까지 재시도한다.

    tools/benchmark.cpp의 find_solvable_scenario_and_run에서 장애물 관련
    부분을 뺀 것과 같다 — GUI가 열리자마자 절대 풀 수 없는 무작위 배치로
    시작하지 않도록 보장한다.

    Returns:
        (agents, initial_paths) 튜플. max_attempts번 안에 못 찾으면
        RuntimeError를 던진다(극히 드문 경우 — 안전장치).
    """
    free_cells = collect_free_cells(game_map)

    for _attempt in range(1, max_attempts + 1):
        agents = make_random_agents(free_cells, num_agents, rng)
        pbs = mapf_py.PBS(game_map)
        initial = pbs.plan(agents)
        if initial is not None:
            return agents, initial

    raise RuntimeError(
        f"{max_attempts}번 재시도해도 초기 계획에 성공하는 배치를 못 찾았습니다 "
        f"(num_agents={num_agents})."
    )

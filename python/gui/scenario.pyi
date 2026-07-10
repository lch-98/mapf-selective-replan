# ─────────────────────────────────────────────────────────────────
# python/gui/scenario.pyi
#
# scenario.py의 공개 인터페이스. tools/benchmark.cpp의 collect_free_cells /
# make_random_agents / find_solvable_scenario_and_run(초기 plan() 재시도
# 부분만)을 파이썬으로 포팅한 것 — "무작위 시나리오를 만드는 것"만 담당하고,
# 실제 경로 계산(PBS.plan)은 바인딩된 mapf_py.PBS를 그대로 호출한다.
# ─────────────────────────────────────────────────────────────────
import random
from typing import List, Tuple

import mapf_py

def collect_free_cells(game_map: "mapf_py.Map") -> List["mapf_py.Cell"]:
    """맵의 벽이 아닌 모든 칸을 (y,x 순서로 스캔하며) 모은다."""
    ...

def make_random_agents(
    free_cells: List["mapf_py.Cell"], num_agents: int, rng: random.Random
) -> List["mapf_py.Agent"]:
    """빈 칸 중에서 서로 겹치지 않는 (start, goal) 쌍을 num_agents개
    무작위로 뽑는다. 시작점끼리는 서로 안 겹치고 목적지끼리도 서로 안
    겹치지만, 한 로봇 자신의 start==goal은 허용한다."""
    ...

def find_solvable_scenario(
    game_map: "mapf_py.Map",
    num_agents: int,
    rng: random.Random,
    max_attempts: int = 200,
) -> Tuple[List["mapf_py.Agent"], "mapf_py.PBSResult"]:
    """초기 PBS.plan()이 성공하는 무작위 배치를 찾을 때까지 재시도한다.
    성공하면 (agents, initial_paths) 튜플을 반환한다. max_attempts번 안에
    못 찾으면(극히 드문 경우) RuntimeError를 던진다."""
    ...

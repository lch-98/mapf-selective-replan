# ─────────────────────────────────────────────────────────────────
# python/smoke_test.py
#
# GUI를 붙이기 전에, 바인딩 자체가 제대로 로드되고 동작하는지만 빠르게
# 확인하는 스크립트. pygame 렌더링 문제와 바인딩 문제를 분리해서
# 디버깅하려는 목적 — 여기가 실패하면 GUI 쪽을 볼 필요도 없다.
# ─────────────────────────────────────────────────────────────────
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from _pathsetup import add_mapf_py_to_path  # noqa: E402

add_mapf_py_to_path()
import mapf_py  # noqa: E402


def check(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(f"FAILED: {message}")
    print(f"  ok: {message}")


def main() -> None:
    print("[1] Map(rows) 생성 + is_passable 확인")
    m = mapf_py.Map(["...", ".#.", "..."])
    check(m.width() == 3 and m.height() == 3, "3x3 맵 크기")
    check(m.is_passable(1, 1) is False, "(1,1)은 벽('#')이라 통행 불가")
    check(m.is_passable(0, 0) is True, "(0,0)은 빈 칸이라 통행 가능")

    print("[2] Cell/SpaceTimeCell 기본 동작")
    c1 = mapf_py.Cell(1, 2)
    c2 = mapf_py.Cell(1, 2)
    check(c1 == c2, "같은 좌표 Cell은 ==")
    check(hash(c1) == hash(c2), "같은 좌표 Cell은 같은 해시(set/dict 키로 사용 가능)")
    obstacle_set = {c1}
    check(c2 in obstacle_set, "Cell을 set에 넣고 다른 동일 인스턴스로 조회 가능")

    print("[3] Agent 생성자 (id, start, goal)")
    agents = [
        mapf_py.Agent(0, mapf_py.Cell(0, 0), mapf_py.Cell(2, 0)),
        mapf_py.Agent(1, mapf_py.Cell(0, 2), mapf_py.Cell(2, 2)),
    ]
    check(agents[0].id == 0 and agents[0].start.x == 0, "Agent 필드 접근")

    print("[4] PBS.plan() 성공 + PBSResult가 dict로 보이는지")
    m2 = mapf_py.Map(5, 5)
    pbs = mapf_py.PBS(m2)
    result = pbs.plan(agents)
    check(result is not None, "plan()이 None이 아님(성공)")
    check(isinstance(result, dict), "PBSResult가 파이썬 dict로 변환됨")
    check(set(result.keys()) == {0, 1}, "결과 dict의 키가 agent id와 일치")
    for agent_id, path in result.items():
        check(isinstance(path, list) and len(path) > 0, f"agent {agent_id}의 경로가 비어있지 않음")
        check(isinstance(path[0], mapf_py.SpaceTimeCell), "경로 원소가 SpaceTimeCell")

    print("[5] full_replan / replan 빈 장애물로 호출")
    full = pbs.full_replan(agents, result, [], 0)
    check(full is not None, "full_replan(장애물 없음)이 성공")

    selective = pbs.replan(agents, result, [], 0)
    check(selective is not None, "replan(장애물 없음)이 성공")
    check(selective.escalation_tier == 0, "장애물이 없으면 escalation_tier=0(아무도 안 건드림)")
    check(isinstance(selective.paths, dict), "ReplanResult.paths가 dict로 보임")

    print("[6] PBS.path_hits_obstacle 정적 메서드 확인")
    hits = mapf_py.PBS.path_hits_obstacle(result[0], [mapf_py.Cell(1, 0)], 0)
    check(isinstance(hits, bool), "path_hits_obstacle이 bool 반환")

    print("[7] std::invalid_argument -> Python ValueError 변환 확인")
    try:
        mapf_py.Map(-1, -1)
        raise AssertionError("FAILED: Map(-1,-1)이 예외를 던지지 않음")
    except ValueError:
        print("  ok: Map(-1,-1)이 ValueError를 던짐")

    print("\nSMOKE TEST PASSED")


if __name__ == "__main__":
    main()

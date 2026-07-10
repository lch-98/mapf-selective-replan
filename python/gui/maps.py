# ─────────────────────────────────────────────────────────────────
# python/gui/maps.py
#
# tools/benchmark_maps.hpp의 make_open_map()/make_corridor_map()과 레이아웃이
# 반드시 일치해야 한다(같은 stride/offset 상수를 그대로 옮김). 이 파일은
# tools/benchmark_maps.hpp를 직접 참조하지 않고 별도로 재구현한 것이므로,
# C++ 쪽 맵 생성 로직이 바뀌면 여기도 손으로 동기화해야 한다 — 공유 헤더로
# 뽑아내지 않은 이유는 그 로직이 15줄 안팎으로 작아서, 새 공유 헤더를
# tools/와 bindings/ 양쪽 CMakeLists.txt에 배선하는 것보다 중복이 훨씬
# 저렴하고 안전하기 때문이다(tools/benchmark.cpp/benchmark_maps.hpp는
# 이번 작업에서 수정하지 않는다는 제약과도 맞음).
# ─────────────────────────────────────────────────────────────────
import mapf_py

MAP_SIZE = 32


def make_open_map() -> "mapf_py.Map":
    """넓은 통로 맵: 3x3 기둥이 8칸 간격으로 3x3=9개 배치."""
    rows = [["." for _ in range(MAP_SIZE)] for _ in range(MAP_SIZE)]
    for by in range(6, MAP_SIZE - 3, 8):
        for bx in range(6, MAP_SIZE - 3, 8):
            for y in range(by, by + 3):
                for x in range(bx, bx + 3):
                    rows[y][x] = "#"
    return mapf_py.Map(["".join(row) for row in rows])


def make_corridor_map() -> "mapf_py.Map":
    """좁은 통로(교차로형) 맵: 3x3 선반이 4칸 간격으로 7x7=49개 배치."""
    rows = [["." for _ in range(MAP_SIZE)] for _ in range(MAP_SIZE)]
    for by in range(2, MAP_SIZE - 3, 4):
        for bx in range(2, MAP_SIZE - 3, 4):
            for y in range(by, by + 3):
                for x in range(bx, bx + 3):
                    rows[y][x] = "#"
    return mapf_py.Map(["".join(row) for row in rows])


MAP_FACTORIES = {
    "open": make_open_map,
    "corridor": make_corridor_map,
}

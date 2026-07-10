# ─────────────────────────────────────────────────────────────────
# python/gui/maps.pyi
#
# maps.py의 공개 인터페이스. tools/benchmark_maps.hpp의 make_open_map()/
# make_corridor_map()과 반드시 레이아웃이 일치해야 하는 Python 재구현체다
# (같은 stride/offset 상수를 그대로 옮김). C++ 쪽이 바뀌면 여기도 손으로
# 동기화해야 한다 — 공유 헤더로 뽑지 않은 이유는 maps.py 자체의 주석 참고.
# ─────────────────────────────────────────────────────────────────
from typing import Callable, Dict

import mapf_py

MAP_SIZE: int  # 두 맵 모두 정사각형이며 한 변의 칸 수(32).

def make_open_map() -> "mapf_py.Map":
    """넓은 통로 맵: 3x3 기둥이 8칸 간격으로 3x3=9개 배치. 로봇이 자유롭게
    우회할 공간이 넉넉하다."""
    ...

def make_corridor_map() -> "mapf_py.Map":
    """좁은 통로(교차로형) 맵: 3x3 선반이 4칸 간격으로 7x7=49개 배치.
    통로 폭이 어디서나 1칸뿐이라 로봇이 마주치면 비켜줄 공간이 거의 없다."""
    ...

# CLI --map 인자("open"|"corridor")로 맵 팩토리 함수를 찾을 때 쓰는 표.
MAP_FACTORIES: Dict[str, Callable[[], "mapf_py.Map"]]

# ─────────────────────────────────────────────────────────────────
# bindings/CMakeLists.txt
#
# mapf_core(C++ 코어)를 파이썬에서 그대로 호출할 수 있는 mapf_py 확장
# 모듈로 컴파일한다. pybind11_add_module은 최상위 CMakeLists.txt에서
# FetchContent로 받아온 pybind11 패키지가 제공하는 매크로 — 플랫폼별
# 확장자(Windows는 .pyd, Linux는 .so), 파이썬 include 경로, 링크 플래그를
# 전부 알아서 맞춰준다.
# ─────────────────────────────────────────────────────────────────

pybind11_add_module(mapf_py mapf_bindings.cpp)

target_link_libraries(mapf_py PRIVATE mapf_core)

# pybind11_add_module이 보통 최상위 CMAKE_CXX_STANDARD(17)를 그대로
# 물려받지만, 일부 제너레이터에서 표준 플래그를 덮어쓰는 경우가 있어
# 방어적으로 한 번 더 명시한다.
set_target_properties(mapf_py PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
)

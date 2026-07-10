# ─────────────────────────────────────────────────────────────────
# 최상위 CMakeLists.txt
#
# 이 파일이 하는 일 (한 줄씩):
#   1. CMake 자체의 최소 버전을 정한다 (이보다 낮은 CMake로는 빌드 못 함).
#   2. 프로젝트 이름과, 이 프로젝트가 C++로 작성됐다는 것을 선언한다.
#   3. 모든 하위 프로젝트(core, tests)가 공통으로 쓸 "C++ 표준 버전"을 정한다.
#   4. GoogleTest를 "설치 없이" 빌드 시점에 자동으로 받아온다 (FetchContent).
#   5. core/, tests/ 하위 폴더에 있는 각자의 CMakeLists.txt를 실행하라고 지시한다.
# ─────────────────────────────────────────────────────────────────

cmake_minimum_required(VERSION 3.15)

# PathPlanningCore: 프로젝트 이름. CXX = "이 프로젝트는 C++ 코드를 쓴다"는 선언.
project(PathPlanningCore CXX)

# ── C++ 표준 버전 ──────────────────────────────────────────────
# C++17을 쓰는 이유: std::optional, 구조적 바인딩 등을 핵심적으로 쓴다.
#   (00_implementation_plan.md §0.5에서 이미 점검한 이유와 동일.)
# CXX_STANDARD_REQUIRED ON: "17을 못 쓰면 그냥 에러를 내라"는 뜻.
#   (조용히 더 낮은 버전으로 떨어지는 것을 막는다 — 그러면 나중에
#   "이 컴파일러에선 왜 안 되지?" 같은 혼란스러운 에러가 생긴다.)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── GoogleTest를 빌드 시점에 자동으로 받아오기 ──────────────────
# FetchContent: "이 URL에서 소스코드를 받아서, 우리 프로젝트의 일부처럼
#   같이 빌드해라"는 CMake 표준 기능. Windows든 Ubuntu든 똑같이 동작한다 —
#   "GoogleTest를 미리 설치해두세요" 같은 사전 준비가 필요 없다.
include(FetchContent)
FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
)
# Windows(MSVC)에서 GoogleTest와 우리 코드가 "같은 방식으로" 런타임 라이브러리를
# 링크하도록 강제한다. 이걸 안 하면 "런타임 라이브러리 불일치" 링크 에러가 난다.
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# ── pybind11를 빌드 시점에 자동으로 받아오기 ─────────────────────
# GUI(bindings/, python/gui/)가 C++ 코어(mapf_core)를 파이썬에서 그대로
# 호출할 수 있게 해주는 바인딩 라이브러리. GoogleTest와 똑같은 FetchContent
# 패턴을 그대로 따른다 — pip 설치 같은 별도 준비 단계를 요구하지 않기 위해서다.
# find_package(Python3 ...)를 먼저 해두면, 이 PC에 파이썬이 여러 개 깔려있어도
# pybind11이 항상 같은 파이썬(Interpreter+Development 헤더가 있는 쪽)을 쓴다.
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

FetchContent_Declare(
    pybind11
    URL https://github.com/pybind/pybind11/archive/refs/tags/v2.13.6.zip
)
FetchContent_MakeAvailable(pybind11)

# ── 하위 폴더의 CMakeLists.txt를 실행 ────────────────────────────
add_subdirectory(core)
add_subdirectory(tests)
add_subdirectory(tools)
add_subdirectory(bindings)

# ─────────────────────────────────────────────────────────────────
# core/CMakeLists.txt
#
# 이 파일이 하는 일:
#   core/src/ 폴더의 .cpp 파일들을 모아서 "mapf_core"라는 이름의
#   라이브러리(.lib / .a)로 만든다. 라이브러리란, 직접 실행할 수는
#   없지만 다른 프로그램(테스트, 나중의 GUI 등)이 가져다 쓸 수 있는
#   컴파일된 코드 묶음이다.
#
# 지금은 src/ 폴더가 비어있다 — 그래서 file(GLOB ...)이 빈 목록을
# 찾게 되고, add_library는 "소스가 0개인 라이브러리"가 된다. 이 단계의
# 목표는 "에러 없이 빌드가 끝나는 것"이지, 실제 코드가 있는 것이 아니다.
# (다음 단계부터 map.cpp 등을 추가하면 이 GLOB이 자동으로 찾아낸다.)
# ─────────────────────────────────────────────────────────────────

# GLOB: core/src 폴더 안의 모든 .cpp 파일 경로를 자동으로 모아서 변수에 담는다.
# 주의: 파일을 추가할 때마다 CMake를 다시 실행해야 GLOB이 새로 인식한다
#   (이건 GLOB 방식의 알려진 단점이다 — 파일 목록을 일일이 적는 방식보다는
#   편하지만, "새 파일을 추가했는데 안 잡힌다"는 혼란이 생길 수 있다.
#   이 프로젝트 규모에서는 편의성이 더 크다고 판단해서 GLOB을 쓴다).
file(GLOB MAPF_CORE_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")

add_library(mapf_core ${MAPF_CORE_SOURCES})

# target_include_directories: "mapf_core를 쓰는 쪽(또는 mapf_core 자신)이
#   #include 할 때, core/include 폴더를 기준으로 찾게 해라."
#   PUBLIC: 이 라이브러리를 가져다 쓰는 다른 타겟(tests 등)도 이 경로를
#   그대로 물려받는다 — 그래서 tests에서 #include "mapf/map.hpp"가 동작한다.
target_include_directories(mapf_core PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

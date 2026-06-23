# ─────────────────────────────────────────────────────────────────
# tests/CMakeLists.txt
#
# 이 파일이 하는 일:
#   tests/ 폴더의 test_*.cpp 파일들을 모아서, mapf_core 라이브러리와
#   GoogleTest를 같이 링크한 "실행 파일"(run_tests)을 만든다.
#   이 실행 파일을 실행하면 모든 TEST(...)가 자동으로 돌아간다.
#
# 지금은 test_*.cpp가 하나도 없다 — 그래서 아주 작은 "더미 테스트
# 파일"을 하나 만들어서, 빌드 시스템 자체가 끝까지 동작하는지부터
# 확인한다(다음 단계에서 진짜 테스트로 교체/추가한다).
# ─────────────────────────────────────────────────────────────────

file(GLOB MAPF_TEST_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/test_*.cpp")

add_executable(run_tests ${MAPF_TEST_SOURCES})

# run_tests가 두 가지를 링크해야 동작한다:
#   mapf_core    — 우리가 만들 알고리즘 코드
#   gtest_main   — GoogleTest 본체 + "main 함수를 대신 만들어주는" 보조 라이브러리
#                  (gtest_main이 있으면 우리가 직접 main()을 안 적어도 된다 —
#                   GoogleTest가 알아서 모든 TEST를 찾아 실행하는 main을 제공한다)
target_link_libraries(run_tests
    PRIVATE
        mapf_core
        gtest_main
)

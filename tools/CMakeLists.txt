# ─────────────────────────────────────────────────────────────────
# tools/CMakeLists.txt
#
# 논문용 데이터를 뽑는 벤치마크 실행 파일(run_benchmark)을 만든다.
# mapf_core 라이브러리만 링크한다 — GoogleTest는 필요 없다(테스트가
# 아니라 데이터 수집용 실행 파일이기 때문).
# ─────────────────────────────────────────────────────────────────

add_executable(run_benchmark benchmark.cpp)

target_link_libraries(run_benchmark
    PRIVATE
        mapf_core
)

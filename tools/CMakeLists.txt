# ─────────────────────────────────────────────────────────────────
# tools/CMakeLists.txt
#
# 논문용 데이터를 뽑는 벤치마크 실행 파일(run_benchmark)을 만든다.
# mapf_core 라이브러리만 링크한다 — GoogleTest는 필요 없다(테스트가
# 아니라 데이터 수집용 실행 파일이기 때문).
#
# Threads::Threads: benchmark.cpp가 std::thread로 trial들을 병렬 실행한다.
# MSVC(Windows)는 보통 추가 링크 없이도 동작하지만, Ubuntu/g++ 등으로
# 옮겼을 때는 pthread를 명시적으로 링크해야 std::thread가 동작한다 —
# find_package(Threads)가 플랫폼에 맞는 스레드 라이브러리를 자동으로
# 찾아준다(00장에서 정한 "OS 무관하게 같은 CMakeLists.txt" 원칙을 따른다).
# ─────────────────────────────────────────────────────────────────

find_package(Threads REQUIRED)

add_executable(run_benchmark benchmark.cpp)

target_link_libraries(run_benchmark
    PRIVATE
        mapf_core
        Threads::Threads
)

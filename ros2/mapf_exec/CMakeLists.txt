# mapf_exec — ROS를 모르는 실행 계층 라이브러리 (docs/31 31.2). ROS 패키지 형식(ament_cmake)이지만 코드는 표준 C++17만 쓴다.
cmake_minimum_required(VERSION 3.16)
project(mapf_exec CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra)

find_package(ament_cmake REQUIRED)

add_library(mapf_exec src/geometry.cpp src/plan_check.cpp src/follower_control.cpp)
target_include_directories(mapf_exec PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
set_target_properties(mapf_exec PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(analyze_run tools/analyze_run.cpp)
target_link_libraries(analyze_run mapf_exec)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS mapf_exec EXPORT export_mapf_exec ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
install(TARGETS analyze_run DESTINATION lib/${PROJECT_NAME})
ament_export_targets(export_mapf_exec HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_mapf_exec test/test_geometry.cpp test/test_plan_check.cpp test/test_follower_control.cpp)
  target_link_libraries(test_mapf_exec mapf_exec)
endif()

ament_package()

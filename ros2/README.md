# ros2/ — 3D(Gazebo)·실제 로봇 실험 (일정 7·8, 설계 docs/31)

Ubuntu 22.04 + ROS2 Humble + Gazebo Classic 11 전용. 최상위 CMakeLists.txt는 이 폴더를 모르므로 Windows 빌드에 영향 없음.

```
cd ros2
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

| 패키지 | 내용 | 상태 |
|---|---|---|
| `mapf_gazebo` | TurtleBot3 N대 launch, 시나리오 JSON → 월드, 점검·시나리오 스크립트 | 7-0, 7-1 |
| `mapf_exec` | ROS 모르는 C++: 칸↔미터, 계획 검사, 몸 검사, 돌고-가기 계산, 기록 분석기 `analyze_run` (이후 ADG·번호 다시 붙이기) | 7-2 |
| `mapf_ros_msgs` | `ActionCommand`, `ActionStatus` | 7-2 |
| `mapf_ros` | `cell_follower`(돌고-가기 노드), `mapf_executive`(저장된 계획 → 계획 검사 → ADG 실행·기록) | 7-2, 7-3 |

## 7-0 환경 점검

```
# 터미널 1: 화면 없이 6대 (gui:=true면 화면도)
ros2 launch mapf_gazebo multi_tb3.launch.py num_robots:=6
# 터미널 2: 자동 점검 (토픽·TF 분리, 따로 움직임, odom 드리프트, RTF)
ros2 run mapf_gazebo check_7_0 --num-robots 6 --out result.json
# (선택) 손으로 한 대씩 조종
ros2 run turtlebot3_teleop teleop_keyboard --ros-args -r __ns:=/robot_1
```
결과: `results/7_0/`, 해석: docs/31 31.18.

## 7-1 시나리오대로 띄우기

```
# 터미널 1: 시나리오 JSON(패키지 scenarios/ 안 이름 또는 파일 경로)대로 월드를 만들고 출발 칸에 6대
ros2 launch mapf_gazebo scenario.launch.py scenario:=corridor10_r6_s1 gui:=true
# 터미널 2: 자동 점검 (출발 위치, LiDAR 실제 거리 ↔ 격자 계산 거리, RTF)
ros2 run mapf_gazebo check_7_1 --scenario corridor10_r6_s1
# 2D 그림(0행   이 위) — gui:=true 화면(처음부터 위에서 내려다봄, 위 = 북 = +y)과 같은 모양이어야 함
ros2 run mapf_gazebo draw_scenario corridor10_r6_s1 --out corridor.png
# 새 시나리오 만들기 (먼저 저장소 최상위에서 코어 빌드 필요: mapf_py)
ros2 run mapf_gazebo make_scenario --map corridor --size 10 --robots 6 --seed 2 --out my.json
# 단위 테스트
python3 -m pytest mapf_gazebo/test -q
```
결과: `results/7_1/`, 해석: docs/31 31.20. 시나리오 형식: 31.19.
월드(.world)는 띄울 때마다 시나리오 JSON에서 새로 만든다(JSON 하나만 원본). 파일로 필요하면 `ros2 run mapf_gazebo grid_to_world 시나리오.json --out x.world`.

## 7-2 한 대 주행 시험

```
# 터미널 1: 한 대짜리 시험 시나리오 (slow = 0.12 m/s, fast = 0.22 m/s). cell_follower가 같이 뜬다
ros2 launch mapf_gazebo scenario.launch.py scenario:=follow_corridor10_slow gui:=true
# 터미널 2: 손 경로 30동작을 보내고 기록·분석 (약 3분)
ros2 run mapf_gazebo run_7_2 --scenario follow_corridor10_slow --out-dir my_run
ros2 run mapf_gazebo plot_run --scenario follow_corridor10_slow --log my_run/run.csv --out my_run/track.png
# 단위 테스트 (mapf_exec)
./build/mapf_exec/test_mapf_exec
```
결과: `results/7_2/`, 해석: docs/31 31.21.

## 7-3 여러 대 ADG 실행

```
# 터미널 1: 6대 시나리오 (cell_follower 같이 뜸, Gazebo 시계 100 Hz)
ros2 launch mapf_gazebo scenario.launch.py scenario:=corridor10_r6_s1 gui:=true
# 터미널 2: 저장된 처음 계획을 ADG로 실행 → 기록 분석 (약 1분)
ros2 run mapf_gazebo run_7_3 --scenario corridor10_r6_s1 --out-dir my73
ros2 run mapf_gazebo plot_run --scenario corridor10_r6_s1 --log my73/run.csv --out my73/track.png
# 변이 검사(부딪혀야 정상): 터미널 1을 estop_distance:=0.0 으로 다시 띄우고
ros2 run mapf_gazebo run_7_3 --scenario corridor10_r6_s1 --out-dir my73_mut --drop-cross-edges
```
한 번 실행하면 로봇이 목적지에 가 있으므로, 다시 돌리려면 터미널 1을 껐다 켠다. 결과: `results/7_3/`, 해석: docs/31 31.22.

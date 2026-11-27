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
| `mapf_ros` | `cell_follower`(돌고-가기 노드), `mapf_executive`(저장된 계획 → 계획 검사 → ADG 실행·기록, 장애물 발견·재계획) | 7-2~7-5 |

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

## RViz2 한 창 + 7-4 장애물 재계획

```
# 실행 + RViz2 화면 + 녹화
ros2 launch mapf_gazebo scenario.launch.py scenario:=corridor10_r6_s1_ob1 rviz:=true record:=my_bag
ros2 run mapf_gazebo run_7_4 --scenario corridor10_r6_s1_ob1 --method ours --out-dir my74   # full = 전체 재계획
# 다시 보기(Gazebo 없이): RViz + 재생(재생 터미널에서 스페이스 = 멈춤/계속)
ros2 launch mapf_gazebo replay.launch.py scenario:=corridor10_r6_s1_ob1
ros2 bag play my_bag   # --clock 붙이지 않음
# 여러 실행을 차례로(Gazebo를 실행마다 새로 띄우고 끔)
ros2 run mapf_gazebo run_batch --out-root my_batch --record corridor10_r6_s1_ob1:ours corridor10_r6_s1_ob1:full
# 시나리오에 장애물 넣기
ros2 run mapf_gazebo add_obstacle corridor10_r6_s1 --spawn 2.0 --out my_ob.json
```
상태 색: MOVE 파랑, WORK 검정(남은 초), YIELD 주황(양보 — 화살표가 기다리게 하는 로봇), HOLD 빨강(장애물 앞), STOP 보라(재계획 대기), DONE 초록, ESTOP 자홍.
결과: `results/7_4/`(녹화 `bag/`은 git에 없음), 해석: docs/31 31.23.

## 곧게 지나가기 · 비상 정지 v2 · LiDAR 발견 (31.24, 31.25) — 모두 스위치, 기본 꺼짐

```
# 한 번 실행: launch에 robust_estop:=true, run_7_4에 --pass-through --detection lidar
ros2 launch mapf_gazebo scenario.launch.py scenario:=corridor10_r6_s1_hold robust_estop:=true rviz:=true record:=my_bag
ros2 run mapf_gazebo run_7_4 --scenario corridor10_r6_s1_hold --method ours --pass-through --detection lidar \
    --d-detect 1.0 --t-wait 10 --out-dir my75
# 여러 번(run_batch가 launch·run_7_4 둘 다에 넘김)
ros2 run mapf_gazebo run_batch --out-root my_batch --record --pass-through --robust-estop --detection lidar \
    --extra="--t-wait 10" corridor10_r6_s1_hold:ours corridor10_r6_s1_hold:full
# 결과 표, 재계획 입력을 2D 파이썬으로 다시 돌려 같은지
ros2 run mapf_gazebo summarize_runs my_batch --csv my_batch.csv
ros2 run mapf_gazebo check_replan_input my_batch/*/replan_*.json --both
# 장애물 칸을 직접 골라 시나리오 만들기(열 행 순서!)
ros2 run mapf_gazebo add_obstacle corridor10_r6_s1 --cell 5 1 --spawn 0.0 --suffix _hold --out my_hold.json
```
- `--pass-through`: 같은 방향으로 이어지는 칸은 칸 중심에서 서지 않음. `--robust-estop`(launch `robust_estop:=true`): 앞으로 갈 거리 안의 물체만, 광선 2개 이상.
- `--detection lidar`: 로봇 LiDAR 점으로 장애물 칸을 찾음(다른 로봇 ± 0.2 m 점, 선반·벽 면 4 cm 안 점은 버림), 최근 `t_wait` 동안 90% 이상 보이면 확정.
- 실행 폴더마다 `replan_<n>.json`(재계획 입력·출력)이 생긴다.
- RViz: 노란 반투명 상자 = LiDAR가 찾았지만 시나리오에 없는 칸(오탐). 결과: `results/7_3pt`, `7_4pt`, `7_5`, 해석: docs/31 31.24·31.25.

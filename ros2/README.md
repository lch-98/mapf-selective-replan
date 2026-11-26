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
| `mapf_gazebo` | TurtleBot3 N대 launch(`multi_tb3.launch.py`), 월드, 점검·시나리오 스크립트 | 7-0 |
| `mapf_exec`, `mapf_ros_msgs`, `mapf_ros` | ADG·번호 다시 붙이기·검사(ROS 모름) / 메시지 / 노드 | 예정(7-2~) |

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

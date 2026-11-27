#!/bin/bash
# 7-6 본 실험: LiDAR 발견 + 곧게 지나가기 + 비상 정지 v2. 우리 방법 vs 전체 재계획.
cd /home/lee/mapf-selective-replan/ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
R=results/7_6   # 장애물 없는 기준(noobs)은 이 스크립트 뒤에 따로 돌림: run_batch --out-root $R/noobs (같은 옵션) corridor10_r6_s2:ours ...
mkdir -p $R
F="--pass-through --robust-estop --detection lidar --record"
C="corridor10_r6_s2_ob1 corridor10_r6_s3_ob1 corridor10_r6_s4_ob1 corridor10_r6_s13_ob1 corridor10_r6_s17_ob1"
O="open10_r6_s9_ob1 open10_r6_s19_ob1 open10_r6_s28_ob1 open10_r6_s32_ob1 open10_r6_s20_ob1"
jobs() { for s in "$@"; do echo "$s:ours $s:full"; done; }
ros2 run mapf_gazebo run_batch --out-root $R/base $F $(jobs $C $O) > $R/base.log 2>&1
ros2 run mapf_gazebo run_batch --out-root $R/twait0 $F --extra="--t-wait 0" $(jobs $C) > $R/twait0.log 2>&1
ros2 run mapf_gazebo run_batch --out-root $R/twait10 $F --extra="--t-wait 10" $(jobs $C) > $R/twait10.log 2>&1
ros2 run mapf_gazebo run_batch --out-root $R/d1 $F --extra="--d-detect 1.0" $(jobs $C) > $R/d1.log 2>&1
ros2 run mapf_gazebo run_batch --out-root $R/d3 $F --extra="--d-detect 3.0" $(jobs $C) > $R/d3.log 2>&1
echo "7-6 끝"

#!/bin/bash
cd /home/lee/mapf-selective-replan/ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
R=results/7_5
F="--pass-through --robust-estop --detection lidar"
ros2 run mapf_gazebo run_batch --out-root $R/noobs $F corridor10_r6_s1:ours open10_r6_s1:ours corridor10_r6_s1_sv1-3:ours > $R/noobs.log 2>&1
ros2 run mapf_gazebo run_batch --out-root $R/ob1 --record $F corridor10_r6_s1_ob1:ours corridor10_r6_s1_ob1:full open10_r6_s1_ob1:ours open10_r6_s1_ob1:full corridor10_r6_s1_sv1-3_ob1:ours corridor10_r6_s1_sv1-3_ob1:full > $R/ob1.log 2>&1
ros2 run mapf_gazebo run_batch --out-root $R/hold --record $F --extra="--d-detect 1.0 --t-wait 10" corridor10_r6_s1_hold:ours corridor10_r6_s1_hold:full open10_r6_s1_hold:ours open10_r6_s1_hold:full > $R/hold.log 2>&1
ros2 run mapf_gazebo run_batch --out-root $R/mut $F --extra="--drop-cross-edges" corridor10_r6_s1:ours > $R/mut.log 2>&1
echo "7-5 끝"

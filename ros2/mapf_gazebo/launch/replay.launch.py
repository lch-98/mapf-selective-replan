"""녹화한 3D 실험을 RViz2 한 창으로 다시 본다 (docs/31 31.23). Gazebo는 필요 없다.

    # 터미널 1: RViz2 + 로봇 모델(시나리오에서 로봇 이름을 읽음)
    ros2 launch mapf_gazebo replay.launch.py scenario:=corridor10_r6_s1_ob1
    # 터미널 2: 녹화 재생 — 이 터미널에서 스페이스 = 멈춤/계속, → = 한 메시지씩 (멈춘 상태에서)
    ros2 bag play <녹화 폴더> --rate 1.0

--clock은 붙이지 않는다: 녹화 안에 Gazebo 시뮬레이터 시계(/clock)가 이미 있다. --clock을 붙이면 재생기가 녹화 때의 컴퓨터 시각으로
두 번째 /clock을 내보내 시계가 0초대와 17억 초대를 오가고(8초에 284번 거꾸로 감, 2026-11-27 측정), RViz가 TF를 계속 지워 로봇이 번쩍인다.
Gazebo 화면은 다시 볼 수 없다(Gazebo는 녹화를 재생하는 도구가 아니라 물리 시뮬레이터) — 실행할 때 scenario.launch.py gui:=true로 본다.

play:=<녹화 폴더>를 주면 터미널 1에서 재생까지 한다(이때는 키보드로 멈출 수 없음).
"""
import os
import tempfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from mapf_gazebo import scenario_io
from mapf_gazebo.rviz_config import make_rviz
from mapf_gazebo.spawn import robot_description, world_odom_tf


def _setup(context):
    sc = scenario_io.load(LaunchConfiguration('scenario').perform(context))
    names = [r['name'] for r in sc['robots']]
    cell, h = sc['cell_size'], len(sc['map'])
    urdf = robot_description()
    actions = [Node(package='robot_state_publisher', executable='robot_state_publisher', namespace=n, output='log',
                    parameters=[{'use_sim_time': True, 'robot_description': urdf, 'frame_prefix': f'{n}/'}]) for n in names]
    actions += world_odom_tf(names)
    rviz_path = os.path.join(tempfile.mkdtemp(prefix='mapf_replay_'), 'mapf.rviz')
    with open(rviz_path, 'w') as f:
        f.write(make_rviz(names, len(sc['map'][0]) * cell, h * cell))
    if LaunchConfiguration('rviz').perform(context).lower() == 'true':
        actions.append(Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_path], output='log',
                            parameters=[{'use_sim_time': True}]))
    bag = LaunchConfiguration('play').perform(context)
    if bag:
        actions.append(ExecuteProcess(cmd=['ros2', 'bag', 'play', bag, '--rate',
                                           LaunchConfiguration('rate').perform(context)], output='screen'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='corridor10_r6_s1'),
        DeclareLaunchArgument('play', default_value='', description='녹화 폴더(주면 바로 재생)'),
        DeclareLaunchArgument('rate', default_value='1.0'),
        DeclareLaunchArgument('rviz', default_value='true', description='false면 RViz 없이(시험용)'),
        OpaqueFunction(function=_setup),
    ])

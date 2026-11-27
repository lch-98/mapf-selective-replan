"""시나리오 JSON대로 월드를 만들고, 로봇을 출발 칸에 띄운다 — 7-1 (docs/31 31.12, 31.19).

    ros2 launch mapf_gazebo scenario.launch.py scenario:=corridor10_r6_s1 gui:=true

- scenario: 파일 경로, 또는 패키지 scenarios/ 안의 이름(.json 생략 가능)
- 월드: grid_to_world.world_xml로 임시 폴더에 만들어 띄운다(손으로 고친 .world 없음 → 2D와 3D 지도가 항상 같음)
- 로봇: robot_{id+1}을 출발 칸 중심(grid.cell_center)에, 처음 이동 방향(start_yaw_deg)을 보게
- followers:=true(기본): 로봇마다 cell_follower(돌고-가기 노드, 7-2)를 같이 띄운다
- debug_no_flip:=true 는 변이 검사용: 세로를 안 뒤집은 "틀린" 월드를 띄워 check_7_1이 잡는지 본다(기본 false)
"""
import math
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from mapf_gazebo import scenario_io
from mapf_gazebo.grid import cell_center
from mapf_gazebo.grid_to_world import world_xml
from mapf_gazebo.spawn import robot_actions


def _setup(context):
    sc = scenario_io.load(LaunchConfiguration('scenario').perform(context))
    flip = LaunchConfiguration('debug_no_flip').perform(context).lower() != 'true'
    world_path = os.path.join(tempfile.mkdtemp(prefix='mapf_world_'), sc['name'] + '.world')
    with open(world_path, 'w') as f:
        f.write(world_xml(sc, flip_y=flip))

    # Gazebo가 /clock을 내보내는 빈도(기본 10 Hz). 낮으면 20 Hz 제어·기록 타이머가 같은 시뮬레이터 시각에 두 번씩 돈다(31.22).
    params_path = os.path.join(os.path.dirname(world_path), 'gazebo_params.yaml')
    with open(params_path, 'w') as f:
        f.write(f"gazebo:\n  ros__parameters:\n    publish_rate: {float(LaunchConfiguration('clock_hz').perform(context))}\n")

    gazebo_ros = get_package_share_directory('gazebo_ros')
    actions = [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world_path, 'params_file': params_path}.items())]
    if LaunchConfiguration('gui').perform(context).lower() == 'true':
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_ros, 'launch', 'gzclient.launch.py'))))

    h, cell = len(sc['map']), sc['cell_size']
    poses = []
    for r in sc['robots']:
        x, y = cell_center(r['start'][0], r['start'][1], h, cell)
        poses.append((r['name'], x, y, math.radians(r['start_yaw_deg'])))
    delay = float(LaunchConfiguration('spawn_delay').perform(context))
    actions += robot_actions(poses, delay)
    # 로봇마다 cell_follower(돌고-가기, 31.7) — 로봇별 최고 속도는 시나리오 JSON 값(31.13 D11)
    if LaunchConfiguration('followers').perform(context).lower() == 'true':
        for r in sc['robots']:
            actions.append(Node(package='mapf_ros', executable='cell_follower', namespace=r['name'], output='log',
                                parameters=[{'use_sim_time': True, 'max_linear': float(r['max_linear']),
                                             'max_angular': float(r['max_angular']),
                                             'estop_distance': float(LaunchConfiguration('estop_distance').perform(context))}]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='corridor10_r6_s1'),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('spawn_delay', default_value='0.5'),
        DeclareLaunchArgument('followers', default_value='true', description='로봇마다 cell_follower 노드(7-2)'),
        DeclareLaunchArgument('clock_hz', default_value='100.0', description='Gazebo /clock 빈도(Hz)'),
        DeclareLaunchArgument('estop_distance', default_value='0.25', description='비상 정지 거리(m). 0이면 끔 — 변이 검사용, 실험에 쓰지 말 것'),
        DeclareLaunchArgument('debug_no_flip', default_value='false', description='변이 검사용(틀린 월드). 실험에 쓰지 말 것'),
        OpaqueFunction(function=_setup),
    ])

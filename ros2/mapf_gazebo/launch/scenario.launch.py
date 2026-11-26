"""시나리오 JSON대로 월드를 만들고, 로봇을 출발 칸에 띄운다 — 7-1 (docs/31 31.12, 31.19).

    ros2 launch mapf_gazebo scenario.launch.py scenario:=corridor10_r6_s1 gui:=true

- scenario: 파일 경로, 또는 패키지 scenarios/ 안의 이름(.json 생략 가능)
- 월드: grid_to_world.world_xml로 임시 폴더에 만들어 띄운다(손으로 고친 .world 없음 → 2D와 3D 지도가 항상 같음)
- 로봇: robot_{id+1}을 출발 칸 중심(grid.cell_center)에, 처음 이동 방향(start_yaw_deg)을 보게
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

    gazebo_ros = get_package_share_directory('gazebo_ros')
    actions = [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world_path}.items())]
    if LaunchConfiguration('gui').perform(context).lower() == 'true':
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_ros, 'launch', 'gzclient.launch.py'))))

    h, cell = len(sc['map']), sc['cell_size']
    poses = []
    for r in sc['robots']:
        x, y = cell_center(r['start'][0], r['start'][1], h, cell)
        poses.append((r['name'], x, y, math.radians(r['start_yaw_deg'])))
    delay = float(LaunchConfiguration('spawn_delay').perform(context))
    return actions + robot_actions(poses, delay)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='corridor10_r6_s1'),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('spawn_delay', default_value='0.5'),
        DeclareLaunchArgument('debug_no_flip', default_value='false', description='변이 검사용(틀린 월드). 실험에 쓰지 말 것'),
        OpaqueFunction(function=_setup),
    ])

"""TurtleBot3 Burger N대를 robot_1 … robot_N 네임스페이스로 띄운다 (docs/31 31.10·31.12, 7-0).

    ros2 launch mapf_gazebo multi_tb3.launch.py num_robots:=2 gui:=false

- 토픽: /robot_i/cmd_vel, /robot_i/odom, /robot_i/scan …  (spawn_entity의 -robot_namespace)
- TF 프레임: robot_i/odom, robot_i/base_footprint, robot_i/base_scan …  (모델 SDF의 프레임 이름을 바꿔서)
- 로봇 위치: x = (i-1) × spacing, y = 0 (7-0 점검용 한 줄 배치. 7-1부터는 시나리오 JSON의 출발 칸)

turtlebot3_gazebo의 multi_robot.launch.py를 참고했지만, 그 파일은 바꾼 SDF를 /opt/ros 안에 써서
일반 사용자 권한으로는 실패한다. 여기서는 임시 폴더에 쓴다.
"""
import os
import tempfile
import xml.etree.ElementTree as ET

import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def _robots(context):
    n = int(LaunchConfiguration('num_robots').perform(context))
    spacing = float(LaunchConfiguration('spacing').perform(context))
    spawn_delay = float(LaunchConfiguration('spawn_delay').perform(context))
    model = os.environ.get('TURTLEBOT3_MODEL', 'burger')

    tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    tb3_desc = get_package_share_directory('turtlebot3_description')
    sdf_src = os.path.join(tb3_gazebo, 'models', f'turtlebot3_{model}', 'model.sdf')
    # TurtleBot3의 .urdf는 사실 xacro 파일이다(${namespace} 자리표시자). 그냥 읽으면 프레임·관절 이름에
    # "${namespace}"가 글자 그대로 남아 바퀴 TF가 안 나온다(Gazebo가 보내는 관절 이름과 안 맞음).
    # namespace는 비우고, 접두어는 robot_state_publisher의 frame_prefix로 붙인다.
    urdf = xacro.process_file(os.path.join(tb3_desc, 'urdf', f'turtlebot3_{model}.urdf'),
                              mappings={'namespace': ''}).toxml()

    out_dir = tempfile.mkdtemp(prefix='mapf_tb3_')
    actions = []
    for i in range(1, n + 1):
        ns = f'robot_{i}'
        tree = ET.parse(sdf_src)
        for tag in tree.getroot().iter('odometry_frame'):
            tag.text = f'{ns}/odom'
        for tag in tree.getroot().iter('robot_base_frame'):
            tag.text = f'{ns}/base_footprint'
        for tag in tree.getroot().iter('frame_name'):
            tag.text = f'{ns}/base_scan'
        sdf_path = os.path.join(out_dir, f'{ns}.sdf')
        # encoding 선언이 있으면 spawn_entity.py(lxml)가 "Unicode strings with encoding declaration" 오류로 못 읽는다.
        with open(sdf_path, 'w') as f:
            f.write('<?xml version="1.0" ?>\n' + ET.tostring(tree.getroot(), encoding='unicode'))

        # 한꺼번에 생성하면 가끔 한 대의 spawn 요청이 응답 없이 멈춘다(12대에서 관찰) → 로봇마다 spawn_delay초씩 늦춤
        actions.append(TimerAction(period=(i - 1) * spawn_delay, actions=[GroupAction([
            PushRosNamespace(ns),
            Node(package='robot_state_publisher', executable='robot_state_publisher', output='log',
                 parameters=[{'use_sim_time': True, 'robot_description': urdf, 'frame_prefix': f'{ns}/'}]),
            Node(package='gazebo_ros', executable='spawn_entity.py', output='log',
                 arguments=['-entity', ns, '-file', sdf_path,
                            '-x', str((i - 1) * spacing), '-y', '0.0', '-z', '0.01',
                            '-robot_namespace', ns]),
        ])]))
    return actions


def generate_launch_description():
    pkg = get_package_share_directory('mapf_gazebo')
    gazebo_ros = get_package_share_directory('gazebo_ros')
    world = LaunchConfiguration('world')

    return LaunchDescription([
        DeclareLaunchArgument('num_robots', default_value='2'),
        DeclareLaunchArgument('spacing', default_value='1.0', description='로봇 사이 간격(m)'),
        DeclareLaunchArgument('spawn_delay', default_value='0.5', description='로봇 사이 생성 간격(초)'),
        DeclareLaunchArgument('gui', default_value='false', description='true면 gzclient(화면)도 띄움'),
        DeclareLaunchArgument('world', default_value=os.path.join(pkg, 'worlds', 'empty_check.world')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py')),
            launch_arguments={'world': world}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_ros, 'launch', 'gzclient.launch.py')),
            condition=IfCondition(LaunchConfiguration('gui'))),
        OpaqueFunction(function=_robots),
    ])

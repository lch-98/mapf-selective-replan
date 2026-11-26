"""TurtleBot3를 robot_i 네임스페이스로 생성하는 launch 동작 — multi_tb3.launch.py(7-0)와 scenario.launch.py(7-1)가 같이 쓴다.

- 토픽: /robot_i/cmd_vel·odom·scan …  (spawn_entity의 -robot_namespace)
- TF 프레임: robot_i/odom, robot_i/base_footprint …  (모델 SDF의 프레임 이름 + robot_state_publisher의 frame_prefix)
- 7-0에서 고친 것: TurtleBot3 .urdf는 xacro라 xacro로 처리, SDF에 encoding 선언 없이 씀, 로봇마다 spawn_delay초 간격(docs/31 31.18).
"""
import math
import os
import tempfile
import xml.etree.ElementTree as ET

import xacro
from ament_index_python.packages import get_package_share_directory
from launch.actions import GroupAction, TimerAction
from launch_ros.actions import Node, PushRosNamespace


def robot_actions(robots, spawn_delay=0.5):
    """robots: [(name, x, y, yaw_rad), ...] → launch 동작 목록."""
    model = os.environ.get('TURTLEBOT3_MODEL', 'burger')
    tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    tb3_desc = get_package_share_directory('turtlebot3_description')
    sdf_src = os.path.join(tb3_gazebo, 'models', f'turtlebot3_{model}', 'model.sdf')
    # TurtleBot3의 .urdf는 사실 xacro 파일이다(${namespace} 자리표시자). 그냥 읽으면 프레임·관절 이름에
    # "${namespace}"가 글자 그대로 남아 바퀴 TF가 안 나온다. namespace는 비우고 접두어는 frame_prefix로.
    urdf = xacro.process_file(os.path.join(tb3_desc, 'urdf', f'turtlebot3_{model}.urdf'),
                              mappings={'namespace': ''}).toxml()

    out_dir = tempfile.mkdtemp(prefix='mapf_tb3_')
    actions = []
    for k, (ns, x, y, yaw) in enumerate(robots):
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
        actions.append(TimerAction(period=k * spawn_delay, actions=[GroupAction([
            PushRosNamespace(ns),
            Node(package='robot_state_publisher', executable='robot_state_publisher', output='log',
                 parameters=[{'use_sim_time': True, 'robot_description': urdf, 'frame_prefix': f'{ns}/'}]),
            Node(package='gazebo_ros', executable='spawn_entity.py', output='log',
                 arguments=['-entity', ns, '-file', sdf_path,
                            '-x', f'{x:.4f}', '-y', f'{y:.4f}', '-z', '0.01', '-Y', f'{yaw:.5f}',
                            '-robot_namespace', ns]),
        ])]))
    return actions


def line_poses(n, spacing):
    """7-0용 한 줄 배치: robot_i를 x = (i-1)·spacing, y = 0, 방향 0."""
    return [(f'robot_{i}', (i - 1) * spacing, 0.0, 0.0) for i in range(1, n + 1)]


def deg(d):
    return math.radians(d)

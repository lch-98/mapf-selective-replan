"""TurtleBot3 Burger N대를 robot_1 … robot_N 네임스페이스로 한 줄로 띄운다 (docs/31 31.12, 7-0 점검용).

    ros2 launch mapf_gazebo multi_tb3.launch.py num_robots:=2 gui:=false

로봇 위치: x = (i-1) × spacing, y = 0. 시나리오의 출발 칸에 띄우려면 scenario.launch.py(7-1).
생성 방법(네임스페이스·TF 접두어·xacro 처리·생성 간격)은 mapf_gazebo/spawn.py.
turtlebot3_gazebo의 multi_robot.launch.py를 참고했지만, 그 파일은 바꾼 SDF를 /opt/ros 안에 써서
일반 사용자 권한으로는 실패한다. 여기서는 임시 폴더에 쓴다.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from mapf_gazebo.spawn import line_poses, robot_actions


def _robots(context):
    n = int(LaunchConfiguration('num_robots').perform(context))
    spacing = float(LaunchConfiguration('spacing').perform(context))
    delay = float(LaunchConfiguration('spawn_delay').perform(context))
    return robot_actions(line_poses(n, spacing), delay)


def generate_launch_description():
    pkg = get_package_share_directory('mapf_gazebo')
    gazebo_ros = get_package_share_directory('gazebo_ros')
    return LaunchDescription([
        DeclareLaunchArgument('num_robots', default_value='2'),
        DeclareLaunchArgument('spacing', default_value='1.0', description='로봇 사이 간격(m)'),
        DeclareLaunchArgument('spawn_delay', default_value='0.5', description='로봇 사이 생성 간격(초)'),
        DeclareLaunchArgument('gui', default_value='false', description='true면 gzclient(화면)도 띄움'),
        DeclareLaunchArgument('world', default_value=os.path.join(pkg, 'worlds', 'empty_check.world')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py')),
            launch_arguments={'world': LaunchConfiguration('world')}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_ros, 'launch', 'gzclient.launch.py')),
            condition=IfCondition(LaunchConfiguration('gui'))),
        OpaqueFunction(function=_robots),
    ])

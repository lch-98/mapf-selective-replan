from glob import glob

from setuptools import setup

package_name = 'mapf_gazebo'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/worlds', glob('worlds/*.world')),
        ('share/' + package_name + '/scenarios', glob('scenarios/*.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'check_7_0 = mapf_gazebo.check_7_0:main',
            'check_7_1 = mapf_gazebo.check_7_1:main',
            'run_7_2 = mapf_gazebo.run_7_2:main',
            'plot_run = mapf_gazebo.plot_run:main',
            'run_7_3 = mapf_gazebo.run_7_3:main',
            'make_scenario = mapf_gazebo.make_scenario:main',
            'grid_to_world = mapf_gazebo.grid_to_world:main',
            'draw_scenario = mapf_gazebo.draw_scenario:main',
        ],
    },
)

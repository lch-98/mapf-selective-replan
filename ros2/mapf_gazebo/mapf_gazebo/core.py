"""저장소의 C++ 코어 파이썬 모듈(mapf_py)을 불러온다. 먼저 저장소 최상위에서 코어를 빌드해야 한다(CLAUDE.md 빌드·테스트)."""
import importlib
import os
import sys
from pathlib import Path


def repo_root():
    # colcon build --symlink-install이면 이 파일은 저장소 안의 원본이다: <repo>/ros2/mapf_gazebo/mapf_gazebo/core.py
    env = os.environ.get('MAPF_REPO')
    return Path(env) if env else Path(__file__).resolve().parents[3]


def load_mapf_py():
    sys.path.insert(0, str(repo_root() / 'python'))
    from _pathsetup import add_mapf_py_to_path
    add_mapf_py_to_path()
    return importlib.import_module('mapf_py')

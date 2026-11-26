"""시나리오 JSON 찾기·읽기. 인자가 파일 경로면 그대로, 아니면 패키지 scenarios/ 안의 이름(.json 생략 가능)."""
import json
import os


def resolve(arg):
    if os.path.isfile(arg):
        return arg
    from ament_index_python.packages import get_package_share_directory
    name = arg if arg.endswith('.json') else arg + '.json'
    return os.path.join(get_package_share_directory('mapf_gazebo'), 'scenarios', name)


def load(arg):
    with open(resolve(arg)) as f:
        sc = json.load(f)
    if sc.get('format') != 'mapf_scenario_v1':
        raise ValueError(f"시나리오 형식이 mapf_scenario_v1이 아님: {sc.get('format')}")
    return sc

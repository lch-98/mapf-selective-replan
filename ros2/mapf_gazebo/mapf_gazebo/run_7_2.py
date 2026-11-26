"""7-2 한 대 주행 시험 (docs/31 31.12, 31.21).

scenario.launch.py로 7-2 시나리오(follow_corridor10_slow / _fast)를 띄운 상태에서:

    ros2 run mapf_gazebo run_7_2 --scenario follow_corridor10_slow --out-dir results/7_2/slow

하는 일
  1. 시나리오의 test_path([열, 행] 목록)를 한 동작씩 robot_1의 cell_follower에 보낸다(이웃 칸 = MOVE, 같은 칸 = WAIT Δt_nom초).
     "끝났다"(DONE)를 받으면 다음 동작. ADG처럼 한 번에 한 동작만 내준다.
  2. 그동안 odom(Gazebo에서는 정답 위치, 31.18)을 모두 CSV로 기록: 그 순간의 동작 종류, 가진 칸(MOVE면 출발·목표 2칸), 구간.
  3. 끝나면 mapf_exec의 analyze_run(C++ 몸 검사)으로 칸 이탈·옆 오차·지나침·선반/벽 겹침을 계산.
  4. 기준(31.12 7-2): 칸마다 중심 3 cm 안 도착, 칸 이탈 0, 추종 오차(옆 오차·지나침) 최대 < 0.11 m, 선반/벽 겹침 0, 비상 정지 0.
"""
import argparse
import json
import math
import os
import subprocess
import sys
import time

import rclpy
from ament_index_python.packages import get_package_prefix
from mapf_ros_msgs.msg import ActionCommand, ActionStatus
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy

from mapf_gazebo import scenario_io
from mapf_gazebo.grid import cell_center

ARRIVE_TOL, TRACK_TOL = 0.03, 0.11


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Driver(Node):
    def __init__(self, robot):
        super().__init__('run_7_2', parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        rel = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(ActionCommand, f'/{robot}/mapf/action', rel)
        self.create_subscription(ActionStatus, f'/{robot}/mapf/status', self._on_status, rel)
        self.create_subscription(Odometry, f'/{robot}/odom', self._on_odom, 50)
        self.robot = robot
        self.status = {}      # seq → 마지막 ActionStatus
        self.pose = None
        self.rows = []        # CSV 줄
        self.act_ctx = ('IDLE', '', (0, 0, 0, 0))

    def _on_status(self, m):
        self.status[m.seq] = m

    def _on_odom(self, m):
        p = m.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_of(p.orientation))
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        kind, held, seg = self.act_ctx
        self.rows.append(f'{t:.3f},{self.robot},{self.pose[0]:.5f},{self.pose[1]:.5f},{self.pose[2]:.5f},'
                         f'{kind},{held},{seg[0]:.4f},{seg[1]:.4f},{seg[2]:.4f},{seg[3]:.4f}')

    def subscribers_ready(self):
        return self.pub.get_subscription_count() > 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--scenario', default='follow_corridor10_slow')
    ap.add_argument('--out-dir', required=True)
    ap.add_argument('--move-timeout', type=float, default=60.0, help='동작 하나의 실제 시간 한도(초)')
    args = ap.parse_args()

    sc = scenario_io.load(args.scenario)
    rows, cell, h = sc['map'], sc['cell_size'], len(sc['map'])
    robot = sc['robots'][0]
    path = sc['test_path']
    os.makedirs(args.out_dir, exist_ok=True)

    rclpy.init()
    nd = Driver(robot['name'])
    end = time.monotonic() + 60
    while time.monotonic() < end and (nd.pose is None or not nd.subscribers_ready()):
        rclpy.spin_once(nd, timeout_sec=0.1)
    if nd.pose is None or not nd.subscribers_ready():
        print('[실패] 준비 안 됨: odom 또는 cell_follower가 없음')
        sys.exit(2)

    results, ok, t_start = [], True, None
    for seq, (a, b) in enumerate(zip(path, path[1:])):
        fa, fb = cell_center(a[0], a[1], h, cell), cell_center(b[0], b[1], h, cell)
        cmd = ActionCommand()
        cmd.seq, cmd.cell_x, cmd.cell_y = seq, b[0], b[1]
        cmd.from_x, cmd.from_y, cmd.target_x, cmd.target_y = fa[0], fa[1], fb[0], fb[1]
        if a == b:
            cmd.kind, cmd.duration_s = ActionCommand.WAIT, float(sc['dt_nom_s'])
            nd.act_ctx = ('WAIT', f'{a[0]}:{a[1]}', (fa[0], fa[1], fb[0], fb[1]))
        else:
            cmd.kind = ActionCommand.MOVE
            nd.act_ctx = ('MOVE', f'{a[0]}:{a[1]};{b[0]}:{b[1]}', (fa[0], fa[1], fb[0], fb[1]))
        nd.pub.publish(cmd)
        if t_start is None and nd.rows:
            t_start = float(nd.rows[-1].split(',')[0])
        wall_end = time.monotonic() + args.move_timeout
        st = None
        while time.monotonic() < wall_end:
            rclpy.spin_once(nd, timeout_sec=0.02)
            st = nd.status.get(seq)
            if st is not None and st.state != ActionStatus.ACTIVE:
                break
        state = {None: '응답 없음', ActionStatus.ACTIVE: '시간 초과', ActionStatus.DONE: 'DONE',
                 ActionStatus.HELD: 'HELD', ActionStatus.ESTOP: 'ESTOP'}[None if st is None else st.state]
        err = st.pos_err_m if st is not None else float('nan')
        step_ok = state == 'DONE' and (cmd.kind == ActionCommand.WAIT or err < ARRIVE_TOL)
        ok &= step_ok
        results.append({'seq': seq, 'from': a, 'to': b, 'kind': 'WAIT' if cmd.kind else 'MOVE', 'state': state,
                        'arrive_err_m': err})
        nd.act_ctx = ('IDLE', f'{b[0]}:{b[1]}', (fb[0], fb[1], fb[0], fb[1]))
        print(f"  동작 {seq:2d} {a[1]}행 {a[0]}열 → {b[1]}행 {b[0]}열 {results[-1]['kind']:4s} {state:5s}"
              + (f'  도착 오차 {err * 100:.1f} cm' if cmd.kind == ActionCommand.MOVE else ''))
        if not step_ok:
            print('  → 실패해서 멈춤')
            break
    # 멈춘 뒤 조금 더 기록
    s_end = time.monotonic() + 1.0
    while time.monotonic() < s_end:
        rclpy.spin_once(nd, timeout_sec=0.05)
    t_end = float(nd.rows[-1].split(',')[0])

    log_path = os.path.join(args.out_dir, 'run.csv')
    map_path = os.path.join(args.out_dir, 'map.txt')
    with open(log_path, 'w') as f:
        f.write('t,robot,x,y,yaw,kind,held,seg_sx,seg_sy,seg_gx,seg_gy\n' + '\n'.join(nd.rows) + '\n')
    with open(map_path, 'w') as f:
        f.write('\n'.join(rows) + '\n')
    analyzer = os.path.join(get_package_prefix('mapf_exec'), 'lib', 'mapf_exec', 'analyze_run')
    an = json.loads(subprocess.run([analyzer, '--map', map_path, '--cell', str(cell), '--log', log_path],
                                   check=True, capture_output=True, text=True).stdout)
    rs = an['robots'][robot['name']]
    moves = [r for r in results if r['kind'] == 'MOVE']
    summary = {
        'scenario': sc['name'], 'max_linear': robot['max_linear'], 'max_angular': robot['max_angular'],
        'actions_done': sum(r['state'] == 'DONE' for r in results), 'actions_total': len(path) - 1,
        'max_arrive_err_m': max((r['arrive_err_m'] for r in moves), default=None),
        'mean_arrive_err_m': sum(r['arrive_err_m'] for r in moves) / max(len(moves), 1),
        'sim_seconds': t_end - (t_start or t_end), 'analysis': rs, 'steps': results,
    }
    track = max(rs['max_cross_m'], rs['max_overshoot_m'])
    checks = {
        '모든 동작 DONE': summary['actions_done'] == summary['actions_total'],
        '도착 오차 < 3 cm': summary['max_arrive_err_m'] is not None and summary['max_arrive_err_m'] < ARRIVE_TOL,
        '칸 이탈 0': rs['escape_samples'] == 0,
        '추종 오차 < 0.11 m': track < TRACK_TOL,
        '선반/벽 겹침 0': rs['shelf_or_wall_overlap_samples'] == 0,
    }
    summary['checks'] = checks
    summary['all_ok'] = bool(ok and all(checks.values()))
    with open(os.path.join(args.out_dir, 'summary.json'), 'w') as f:
        json.dump(summary, f, ensure_ascii=False, indent=1)
    print(f"\n{sc['name']} (최고 {robot['max_linear']} m/s, {robot['max_angular']} rad/s): "
          f"동작 {summary['actions_done']}/{summary['actions_total']}, 시뮬레이터 시간 {summary['sim_seconds']:.1f}초")
    print(f"  도착 오차 최대 {summary['max_arrive_err_m'] * 100:.2f} cm (평균 {summary['mean_arrive_err_m'] * 100:.2f} cm)")
    print(f"  옆 오차 최대 {rs['max_cross_m'] * 100:.2f} cm, 지나침 최대 {rs['max_overshoot_m'] * 100:.2f} cm, "
          f"칸 이탈 {rs['escape_samples']}회(최대 {rs['max_escape_m'] * 100:.2f} cm), 선반/벽 겹침 {rs['shelf_or_wall_overlap_samples']}회"
          f" — 기록 {rs['samples']}개")
    for k, v in checks.items():
        print(f"  [{'통과' if v else '실패'}] {k}")
    print('7-2 주행 시험:', '모두 통과' if summary['all_ok'] else '실패 있음')
    nd.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if summary['all_ok'] else 1)


if __name__ == '__main__':
    main()

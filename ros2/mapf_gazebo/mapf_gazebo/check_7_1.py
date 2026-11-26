"""7-1 점검 — Gazebo 월드와 로봇 위치가 시나리오 격자와 같은가 (docs/31 31.12, 31.19).

scenario.launch.py로 띄운 상태에서:

    ros2 run mapf_gazebo check_7_1 --scenario corridor10_r6_s1 --out result.json

점검 항목
  P. 출발 위치: 로봇마다 정답 위치(Gazebo)가 출발 칸 중심에서 3 cm 안, 방향이 start_yaw_deg에서 2° 안
  L. LiDAR 대조: 로봇마다 실제 LiDAR 거리 ↔ "격자 지도만 보고 계산한 거리"(선반 + 테두리 벽, grid.expected_range).
     다른 로봇에 가린 광선(다른 로봇 중심 0.15 m 안을 지나는 광선)은 뺀다. 차이 5 cm 안인 광선이 97% 이상이면 통과.
     세로를 안 뒤집는 실수 같은 "월드가 격자와 다른" 문제를 그림을 안 보고도 잡는다.
  E. RTF: 선반이 생긴 월드에서 시뮬레이터 시간 ÷ 실제 시간(가만히)
"""
import argparse
import json
import math
import statistics
import sys
import time

import rclpy
from gazebo_msgs.srv import GetEntityState
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

from mapf_gazebo import scenario_io
from mapf_gazebo.grid import blocked_boxes, cell_center, expected_range, ray_passes_near, wall_boxes

POS_TOL, YAW_TOL_DEG = 0.03, 2.0
RANGE_TOL, AGREE_MIN, OCCLUDE_R = 0.05, 0.97, 0.15


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Checker(Node):
    def __init__(self, names):
        super().__init__('check_7_1', parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.sim_now, self.scan, self.static = None, {}, {}
        self.create_subscription(Clock, '/clock', self._on_clock, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        for n in names:
            self.create_subscription(LaserScan, f'/{n}/scan', lambda m, n=n: self.scan.__setitem__(n, m), qos_profile_sensor_data)
        self.create_subscription(TFMessage, '/tf_static', self._on_tf_static,
                                 QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.gs = self.create_client(GetEntityState, '/gazebo/get_entity_state')

    def _on_clock(self, m):
        self.sim_now = m.clock.sec + m.clock.nanosec * 1e-9

    def _on_tf_static(self, m):
        for t in m.transforms:
            self.static[(t.header.frame_id, t.child_frame_id)] = t.transform

    def gt(self, name):
        req = GetEntityState.Request()
        req.name, req.reference_frame = name, 'world'
        fut = self.gs.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        r = fut.result()
        if r is None or not r.success:
            raise RuntimeError(f'{name} 정답 위치를 못 받음')
        p = r.state.pose
        return p.position.x, p.position.y, yaw_of(p.orientation)

    def scan_offset(self, name):
        """base_footprint → base_scan의 (x, y) — /tf_static의 두 연결(base_footprint→base_link→base_scan)을 더함(회전 없음)."""
        a = self.static.get((f'{name}/base_footprint', f'{name}/base_link'))
        b = self.static.get((f'{name}/base_link', f'{name}/base_scan'))
        if a is None or b is None:
            return None
        return a.translation.x + b.translation.x, a.translation.y + b.translation.y


def lidar_agreement(scan, pose, offset, boxes, others):
    """한 로봇의 스캔 하나를 격자 계산과 맞대 본다 → (맞은 비율, 비교한 광선 수, 차이 중앙값, 안 맞은 예시)."""
    x, y, yaw = pose
    ox = x + math.cos(yaw) * offset[0] - math.sin(yaw) * offset[1]
    oy = y + math.sin(yaw) * offset[0] + math.cos(yaw) * offset[1]
    far = scan.range_max - 0.05
    agree, used, errs, bad = 0, 0, [], []
    for k, r in enumerate(scan.ranges):
        a = yaw + scan.angle_min + k * scan.angle_increment
        exp = expected_range(ox, oy, a, boxes)
        if any(ray_passes_near(ox, oy, a, min(exp, scan.range_max), px, py, OCCLUDE_R) for px, py in others):
            continue
        used += 1
        meas_far = not math.isfinite(r) or r >= far
        exp_far = exp >= far
        if meas_far and exp_far:
            agree += 1
        elif not meas_far and not exp_far and abs(r - exp) < RANGE_TOL:
            agree += 1
            errs.append(abs(r - exp))
        elif len(bad) < 3:
            bad.append((round(math.degrees(scan.angle_min + k * scan.angle_increment)), round(r, 3), round(exp, 3)))
    return agree / max(used, 1), used, (statistics.median(errs) if errs else None), bad


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--scenario', default='corridor10_r6_s1')
    ap.add_argument('--rtf-seconds', type=float, default=10.0)
    ap.add_argument('--out', default='')
    args = ap.parse_args()

    sc = scenario_io.load(args.scenario)
    rows, cell, h = sc['map'], sc['cell_size'], len(sc['map'])
    names = [r['name'] for r in sc['robots']]
    boxes = blocked_boxes(rows, cell) + wall_boxes(len(rows[0]), h, cell)

    rclpy.init()
    nd = Checker(names)
    end = time.monotonic() + 90
    while time.monotonic() < end and (nd.sim_now is None or len(nd.scan) < len(names) or not nd.gs.service_is_ready()
                                      or any(nd.scan_offset(n) is None for n in names)):
        rclpy.spin_once(nd, timeout_sec=0.1)
    if nd.sim_now is None or len(nd.scan) < len(names):
        print(f'[실패] 준비 안 됨: clock={nd.sim_now is not None}, scan {len(nd.scan)}/{len(names)}대')
        sys.exit(2)
    # 생성 직후 살짝 떨어지며 흔들린 것이 가라앉도록 시뮬레이터 시간 2초 기다린 뒤 새 스캔으로
    s0 = nd.sim_now
    while nd.sim_now - s0 < 2.0:
        rclpy.spin_once(nd, timeout_sec=0.05)
    nd.scan.clear()
    while len(nd.scan) < len(names):
        rclpy.spin_once(nd, timeout_sec=0.05)

    res = {'scenario': sc['name'], 'robots': {}}
    ok = True
    poses = {n: nd.gt(n) for n in names}

    print(f"시나리오 {sc['name']}: {len(rows[0])}×{h}칸, 칸 {cell} m, 로봇 {len(names)}대")
    for r in sc['robots']:
        n = r['name']
        ex, ey = cell_center(r['start'][0], r['start'][1], h, cell)
        gx, gy, gyaw = poses[n]
        dpos = math.hypot(gx - ex, gy - ey)
        dyaw = abs(math.degrees(wrap(gyaw - math.radians(r['start_yaw_deg']))))
        p_ok = dpos < POS_TOL and dyaw < YAW_TOL_DEG
        others = [(poses[m][0], poses[m][1]) for m in names if m != n]
        rate, used, med, bad = lidar_agreement(nd.scan[n], poses[n], nd.scan_offset(n), boxes, others)
        l_ok = rate >= AGREE_MIN
        ok &= p_ok and l_ok
        res['robots'][n] = {'start_cell': r['start'], 'pos_err_m': dpos, 'yaw_err_deg': dyaw, 'P_ok': p_ok,
                            'lidar_agree': rate, 'lidar_rays': used, 'lidar_median_err_m': med, 'lidar_bad_examples': bad,
                            'L_ok': l_ok}
        print(f"  {n} 출발 {r['start'][1]}행 {r['start'][0]}열: [{'통과' if p_ok else '실패'}] P 위치 차이 {dpos * 100:.1f} cm, 방향 {dyaw:.1f}°"
              f"   [{'통과' if l_ok else '실패'}] L LiDAR {rate * 100:.1f}% 일치({used}개 광선, 차이 중앙값 "
              f"{(med or 0) * 100:.1f} cm){'' if l_ok else f', 안 맞은 예(각도°, 실제, 격자) {bad}'}")

    s0, w0 = nd.sim_now, time.monotonic()
    while time.monotonic() - w0 < args.rtf_seconds:
        rclpy.spin_once(nd, timeout_sec=0.05)
    rtf = (nd.sim_now - s0) / (time.monotonic() - w0)
    res['rtf_idle'], res['all_ok'] = rtf, bool(ok)
    print(f'[기록] E RTF (선반 있는 월드, {len(names)}대, 가만히): {rtf:.2f}')
    print('7-1 점검:', '모두 통과' if ok else '실패 있음')
    if args.out:
        with open(args.out, 'w') as f:
            json.dump(res, f, ensure_ascii=False, indent=2)
    nd.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()

"""7-0 환경 점검 (docs/31 31.12).

multi_tb3.launch.py로 N대를 띄운 상태에서 실행한다:

    ros2 run mapf_gazebo check_7_0 --num-robots 2 --out result.json

점검 항목
  A. 토픽 분리  : /robot_i/cmd_vel·odom·scan이 로봇마다 있고, 네임스페이스 없는 /cmd_vel·/odom·/scan은 없다
  B. TF 분리    : robot_i/odom → robot_i/base_footprint가 로봇마다 따로 있고, 접두어 없는 프레임이 없다
  C. 따로 움직임: robot_1에만 명령 → robot_1만 움직임, robot_2에만 명령 → robot_2만 움직임 (정답 위치로 확인)
  D. odom 드리프트: robot_1이 사각형을 돈 뒤, odom이 잰 이동과 정답 위치의 이동 차이
  E. RTF        : 시뮬레이터 시간 ÷ 실제 시간 (가만히 있을 때 / 모두 제자리 회전할 때)
정답 위치 = Gazebo의 /gazebo/get_entity_state (월드 파일의 gazebo_ros_state 플러그인).
"""
import argparse
import json
import math
import sys
import time

import rclpy
from gazebo_msgs.srv import GetEntityState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Checker(Node):
    def __init__(self, n):
        super().__init__('check_7_0', parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.n = n
        self.sim_now = None
        self.odom = {}
        self.tf_pairs = set()
        self.cmd = {i: self.create_publisher(Twist, f'/robot_{i}/cmd_vel', 10) for i in range(1, n + 1)}
        for i in range(1, n + 1):
            self.create_subscription(Odometry, f'/robot_{i}/odom', lambda m, i=i: self.odom.__setitem__(i, m), 10)
        self.create_subscription(TFMessage, '/tf', self._on_tf, 100)
        # 고정 관절(base_footprint→base_link, base_scan 등)은 /tf_static에 한 번만 나온다(늦게 붙어도 받으려면 transient_local).
        self.create_subscription(TFMessage, '/tf_static', self._on_tf,
                                 QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        # Gazebo의 /clock은 best-effort로 나온다. reliable로 받으려 하면 QoS가 안 맞아 하나도 안 온다.
        self.create_subscription(Clock, '/clock', self._on_clock,
                                 QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.gs = self.create_client(GetEntityState, '/gazebo/get_entity_state')

    def _on_tf(self, msg):
        for t in msg.transforms:
            self.tf_pairs.add((t.header.frame_id, t.child_frame_id))

    def _on_clock(self, msg):
        self.sim_now = msg.clock.sec + msg.clock.nanosec * 1e-9

    # ── 시간 ──────────────────────────────────────────────
    def spin_wall(self, sec):
        end = time.monotonic() + sec
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def spin_sim(self, sec, every=None, wall_limit=120.0):
        """시뮬레이터 시간으로 sec초 동안 돈다. every가 있으면 0.05초마다 부른다(명령 계속 보내기)."""
        start = self.sim_now
        wall_end = time.monotonic() + wall_limit
        while self.sim_now - start < sec:
            if time.monotonic() > wall_end:
                raise RuntimeError('시뮬레이터 시간이 흐르지 않음(Gazebo 멈춤?)')
            if every:
                every()
            rclpy.spin_once(self, timeout_sec=0.05)

    # ── 움직이기 ──────────────────────────────────────────
    def drive(self, ids, v, w, sec):
        tw = Twist()
        tw.linear.x, tw.angular.z = v, w
        self.spin_sim(sec, every=lambda: [self.cmd[i].publish(tw) for i in ids])
        self.stop(ids)

    def stop(self, ids):
        zero = Twist()
        self.spin_sim(0.8, every=lambda: [self.cmd[i].publish(zero) for i in ids])

    # ── 위치 ──────────────────────────────────────────────
    def gt(self, i):
        req = GetEntityState.Request()
        req.name, req.reference_frame = f'robot_{i}', 'world'
        fut = self.gs.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        r = fut.result()
        if r is None or not r.success:
            raise RuntimeError(f'robot_{i} 정답 위치를 못 받음')
        p = r.state.pose
        return p.position.x, p.position.y, yaw_of(p.orientation)

    def od(self, i):
        p = self.odom[i].pose.pose
        return p.position.x, p.position.y, yaw_of(p.orientation)


def delta_in_start_frame(a, b):
    """a → b 이동을 a의 방향 기준으로: (앞으로, 왼쪽으로, 회전)."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    c, s = math.cos(-a[2]), math.sin(-a[2])
    return c * dx - s * dy, s * dx + c * dy, wrap(b[2] - a[2])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--num-robots', type=int, default=2)
    ap.add_argument('--rtf-seconds', type=float, default=15.0)
    ap.add_argument('--skip-motion', action='store_true', help='A·B·E만 (대수를 늘려 RTF만 잴 때)')
    ap.add_argument('--out', default='')
    args = ap.parse_args()

    rclpy.init()
    nd = Checker(args.num_robots)
    n = args.num_robots
    res = {'num_robots': n}
    ok = True

    # 준비: 시계·odom·정답 위치 서비스가 다 올 때까지
    wall_end = time.monotonic() + 60
    while (nd.sim_now is None or len(nd.odom) < n or not nd.gs.service_is_ready()) and time.monotonic() < wall_end:
        rclpy.spin_once(nd, timeout_sec=0.1)
    if nd.sim_now is None or len(nd.odom) < n:
        print(f'[실패] 준비 안 됨: clock={nd.sim_now is not None}, odom {len(nd.odom)}/{n}대')
        sys.exit(2)
    nd.spin_wall(2.0)  # 토픽 목록·TF 모으기

    # A. 토픽
    topics = {name for name, _ in nd.get_topic_names_and_types()}
    missing = [f'/robot_{i}/{t}' for i in range(1, n + 1) for t in ('cmd_vel', 'odom', 'scan') if f'/robot_{i}/{t}' not in topics]
    bare = [t for t in ('/cmd_vel', '/odom', '/scan') if t in topics]
    a_ok = not missing and not bare
    res['A_topics'] = {'ok': a_ok, 'missing': missing, 'unnamespaced': bare}
    print(f"[{'통과' if a_ok else '실패'}] A 토픽 분리: 없는 것 {missing or '없음'}, 네임스페이스 없는 것 {bare or '없음'}")
    ok &= a_ok

    # B. TF
    # 로봇마다 있어야 할 연결: 바퀴 odom(diff drive), 몸·센서(고정 관절), 바퀴(움직이는 관절 = joint_states가 맞아야 나옴)
    need = []
    for i in range(1, n + 1):
        r = f'robot_{i}'
        need += [(f'{r}/odom', f'{r}/base_footprint'), (f'{r}/base_footprint', f'{r}/base_link'),
                 (f'{r}/base_link', f'{r}/base_scan'), (f'{r}/base_link', f'{r}/wheel_left_link'),
                 (f'{r}/base_link', f'{r}/wheel_right_link')]
    lacking = [p for p in need if p not in nd.tf_pairs]
    frames = {f for p in nd.tf_pairs for f in p}
    # 접두어가 robot_숫자/ 가 아니거나, 이름에 처리 안 된 자리표시자($, {) 같은 이상한 글자가 있으면 실패
    bad = sorted(f for f in frames
                 if not (f.startswith('robot_') and '/' in f and f.split('/')[0][6:].isdigit())
                 or any(ch in f for ch in '${}'))
    b_ok = not lacking and not bad
    res['B_tf'] = {'ok': b_ok, 'lacking': lacking, 'bad_names': bad, 'frames': sorted(frames)}
    print(f"[{'통과' if b_ok else '실패'}] B TF 분리: 프레임 {len(frames)}개, 빠진 연결 {lacking or '없음'}, 이상한 이름 {bad or '없음'}")
    ok &= b_ok

    if not args.skip_motion:
        # C. 따로 움직임
        def moved(before, after):
            return {i: (math.hypot(after[i][0] - before[i][0], after[i][1] - before[i][1]),
                        abs(math.degrees(wrap(after[i][2] - before[i][2])))) for i in before}
        g0 = {i: nd.gt(i) for i in range(1, n + 1)}
        nd.drive([1], 0.15, 0.0, 3.0)
        g1 = {i: nd.gt(i) for i in range(1, n + 1)}
        m1 = moved(g0, g1)
        c1 = m1[1][0] > 0.3 and all(m1[i][0] < 0.01 and m1[i][1] < 1.0 for i in m1 if i != 1)
        c2, m2 = True, {}
        if n >= 2:
            nd.drive([2], 0.0, 1.0, math.pi / 2)
            g2 = {i: nd.gt(i) for i in range(1, n + 1)}
            m2 = moved(g1, g2)
            c2 = m2[2][1] > 60 and all(m2[i][0] < 0.01 and m2[i][1] < 1.0 for i in m2 if i != 2)
        c_ok = c1 and c2
        res['C_separate'] = {'ok': c_ok, 'robot1_cmd': m1, 'robot2_cmd': m2}
        print(f"[{'통과' if c_ok else '실패'}] C 따로 움직임:")
        print('    robot_1에 직진 명령 → ' + ', '.join(f'robot_{i} {d:.3f} m/{a:.1f}°' for i, (d, a) in m1.items()))
        if m2:
            print('    robot_2에 회전 명령 → ' + ', '.join(f'robot_{i} {d:.3f} m/{a:.1f}°' for i, (d, a) in m2.items()))
        ok &= c_ok

        # D. odom 드리프트: 사각형(0.6 m 직진 + 90° 회전) × 4
        o_s, g_s = nd.od(1), nd.gt(1)
        for _ in range(4):
            nd.drive([1], 0.15, 0.0, 4.0)
            nd.drive([1], 0.0, 1.0, math.pi / 2)
        nd.spin_sim(0.5)
        o_e, g_e = nd.od(1), nd.gt(1)
        do, dg = delta_in_start_frame(o_s, o_e), delta_in_start_frame(g_s, g_e)
        pos_err = math.hypot(do[0] - dg[0], do[1] - dg[1])
        yaw_err = abs(math.degrees(wrap(do[2] - dg[2])))
        travelled = 4 * 0.6
        origin_gap = math.hypot(o_s[0] - g_s[0], o_s[1] - g_s[1])
        d_ok = pos_err < 0.02 and yaw_err < 1.0
        res['D_odom'] = {'ok': d_ok, 'pos_err_m': pos_err, 'yaw_err_deg': yaw_err, 'approx_travel_m': travelled,
                         'odom_start': o_s, 'gt_start': g_s, 'odom_vs_world_origin_gap_m': origin_gap}
        print(f"[{'통과' if d_ok else '실패'}] D odom 드리프트: 약 {travelled:.1f} m 사각형 후 위치 차이 {pos_err * 100:.2f} cm, 방향 차이 {yaw_err:.2f}°"
              f" (기준 2 cm, 1°)")
        print(f"    odom 원점: 출발 때 odom과 정답 위치 차이 {origin_gap:.3f} m → "
              + ('odom은 월드 좌표 그대로' if origin_gap < 0.02 else 'odom은 생성 위치가 원점(월드로 바꾸려면 생성 위치를 더함)'))
        ok &= d_ok

    # E. RTF
    def rtf(every=None):
        s0, w0 = nd.sim_now, time.monotonic()
        end = w0 + args.rtf_seconds
        while time.monotonic() < end:
            if every:
                every()
            rclpy.spin_once(nd, timeout_sec=0.05)
        return (nd.sim_now - s0) / (time.monotonic() - w0)
    idle = rtf()
    spin = Twist()
    spin.angular.z = 0.5
    moving = rtf(lambda: [nd.cmd[i].publish(spin) for i in range(1, n + 1)])
    nd.stop(list(range(1, n + 1)))
    res['E_rtf'] = {'idle': idle, 'all_rotating': moving}
    print(f'[기록] E RTF ({n}대): 가만히 {idle:.2f}, 모두 제자리 회전 {moving:.2f}  (1.0 = 실시간)')

    res['all_ok'] = bool(ok)
    if args.out:
        with open(args.out, 'w') as f:
            json.dump(res, f, ensure_ascii=False, indent=2)
    print('7-0 점검:', '모두 통과' if ok else '실패 있음')
    nd.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()

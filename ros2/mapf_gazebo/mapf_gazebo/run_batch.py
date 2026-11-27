"""여러 3D 실행을 차례로: Gazebo 띄우기 → run_7_4 → Gazebo 끄기 (7-4, 7-6 본 실험용, docs/31 31.23).

    ros2 run mapf_gazebo run_batch --out-root results/7_4 --record \\
        corridor10_r6_s1_ob1:ours corridor10_r6_s1_ob1:full

- 작업 하나 = "시나리오:방법". 결과는 <out-root>/<시나리오>_<방법>/ (녹화는 --record면 그 안의 bag/).
- launch는 새 프로세스 그룹으로 띄우고, 끌 때 그 그룹에만 SIGINT → 안 꺼지면 SIGTERM. 사용자가 띄운 Gazebo는 건드리지 않는다
  (시작 전에 gzserver가 이미 떠 있으면 멈춤 — CLAUDE.md "AI가 Gazebo를 띄우고 끌 때").
  셸에서 `&`로 띄운 launch는 SIGINT를 무시해서 안 꺼진다(2026-11-27 겪음) — 그래서 파이썬으로 띄운다.
"""
import argparse
import os
import signal
import subprocess
import sys
import time


def gz_running():
    return subprocess.run(['pgrep', '-x', 'gzserver'], capture_output=True).returncode == 0


def stop(proc, timeout=40):
    pgid = os.getpgid(proc.pid)
    for sig, wait in ((signal.SIGINT, timeout), (signal.SIGTERM, 15), (signal.SIGKILL, 5)):
        try:
            os.killpg(pgid, sig)
        except ProcessLookupError:
            return
        end = time.monotonic() + wait
        while time.monotonic() < end:
            if proc.poll() is not None and not gz_running():
                return
            time.sleep(0.5)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('jobs', nargs='+', help='시나리오:방법 (방법 = ours | full)')
    ap.add_argument('--out-root', required=True)
    ap.add_argument('--record', action='store_true')
    ap.add_argument('--settle', type=float, default=8.0, help='launch 뒤 기다리는 초')
    ap.add_argument('--extra', default='', help='run_7_4에 그대로 넘길 인자. -로 시작하면 = 로 붙일 것(예: --extra="--t-wait 0")')
    ap.add_argument('--pass-through', action='store_true', help='곧은 길은 칸마다 서지 않음(31.24) — run_7_4 --pass-through')
    ap.add_argument('--robust-estop', action='store_true', help='비상 정지 v2(31.24 보충) — launch robust_estop:=true')
    ap.add_argument('--detection', choices=['simulated', 'lidar'], default='simulated', help='장애물 발견(31.25)')
    a = ap.parse_args()
    os.makedirs(a.out_root, exist_ok=True)
    for job in a.jobs:
        scenario, method = job.split(':')
        if gz_running():
            print('[멈춤] gzserver가 이미 떠 있음(사용자 것일 수 있음) — 끄고 다시 실행하세요')
            sys.exit(2)
        out = os.path.join(a.out_root, f'{scenario}_{method}')
        os.makedirs(out, exist_ok=True)
        cmd = ['ros2', 'launch', 'mapf_gazebo', 'scenario.launch.py', f'scenario:={scenario}',
               f'robust_estop:={str(a.robust_estop).lower()}']
        if a.record:
            cmd.append(f'record:={os.path.abspath(os.path.join(out, "bag"))}')
        with open(os.path.join(out, 'launch.log'), 'w') as log:
            proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            try:
                time.sleep(a.settle)
                print(f'\n===== {scenario} [{method}] =====', flush=True)
                subprocess.run(['ros2', 'run', 'mapf_gazebo', 'run_7_4', '--scenario', scenario, '--method', method,
                                '--out-dir', out] + a.extra.split() + (['--pass-through'] if a.pass_through else [])
                               + ['--detection', a.detection])
            finally:
                stop(proc)
    print('\n일괄 실행 끝, gzserver', '남아 있음!' if gz_running() else '없음')


if __name__ == '__main__':
    main()

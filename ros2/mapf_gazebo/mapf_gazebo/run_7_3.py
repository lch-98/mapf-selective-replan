"""7-3·7-4 여러 대 실행 시험 (docs/31 31.12, 31.22, 31.23). run_7_4도 같은 프로그램.

scenario.launch.py로 시나리오(예: corridor10_r6_s1)를 띄운 상태에서:

    ros2 run mapf_gazebo run_7_3 --scenario corridor10_r6_s1 --out-dir results/7_3/corridor10_r6_s1

1. mapf_executive(C++)를 실행: 저장된 처음 계획 → 계획 검사 (가) → ADG → 로봇별 동작 내주기 → 기록.
2. 끝나면 analyze_run(C++ 몸 검사)으로 몸 겹침·칸 이탈·선반/벽 겹침·로봇 사이 최소 거리를 계산.
3. 기준(31.12 7-3): 모든 동작 완료, 계획 검사 통과, 몸 겹침 0, 접촉 0, 칸 이탈 0, 선반/벽 겹침 0, 비상 정지 0, 칸 사용 순서 = 계획.
--drop-cross-edges는 변이 검사용(로봇 사이 약속을 뺌) — 기준을 못 넘어야 정상.
"""
import argparse
import json
import os
import subprocess
import sys

from ament_index_python.packages import get_package_prefix

from mapf_gazebo import scenario_io


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--scenario', default='corridor10_r6_s1')
    ap.add_argument('--out-dir', required=True)
    ap.add_argument('--drop-cross-edges', action='store_true', help='변이 검사용: 로봇 사이 약속을 뺌')
    ap.add_argument('--method', choices=['ours', 'full'], default='ours', help='장애물 재계획 방법(7-4)')
    ap.add_argument('--d-detect', type=float, default=2.0)
    ap.add_argument('--t-wait', type=float, default=3.0)
    ap.add_argument('--analyze-only', action='store_true', help='실행기는 안 돌리고 out-dir의 기록만 다시 분석')
    ap.add_argument('--pass-through', action='store_true', help='곧은 길은 칸마다 서지 않고 지나감(31.24)')
    ap.add_argument('--detection', choices=['simulated', 'lidar'], default='simulated', help='장애물 발견(31.25)')
    a = ap.parse_args()

    path = scenario_io.resolve(a.scenario)
    sc = scenario_io.load(a.scenario)
    os.makedirs(a.out_dir, exist_ok=True)
    cmd = ['ros2', 'run', 'mapf_ros', 'mapf_executive', '--ros-args', '-p', 'use_sim_time:=true',
           '-p', f'scenario:={os.path.abspath(path)}', '-p', f'out_dir:={os.path.abspath(a.out_dir)}',
           '-p', f'method:={a.method}', '-p', f'd_detect:={a.d_detect}', '-p', f't_wait:={a.t_wait}',
           '-p', f'pass_through:={str(a.pass_through).lower()}', '-p', f'detection:={a.detection}']
    if a.drop_cross_edges:
        cmd += ['-p', 'debug_drop_cross_edges:=true', '-p', 'total_timeout_s:=600.0']
    if not a.analyze_only:
        subprocess.run(cmd)  # 끝 코드는 아래에서 executive.json으로 다시 판단

    ex = json.load(open(os.path.join(a.out_dir, 'executive.json')))
    analyzer = os.path.join(get_package_prefix('mapf_exec'), 'lib', 'mapf_exec', 'analyze_run')
    an = json.loads(subprocess.run([analyzer, '--map', os.path.join(a.out_dir, 'map.txt'), '--cell', str(sc['cell_size']),
                                    '--log', os.path.join(a.out_dir, 'run.csv')],
                                   check=True, capture_output=True, text=True).stdout)
    rs, ps = an['robots'], an['pairs']
    overlap = sum(p['overlap_samples'] for p in ps.values())
    min_gap = min((p['min_distance_m'] for p in ps.values()), default=float('nan'))
    escape = sum(r['escape_samples'] for r in rs.values())
    shelf = sum(r['shelf_or_wall_overlap_samples'] for r in rs.values())
    checks = {
        '모든 동작 완료': ex['all_done'],
        '계획 검사 (가) 통과': ex['plan_check_violations'] == 0,
        '몸 겹침 0 (기록)': overlap == 0,
        '접촉 0 (Gazebo 센서)': ex['contacts'] == 0,
        '칸 이탈 0': escape == 0,
        '선반/벽 겹침 0': shelf == 0,
        '비상 정지 0': ex['estops'] == 0,
        '칸 사용 순서 = 계획': len(ex['order_mismatches']) == 0,
        '늦은 취소 0': ex.get('late_cancels', 0) == 0,
    }
    if ex.get('detection') == 'lidar':
        checks['오탐 확정 0 (LiDAR)'] = ex.get('phantoms_confirmed', 0) == 0
    if ex.get('obstacles'):
        checks['재계획 성공 + 새 계획 검사 통과'] = all(o.get('success', False) and o.get('plan_check_violations', 1) == 0
                                                  for o in ex['obstacles'] if o.get('detect_s', -1) >= 0)
    moves = sum(1 for x in ex.get('action_log', []) if x['kind'] == 'MOVE')
    passed = sum(1 for x in ex.get('action_log', []) if x['kind'] == 'MOVE' and x.get('passed_through'))
    summary = {'scenario': sc['name'], 'method': a.method, 'drop_cross_edges': a.drop_cross_edges, 'checks': checks,
               'obstacles': ex.get('obstacles', []), 'state_time_s': {k: v['state_time_s'] for k, v in ex['robots'].items()},
               'all_ok': all(checks.values()), 'body_overlap_samples': overlap, 'min_body_gap_m': min_gap,
               'escape_samples': escape, 'shelf_overlap_samples': shelf, 'analysis': an,
               'makespan_s': ex['makespan_s'], 'plan_makespan_steps': ex['plan_makespan_steps'],
               'cross_edges': ex['cross_edges'], 'actions': ex.get('actions', len(ex.get('action_log', []))),
               'pass_through': ex.get('pass_through', False), 'moves': moves, 'passed_moves': passed,
               'cancels': sum(r.get('cancels', 0) for r in ex['robots'].values()),
               'sum_finish_s': sum(max(0.0, r['finish_s']) for r in ex['robots'].values()),
               'detection': ex.get('detection', 'simulated'), 'phantoms_confirmed': ex.get('phantoms_confirmed', 0),
               'held_cell_hits': ex.get('held_cell_hits', 0), 'phantom_detections': ex.get('phantom_detections', 0),
               'late_scans': ex.get('late_scans', 0), 't_wait': ex.get('t_wait'), 'd_detect': ex.get('d_detect')}
    json.dump(summary, open(os.path.join(a.out_dir, 'summary.json'), 'w'), ensure_ascii=False, indent=1)

    print(f"\n{sc['name']}{' [변이 검사: 약속 뺌]' if a.drop_cross_edges else ''}: 로봇 {len(sc['robots'])}대, "
          f"동작 {ex.get('actions', len(ex.get('action_log', [])))}개, 로봇 사이 약속 {ex['cross_edges']}개")
    print(f"  {'모두 완료' if ex['all_done'] else '중단: ' + ex['abort_reason']} — 다 끝난 시간 {ex['makespan_s']:.1f}초 "
          f"(계획 {ex['plan_makespan_steps']}스텝 × Δt_nom {sc['dt_nom_s']}초 = {ex['plan_makespan_s_at_dt_nom']:.1f}초)")
    if ex.get('pass_through'):
        print(f"  곧게 지나가기: 이동 {moves}번 중 {passed}번은 칸 중심에서 안 서고 지나감, 취소 {summary['cancels']}번")
    print(f"  몸 사이 최소 거리 {min_gap * 100:.1f} cm, 몸 겹침 {overlap}회, 접촉 {ex['contacts']}회, 칸 이탈 {escape}회")
    for name, r in sorted(ex['robots'].items()):
        print(f"    {name}: 끝 {r['finish_s']:.1f}초, 약속 때문에 기다림 {r['adg_wait_s']:.1f}초, "
              f"도착 오차 최대 {r['max_pos_err_m'] * 100:.2f} cm, 접촉 {r['contacts']}")
    for o in ex.get('obstacles', []):
        c = o['cell']
        if o.get('detect_s', -1) < 0:
            print(f"  장애물 {c[1]}행 {c[0]}열: 생성 {o.get('spawn_s', -1):.1f}초, 아무도 발견 못 함(재계획 없음)")
            continue
        print(f"  장애물 {c[1]}행 {c[0]}열: 생성 {o['spawn_s']:.1f}초 → 발견 {o['detect_s']:.1f}초({o['detected_by']}) → 확정 "
              f"{o['confirm_s']:.1f}초 → 재계획 {o['replan_s']:.1f}초 [{o['method']}] 계산 {o['compute_ms']:.2f} ms, "
              f"{'성공' if o['success'] else '실패'}, 길이 바뀐 로봇 {len(o.get('changed_robots', []))}대 {o.get('changed_robots', [])}"
              + (f", Tier {o['escalation_tier']}, 다시 계획 {o['replanned_ids']}, 구조 {o['rescued_ids']}" if 'escalation_tier' in o else ''))
    if ex.get('detection') == 'lidar':
        for o in ex.get('obstacles', []):
            c = o['cell']
            print(f"  LiDAR {c[1]}행 {c[0]}열{' (시나리오에 없음 = 오탐)' if o.get('phantom') else ''}: 처음 발견 "
                  f"{o.get('first_detect_s', -1):.1f}초 / 흉내 규칙이면 {o.get('sim_detect_s', -1):.1f}초, 버림 {o.get('drops', 0)}번, "
                  f"확정 때 {o.get('bins_seen', 0)}/{o.get('bins_total', 0)}칸 보임")
        print(f"  로봇이 가진 칸에 찍혀 버린 점 묶음: {ex.get('held_cell_hits', 0)}번, 시나리오에 없는 칸 후보(발견) "
              f"{ex.get('phantom_detections', 0)}번, odom이 늦어 그냥 처리한 스캔 {ex.get('late_scans', 0)}개")
    st = {}
    for v in ex['robots'].values():
        for k, x in v['state_time_s'].items():
            st[k] = st.get(k, 0.0) + x
    print('  상태별 시간 합(초): ' + ', '.join(f'{k} {v:.1f}' for k, v in sorted(st.items())))
    if ex['order_mismatches']:
        print(f"  칸 사용 순서가 다른 칸: {ex['order_mismatches'][:3]}")
    if ex['contact_examples']:
        print(f"  접촉 예: {ex['contact_examples'][:3]}")
    for k, v in checks.items():
        print(f"  [{'통과' if v else '실패'}] {k}")
    print('7-3 실행 시험:', '모두 통과' if summary['all_ok'] else '실패 있음')
    sys.exit(0 if summary['all_ok'] else 1)


if __name__ == '__main__':
    main()

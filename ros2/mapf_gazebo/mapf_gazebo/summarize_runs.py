"""여러 3D 실행 결과(run_batch 폴더)를 표로 모은다 — 7-6 본 실험·32장 (docs/32).

    ros2 run mapf_gazebo summarize_runs results/7_6/base [다른 폴더 ...] [--csv 모음.csv]

폴더 안의 */summary.json + executive.json을 읽어 실행마다 한 줄(markdown 표), --csv면 CSV도 쓴다.
장애물 처리 시간(초): 생성 → 발견 → 확정 → 재계획. "멈춤 대기" = 확정 → 재계획(이동 중인 로봇이 칸 중심에 설 때까지, 31.6 ①).
"""
import argparse
import csv
import glob
import json
import os


def rows_of(root):
    out = []
    for sp in sorted(glob.glob(os.path.join(root, '*', 'summary.json'))):
        d = os.path.dirname(sp)
        s = json.load(open(sp))
        ex = json.load(open(os.path.join(d, 'executive.json')))
        obs = [o for o in ex.get('obstacles', []) if o.get('replan_s', -1) >= 0]
        o = obs[0] if obs else {}
        st = {}
        for v in ex['robots'].values():
            for k, x in v['state_time_s'].items():
                st[k] = st.get(k, 0.0) + x
        out.append({
            'group': os.path.basename(os.path.normpath(root)), 'scenario': s['scenario'], 'method': s['method'],
            'ok': s['all_ok'], 'done': ex['all_done'], 'abort': ex.get('abort_reason', ''),
            'makespan_s': round(ex['makespan_s'], 1), 'sum_finish_s': round(s.get('sum_finish_s', 0.0), 1),
            'replans': ex['replans'], 'phantoms': ex.get('phantoms_confirmed', 0),
            'phantom_cands': ex.get('phantom_detections', 0),
            'changed': len(o.get('changed_robots', [])) if o else 0, 'compute_ms': round(o.get('compute_ms', 0.0), 2) if o else '',
            'tier': o.get('escalation_tier', '') if o else '', 'new_plan_steps': o.get('new_plan_steps', '') if o else '',
            'detect_s': round(o['detect_s'], 1) if o else '', 'confirm_s': round(o['confirm_s'], 1) if o else '',
            'replan_s': round(o['replan_s'], 1) if o else '',
            'stop_wait_s': round(o['replan_s'] - o['confirm_s'], 1) if o else '',
            'hold_s': round(st.get('HOLD', 0.0), 1), 'stop_s': round(st.get('STOP', 0.0), 1), 'yield_s': round(st.get('YIELD', 0.0), 1),
            'min_gap_cm': round(s['min_body_gap_m'] * 100, 1), 'contacts': ex['contacts'], 'estops': ex['estops'],
            'escape': s['escape_samples'], 'overlap': s['body_overlap_samples'],
            'passed': f"{s.get('passed_moves', 0)}/{s.get('moves', 0)}", 'cancels': s.get('cancels', 0),
            't_wait': ex.get('t_wait'), 'd_detect': ex.get('d_detect'), 'detection': ex.get('detection', 'simulated'),
        })
    return out


COLS = ['group', 'scenario', 'method', 'ok', 'makespan_s', 'sum_finish_s', 'changed', 'tier', 'new_plan_steps', 'compute_ms',
        'detect_s', 'confirm_s', 'stop_wait_s', 'hold_s', 'stop_s', 'yield_s', 'min_gap_cm', 'contacts', 'estops', 'escape',
        'overlap', 'phantoms', 'phantom_cands', 'passed', 'cancels', 'abort']


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('roots', nargs='+')
    ap.add_argument('--csv')
    a = ap.parse_args()
    rows = [r for root in a.roots for r in rows_of(root)]
    print('| ' + ' | '.join(COLS) + ' |')
    print('|' + '---|' * len(COLS))
    for r in rows:
        print('| ' + ' | '.join(str(r[c]) for c in COLS) + ' |')
    if a.csv:
        with open(a.csv, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0].keys()) if rows else COLS)
            w.writeheader()
            w.writerows(rows)


if __name__ == '__main__':
    main()

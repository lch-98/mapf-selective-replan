"""주행 기록(run.csv)을 지도 위에 그린다 — 2D 그림과 같은 방향(0행이 위). 7-2부터 결과 확인용.

    ros2 run mapf_gazebo plot_run --scenario follow_corridor10_slow --log results/7_2/slow/run.csv --out results/7_2/slow/track.png

- 회색 칸 경계, 주황 = 선반, 선 = 로봇 회전 중심이 실제로 지나간 자리(로봇마다 색), 점선 = 칸 중심을 잇는 계획 경로(test_path가 있으면)
- 확대 그림(오른쪽): 칸 중심선에서 벗어난 거리(옆 오차)를 cm로
"""
import argparse
import csv

import matplotlib
matplotlib.use('Agg')
matplotlib.rcParams['font.family'] = ['NanumSquare', 'Noto Sans CJK TC', 'DejaVu Sans']
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Rectangle  # noqa: E402

from mapf_gazebo import scenario_io  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--scenario', required=True)
    ap.add_argument('--log', required=True)
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    sc = scenario_io.load(a.scenario)
    rows, cell = sc['map'], sc['cell_size']
    w, h = len(rows[0]), len(rows)

    tracks = {}
    with open(a.log) as f:
        for r in csv.DictReader(f):
            # 월드 m → 격자 좌표(열 = 오른쪽, 행 = 아래로, 칸 중심이 정수 + 0.5)
            col = float(r['x']) / cell
            row = h - float(r['y']) / cell
            tracks.setdefault(r['robot'], []).append((col, row))

    fig, ax = plt.subplots(figsize=(w * 0.6 + 1, h * 0.6 + 1))
    for y, line in enumerate(rows):
        for x, ch in enumerate(line):
            ax.add_patch(Rectangle((x, y), 1, 1, facecolor='#d98c33' if ch == '#' else 'white', edgecolor='#cccccc', lw=0.5))
    if 'test_path' in sc:
        p = sc['test_path']
        ax.plot([c[0] + 0.5 for c in p], [c[1] + 0.5 for c in p], ':', color='gray', lw=1.5, label='계획(칸 중심)')
    colors = plt.cm.tab10.colors
    for i, (name, pts) in enumerate(sorted(tracks.items())):
        ax.plot([q[0] for q in pts], [q[1] for q in pts], color=colors[i % 10], lw=1.2, label=f'{name} 실제')
    ax.set_xlim(0, w)
    ax.set_ylim(h, 0)
    ax.set_aspect('equal')
    ax.set_xticks([i + 0.5 for i in range(w)], [str(i) for i in range(w)])
    ax.set_yticks([i + 0.5 for i in range(h)], [str(i) for i in range(h)])
    ax.tick_params(length=0)
    ax.set_xlabel('열 (x)')
    ax.set_ylabel('행 (y)')
    ax.legend(loc='upper right', fontsize=7)
    ax.set_title(f"{sc['name']} 실제 궤적", fontsize=9)
    fig.tight_layout()
    fig.savefig(a.out, dpi=110)
    print(a.out)


if __name__ == '__main__':
    main()

"""시나리오를 2D GUI와 같은 방향(0번 줄이 위)으로 그린다 — Gazebo 화면(위에서 본 모습)과 눈으로 맞대 보는 용도 (7-1).

    ros2 run mapf_gazebo draw_scenario corridor10_r6_s1 --out corridor10_r6_s1.png

- 주황 칸 = 선반('#'), 동그라미 = 출발 칸(숫자 = robot_번호), 네모 = 목적지, 선 = 처음 계획 경로, 화살표 = 처음 방향
- Gazebo 화면에서는 위가 +y(북). grid.cell_center가 세로를 뒤집으므로 이 그림과 같은 모양이어야 정상.
"""
import argparse
import math

import matplotlib
matplotlib.use('Agg')
matplotlib.rcParams['font.family'] = ['NanumSquare', 'Noto Sans CJK TC', 'DejaVu Sans']  # 한글 제목
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Rectangle  # noqa: E402

from mapf_gazebo import scenario_io  # noqa: E402


def draw(sc, out):
    rows = sc['map']
    w, h = len(rows[0]), len(rows)
    fig, ax = plt.subplots(figsize=(w * 0.6 + 1, h * 0.6 + 1))
    for y, row in enumerate(rows):
        for x, ch in enumerate(row):
            ax.add_patch(Rectangle((x, y), 1, 1, facecolor='#d98c33' if ch == '#' else 'white', edgecolor='#cccccc', lw=0.5))
    colors = plt.cm.tab10.colors
    for r in sc['robots']:
        c = colors[r['id'] % 10]
        path = sc.get('initial_plan', {}).get(str(r['id']), [])
        if path:
            ax.plot([p[0] + 0.5 for p in path], [p[1] + 0.5 for p in path], color=c, lw=2, alpha=0.6)
        sx, sy = r['start'][0] + 0.5, r['start'][1] + 0.5
        ax.add_patch(plt.Circle((sx, sy), 0.32, color=c))
        ax.text(sx, sy, str(r['id'] + 1), ha='center', va='center', color='white', fontsize=10, weight='bold')
        yaw = math.radians(r['start_yaw_deg'])
        ax.arrow(sx, sy, 0.45 * math.cos(yaw), -0.45 * math.sin(yaw), head_width=0.15, color='black', length_includes_head=True)
        gx, gy = r['goal'][0] + 0.5, r['goal'][1] + 0.5
        ax.add_patch(Rectangle((gx - 0.3, gy - 0.3), 0.6, 0.6, fill=False, edgecolor=c, lw=2.5))
        ax.text(gx, gy, str(r['id'] + 1), ha='center', va='center', color=c, fontsize=9, weight='bold')
    ax.set_xlim(0, w)
    ax.set_ylim(h, 0)  # 0번 줄이 위 (2D GUI와 같음)
    ax.set_aspect('equal')
    ax.set_xticks(range(w))
    ax.set_yticks(range(h))
    ax.set_title(f"{sc['name']}  (동그라미=출발, 네모=목적지, 화살표=처음 방향)", fontsize=9)
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('scenario')
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    draw(scenario_io.load(a.scenario), a.out)
    print(a.out)


if __name__ == '__main__':
    main()

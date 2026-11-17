"""그림: 안전망 없이 스스로 푼 비율 — 현재 엔진(v3) 기준.

예전 그림(tools/plots_large/success_rate.png, 2026-09-13)은 (1) 원래 방식만, (2) 동점 규칙을 고치기 전 엔진으로
그린 것이라 지금 상태와 다르다. 이 그림은 tools/results/v3_service/의 같은 시나리오로
전체 재계획 / 원래 방식 / 우리 방법을 나란히 그린다.

실행: python tools/plot_success_v3.py  →  tools/plots_study/success_v3.png
"""
import csv
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RES = os.path.join(ROOT, "tools", "results", "v3_service")
OUT = os.path.join(ROOT, "tools", "plots_study", "success_v3.png")

plt.rcParams["font.family"] = "Malgun Gothic"
plt.rcParams["axes.unicode_minus"] = False

# 범주 색: 고정 순서(파랑, 주황, 청록). 글자는 검정/회색.
SERIES = [
    ("전체 재계획 (성공)", "#2a78d6"),
    ("원래 방식 (안전망 없이)", "#eb6834"),
    ("우리 방법 (안전망 없이)", "#1baf7a"),
]
TEXT, MUTED, GRID = "#1f1f1e", "#6b6a63", "#e4e3dd"


def load(name):
    with open(os.path.join(RES, name + ".csv"), encoding="utf-8") as f:
        return list(csv.DictReader(f))


def rates(original, ours, map_name, counts):
    full, orig, us = [], [], []
    for n in counts:
        ro = [r for r in original if r["map"] == map_name and r["num_agents"] == n]
        ru = [r for r in ours if r["map"] == map_name and r["num_agents"] == n]
        alone = lambda rs: sum(r["selective_replan_ok"] == "1" and int(r["escalation_tier"]) >= 0 for r in rs)
        full.append(100 * sum(r["full_replan_ok"] == "1" for r in ru) / len(ru))
        orig.append(100 * alone(ro) / len(ro))
        us.append(100 * alone(ru) / len(ru))
    return full, orig, us


def main():
    panels = [
        ("32×32 · 넓은 맵", "open", ["5", "10", "20", "40"], "order_priority_t1", "idea3_path_t1"),
        ("32×32 · 좁은 통로 맵", "corridor", ["5", "10", "20", "40"], "order_priority_t1", "idea3_path_t1"),
        ("64×64 · 넓은 맵", "open", ["50", "100", "150", "200"], "order_large_priority_t1", "idea3_large_path_t1"),
        ("64×64 · 좁은 통로 맵", "corridor", ["50", "100", "150", "200"], "order_large_priority_t1", "idea3_large_path_t1"),
    ]
    fig, axes = plt.subplots(2, 2, figsize=(12, 8), sharey=True)
    fig.patch.set_facecolor("white")
    width = 0.26
    for ax, (title, map_name, counts, orig_name, ours_name) in zip(axes.flat, panels):
        values = rates(load(orig_name), load(ours_name), map_name, counts)
        xs = range(len(counts))
        for k, ((label, color), vals) in enumerate(zip(SERIES, values)):
            pos = [x + (k - 1) * width for x in xs]
            ax.bar(pos, vals, width=width - 0.03, color=color, label=label, zorder=3)
            if k == 1:  # 원래 방식만 숫자를 단다(떨어지는 쪽)
                for p, v in zip(pos, vals):
                    if v < 95:
                        ax.text(p, v + 1.5, f"{v:.0f}", ha="center", va="bottom", fontsize=9, color=TEXT)
        ax.set_title(title, loc="left", fontsize=12, color=TEXT)
        ax.set_xticks(list(xs))
        ax.set_xticklabels([f"{n}대" for n in counts], color=MUTED)
        ax.set_ylim(0, 108)
        ax.yaxis.grid(True, color=GRID, zorder=0)
        ax.tick_params(colors=MUTED, length=0)
        for side in ("top", "right", "left"):
            ax.spines[side].set_visible(False)
        ax.spines["bottom"].set_color(GRID)
    for ax in axes[:, 0]:
        ax.set_ylabel("시나리오 비율 (%)", color=MUTED)
    handles, labels = axes[0, 0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", ncol=3, frameon=False, bbox_to_anchor=(0.5, 0.95))
    fig.suptitle("안전망(전체 재계획) 없이 스스로 푼 비율 — 현재 엔진, 시나리오 50개씩",
                 x=0.02, ha="left", fontsize=14, color=TEXT)
    fig.text(0.02, 0.01, "같은 시나리오. 전체 재계획은 자기 성공률. 데이터: tools/results/v3_service/",
             fontsize=9, color=MUTED)
    fig.tight_layout(rect=(0, 0.03, 1, 0.91))
    fig.savefig(OUT, dpi=150)
    print(OUT)


if __name__ == "__main__":
    main()

"""23장 실험 그림 3장 (tools/plots_study/v4_*.png).

  v4_1_compute_time.png  — 재계획 계산 시간(한 번에 하나씩 잼): 전체 재계획 vs 우리 방법
  v4_2_latency_loss.png  — 로봇당 총 늦어짐(경로 + 계산 동안 멈춤) vs 한 칸 이동 시간 Δ
  v4_3_standing.png      — 작업 시간 때문에 결과가 바뀐 시나리오 수: 기존 장면 vs standing 장면

실행: python tools/plot_v4.py   (128x128 결과 파일이 있으면 함께 그린다)
"""
import csv
import math
import os
import statistics as st

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RES = os.path.join(ROOT, "tools", "results")
OUT = os.path.join(ROOT, "tools", "plots_study")

plt.rcParams["font.family"] = "Malgun Gothic"
plt.rcParams["axes.unicode_minus"] = False
FULL, ORIG, OURS = "#2a78d6", "#eb6834", "#1baf7a"
TEXT, MUTED, GRID = "#1f1f1e", "#6b6a63", "#e4e3dd"
DELTAS = [0.25, 0.5, 1.0, 2.0]


def load(*parts):
    path = os.path.join(RES, *parts)
    if not os.path.exists(path):
        return None
    with open(path, encoding="utf-8") as f:
        return list(csv.DictReader(f))


def style(ax):
    ax.yaxis.grid(True, color=GRID, zorder=0)
    ax.tick_params(colors=MUTED, length=0)
    for side in ("top", "right", "left"):
        ax.spines[side].set_visible(False)
    ax.spines["bottom"].set_color(GRID)


def stall(ms, d):
    return max(0, math.ceil(ms / 1000.0 / d) - 1)


def loss_per_robot(rows, n, d, col_ms, col_soc):
    both = [r for r in rows if r["num_agents"] == n and r["full_replan_ok"] == "1" and r["selective_replan_ok"] == "1"]
    vals = [(int(r[col_soc]) - int(r["initial_soc"])) + stall(float(r[col_ms]), d) * int(r["moving_at_t"]) for r in both]
    return st.mean(vals) / int(n)


def datasets():
    sets = [("64×64", load("v4_latency", "ours_large_t1thread.csv"), load("v4_latency", "original_large_t1thread.csv"))]
    huge = load("v4_latency", "ours_huge_t1thread.csv")
    if huge:
        sets.append(("128×128", huge, load("v4_latency", "original_huge_t1thread.csv")))
    return sets


def fig_compute_time():
    sets = datasets()
    fig, axes = plt.subplots(1, len(sets), figsize=(max(9, 6.5 * len(sets)), 4.8), squeeze=False)
    for ax, (name, ours, _) in zip(axes[0], sets):
        counts = sorted({r["num_agents"] for r in ours}, key=int)
        xs = range(len(counts))
        full_med = [st.median(float(r["full_replan_ms"]) for r in ours if r["num_agents"] == n) for n in counts]
        ours_med = [st.median(float(r["selective_replan_ms"]) for r in ours if r["num_agents"] == n) for n in counts]
        w = 0.36
        ax.bar([x - w / 2 for x in xs], full_med, w - 0.03, color=FULL, label="전체 재계획", zorder=3)
        ax.bar([x + w / 2 for x in xs], ours_med, w - 0.03, color=OURS, label="우리 방법", zorder=3)
        for x, f, o in zip(xs, full_med, ours_med):
            ax.text(x - w / 2, f, f"{f:.0f}", ha="center", va="bottom", fontsize=9, color=TEXT)
            ax.text(x + w / 2, o, f"{o:.0f}", ha="center", va="bottom", fontsize=9, color=TEXT)
            ax.text(x, max(full_med) * 1.12, f"{f / o:.0f}배", ha="center", fontsize=10, color=MUTED)
        ax.set_xticks(list(xs))
        ax.set_xticklabels([f"{n}대" for n in counts], color=MUTED)
        ax.set_ylim(0, max(full_med) * 1.25)
        ax.set_title(f"{name} 지도", loc="left", color=TEXT)
        ax.set_ylabel("계산 시간 중앙값 (ms)", color=MUTED)
        style(ax)
    handles, labels = axes[0][0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", ncol=2, frameon=False, bbox_to_anchor=(0.5, 0.92))
    fig.suptitle("그림 A. 재계획 한 번에 걸리는 시간 (중앙값, 한 번에 하나씩 잼)", x=0.01, ha="left",
                 fontsize=13, color=TEXT)
    fig.tight_layout(rect=(0, 0, 1, 0.84))
    fig.savefig(os.path.join(OUT, "v4_1_compute_time.png"), dpi=150)


def fig_latency_loss():
    sets = datasets()
    panels = []
    for name, ours, orig in sets:
        counts = sorted({r["num_agents"] for r in ours}, key=int)
        for n in counts[-3:]:
            panels.append((name, n, ours, orig))
    cols = 3
    rows_n = math.ceil(len(panels) / cols)
    fig, axes = plt.subplots(rows_n, cols, figsize=(14, 4.2 * rows_n), squeeze=False)
    for ax, (name, n, ours, orig) in zip(axes.flat, panels):
        full = [loss_per_robot(ours, n, d, "full_replan_ms", "full_replan_soc") for d in DELTAS]
        mine = [loss_per_robot(ours, n, d, "selective_replan_ms", "selective_replan_soc") for d in DELTAS]
        ax.plot(DELTAS, full, color=FULL, lw=2, marker="o", ms=8, label="전체 재계획", zorder=3)
        if orig:
            old = [loss_per_robot(orig, n, d, "selective_replan_ms", "selective_replan_soc") for d in DELTAS]
            ax.plot(DELTAS, old, color=ORIG, lw=2, marker="o", ms=8, label="원래 방식", zorder=3)
        ax.plot(DELTAS, mine, color=OURS, lw=2, marker="o", ms=8, label="우리 방법", zorder=3)
        ax.set_xscale("log", base=2)
        ax.set_xticks(DELTAS)
        ax.set_xticklabels(["0.25초\n(빠름)", "0.5초", "1초\n(창고 로봇)", "2초\n(TurtleBot3)"], color=MUTED)
        ax.set_ylim(bottom=0)
        ax.set_title(f"{name} · {n}대", loc="left", color=TEXT)
        style(ax)
    for ax in axes[:, 0]:
        ax.set_ylabel("로봇당 늦어진 스텝 (작을수록 좋음)", color=MUTED)
    for ax in list(axes.flat)[len(panels):]:
        ax.axis("off")
    handles, labels = axes[0][0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", ncol=3, frameon=False, bbox_to_anchor=(0.5, 0.95))
    fig.suptitle("그림 B. 계산하는 동안 로봇이 멈춘다고 하면, 로봇 한 대가 얼마나 늦어지나  (가로축: 로봇이 한 칸 가는 데 걸리는 시간)",
                 x=0.01, ha="left", fontsize=13, color=TEXT)
    fig.tight_layout(rect=(0, 0, 1, 0.90))
    fig.savefig(os.path.join(OUT, "v4_2_latency_loss.png"), dpi=150)


RESULT = ["full_replan_ok", "selective_replan_ok", "escalation_tier", "full_replan_soc", "selective_replan_soc",
          "selective_replanned", "full_replan_delayed", "selective_delayed", "full_replan_away",
          "selective_away", "selective_rescued"]


def changed(base, other, prefix_full):
    out = 0
    for a, b in zip(base, other):
        cols = [c for c in RESULT if c.startswith("full") == prefix_full]
        if any(a[c] != b[c] for c in cols):
            out += 1
    return out


def fig_standing():
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.6), sharey=True)
    for ax, (name, size, base) in zip(axes, (("32×32", "small", "idea3_path_t1"), ("64×64", "large", "idea3_large_path_t1"))):
        p0, p1 = load("v3_service", base + ".csv"), load("v3_service", f"{size}_s30_60.csv")
        s0, s1 = load("v4_standing", f"{size}_s0.csv"), load("v4_standing", f"{size}_s30_60.csv")
        groups = [("기존 장면\n(장애물이 움직이는 로봇 길 중간)", p0, p1), ("standing 장면\n(우회로에 서 있는 로봇)", s0, s1)]
        w = 0.36
        for i, (label, a, b) in enumerate(groups):
            f, o = changed(a, b, True), changed(a, b, False)
            ax.bar(i - w / 2, f, w - 0.03, color=FULL, label="전체 재계획" if i == 0 else None, zorder=3)
            ax.bar(i + w / 2, o, w - 0.03, color=OURS, label="우리 방법" if i == 0 else None, zorder=3)
            ax.text(i - w / 2, f + 2, str(f), ha="center", fontsize=10, color=TEXT)
            ax.text(i + w / 2, o + 2, str(o), ha="center", fontsize=10, color=TEXT)
        ax.set_xticks([0, 1])
        ax.set_xticklabels([g[0] for g in groups], color=MUTED)
        ax.set_title(f"{name} 지도 (시나리오 400개)", loc="left", color=TEXT)
        style(ax)
    axes[0].set_ylabel("결과가 바뀐 시나리오 수", color=MUTED)
    axes[0].legend(frameon=False, loc="upper left")
    fig.suptitle("그림 C. 작업 시간(30~60스텝)을 켰을 때 결과가 바뀐 시나리오 수  — 같은 시나리오, 작업 시간 0과 비교",
                 x=0.01, ha="left", fontsize=13, color=TEXT)
    fig.tight_layout(rect=(0, 0, 1, 0.92))
    fig.savefig(os.path.join(OUT, "v4_3_standing.png"), dpi=150)


if __name__ == "__main__":
    fig_compute_time()
    fig_latency_loss()
    fig_standing()
    print("saved v4_1_compute_time.png, v4_2_latency_loss.png, v4_3_standing.png")

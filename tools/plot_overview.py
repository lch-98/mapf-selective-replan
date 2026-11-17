"""15장 그림 1~3: 지금까지 모은 벤치마크 데이터를 한눈에 본다.

사용(리포 루트에서):  python tools/plot_overview.py
읽는 파일: tools/results/order_*priority_t1.csv(원래 방식 + 전체 재계획),
          tools/results/idea3_*base_t1.csv(고정 먼저), tools/results/idea3_*path_t1.csv(+아이디어 3)
만드는 파일: tools/plots_study/overview_*.png
"""
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.ticker import FuncFormatter
import numpy as np
import pandas as pd

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RESULTS = os.path.join(ROOT, "tools", "results")
OUT = os.path.join(ROOT, "tools", "plots_study")

# 색: 검증된 범주형 팔레트 1~4번 칸을 고정 순서로 쓴다(validate_palette.js 통과 — 인접 쌍 기준).
# 3·4번 칸(청록·노랑)은 밝은 배경 대비가 3:1 미만이라, 범례 + 끝값 라벨 + 문서의 표로 보완한다.
INK, INK2, MUTED, GRID, AXIS, SURFACE = "#0b0b0b", "#52514e", "#898781", "#e1e0d9", "#c3c2b7", "#fcfcfb"
METHODS = ["full", "priority", "fixed_first", "ff_idea3"]
COLOR = {"full": "#2a78d6", "priority": "#eb6834", "fixed_first": "#1baf7a", "ff_idea3": "#eda100"}
LABEL = {
    "full": "전체 재계획(비교 기준)",
    "priority": "원래 방식",
    "fixed_first": "고정 로봇 먼저",
    "ff_idea3": "고정 로봇 먼저 + 아이디어 3",
}
MAP_LABEL = {"open": "넓은 맵(open)", "corridor": "좁은 통로 맵(corridor)"}
SCALES = [("", "32×32"), ("large_", "64×64")]


def style():
    plt.rcParams.update({
        "font.family": "Malgun Gothic",
        "axes.unicode_minus": False,
        "figure.facecolor": SURFACE,
        "axes.facecolor": SURFACE,
        "axes.edgecolor": AXIS,
        "axes.labelcolor": INK2,
        "axes.titlecolor": INK,
        "xtick.color": MUTED,
        "ytick.color": MUTED,
        "axes.grid": True,
        "axes.axisbelow": True,
        "grid.color": GRID,
        "grid.linewidth": 0.8,
        "axes.spines.top": False,
        "axes.spines.right": False,
        "font.size": 10,
        "axes.titlesize": 11,
        "legend.frameon": False,
    })


def load(prefix):
    """방법마다 같은 열 이름으로 맞춘 긴 표를 만든다(한 줄 = 한 시나리오 × 한 방법)."""
    pri = pd.read_csv(os.path.join(RESULTS, f"order_{prefix}priority_t1.csv"))
    ff = pd.read_csv(os.path.join(RESULTS, f"idea3_{prefix}base_t1.csv"))
    i3 = pd.read_csv(os.path.join(RESULTS, f"idea3_{prefix}path_t1.csv"))
    key = ["map", "num_agents", "trial"]
    rows = []
    full = pri[key].copy()
    full["method"] = "full"
    full["solved"] = pri.full_replan_ok == 1  # 전체 재계획은 그 자체가 끝이라 성공 = 해결
    full["ms"] = pri.full_replan_ms
    full["changed"] = pri.full_replan_changed.where(pri.full_replan_ok == 1)
    full["extra_cost"] = (pri.full_replan_soc - pri.initial_soc).where(pri.full_replan_ok == 1)
    rows.append(full)
    for name, d in [("priority", pri), ("fixed_first", ff), ("ff_idea3", i3)]:
        ok = d.selective_replan_ok == 1
        m = d[key].copy()
        m["method"] = name
        # 선택적 재계획은 "안전망(=전체 재계획) 없이" 푼 것만 자기 힘으로 푼 것으로 센다.
        m["solved"] = ok & (d.escalation_tier != -1)
        m["ms"] = d.selective_replan_ms
        m["changed"] = d.selective_replan_changed.where(ok)
        m["extra_cost"] = (d.selective_replan_soc - d.initial_soc).where(ok)
        rows.append(m)
    return pd.concat(rows, ignore_index=True)


def fig_success():
    fig, axes = plt.subplots(2, 2, figsize=(11, 7), sharey=True)
    for r, (prefix, scale_label) in enumerate(SCALES):
        df = load(prefix)
        for c, map_name in enumerate(["open", "corridor"]):
            ax = axes[r][c]
            sub = df[df["map"] == map_name]  # df.map은 판다스 메서드라 열 이름으로 접근
            counts = sorted(sub.num_agents.unique())
            x = np.arange(len(counts))
            width = 0.19
            for i, m in enumerate(METHODS):
                rate = [100 * sub[(sub.num_agents == n) & (sub.method == m)].solved.mean() for n in counts]
                ax.bar(x + (i - 1.5) * (width + 0.01), rate, width, color=COLOR[m], label=LABEL[m],
                       edgecolor=SURFACE, linewidth=1.5)
            ax.set_xticks(x, [f"{n}대" for n in counts])
            ax.set_ylim(0, 105)
            ax.set_title(f"{scale_label} · {MAP_LABEL[map_name]}", loc="left")
            if c == 0:
                ax.set_ylabel("해결한 비율 (%)")
    handles, labels = axes[0][0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", ncol=4, bbox_to_anchor=(0.5, 1.0))
    fig.suptitle("그림 1. 안전망 없이 자기 힘으로 푼 시나리오 비율 (시나리오 50개씩)", y=1.05, x=0.02,
                 ha="left", color=INK, fontsize=13)
    fig.text(0.02, -0.02, "선택적 재계획 3종은 안전망(전체 재계획)으로 넘어가지 않고 푼 것만 셈. "
             "전체 재계획은 자기 성공률.", color=INK2, fontsize=9)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "overview_1_success.png"), dpi=150, bbox_inches="tight")
    plt.close(fig)


def fig_changed():
    fig, axes = plt.subplots(2, 2, figsize=(11, 7))
    for r, (prefix, scale_label) in enumerate(SCALES):
        df = load(prefix)
        for c, map_name in enumerate(["open", "corridor"]):
            ax = axes[r][c]
            sub = df[df["map"] == map_name]  # df.map은 판다스 메서드라 열 이름으로 접근
            counts = sorted(sub.num_agents.unique())
            ends = {}
            for m in METHODS:
                means = [sub[(sub.num_agents == n) & (sub.method == m)].changed.mean() for n in counts]
                ax.plot(range(len(counts)), means, color=COLOR[m], linewidth=2, marker="o", markersize=7,
                        markeredgecolor=SURFACE, markeredgewidth=2, label=LABEL[m])
                ends[m] = means[-1]
            # 끝값 라벨: 두 값이 거의 같으면(로그 축에서 겹침) 위아래로 벌린다.
            close = abs(np.log(ends["fixed_first"] / ends["ff_idea3"])) < 0.15
            for m, v in ends.items():
                dy = (6 if m == "fixed_first" else -6) if close and m in ("fixed_first", "ff_idea3") else 0
                ax.annotate(f"{v:.1f}", (len(counts) - 1, v), xytext=(8, dy), textcoords="offset points",
                            va="center", color=INK2, fontsize=9)
            ax.set_yscale("log")
            ax.yaxis.set_major_formatter(FuncFormatter(lambda val, _: f"{val:g}"))
            ax.set_xticks(range(len(counts)), [f"{n}대" for n in counts])
            ax.set_xlim(-0.3, len(counts) - 0.4)
            ax.set_title(f"{scale_label} · {MAP_LABEL[map_name]}", loc="left")
            if c == 0:
                ax.set_ylabel("경로가 바뀐 로봇 수 평균 (로그 축)")
    handles, labels = axes[0][0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", ncol=4, bbox_to_anchor=(0.5, 1.0))
    fig.suptitle("그림 2. 장애물 하나 때문에 경로가 바뀐 로봇 수 — 적을수록 '덜 건드림'", y=1.05, x=0.02,
                 ha="left", color=INK, fontsize=13)
    fig.text(0.02, -0.02, "각 방법이 성공한 시나리오의 평균(안전망 포함). 오른쪽 끝 숫자는 가장 많은 로봇 수에서의 값.",
             color=INK2, fontsize=9)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "overview_2_changed.png"), dpi=150, bbox_inches="tight")
    plt.close(fig)


def fig_runtime():
    df = load("large_")
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.2), sharey=True)
    for c, map_name in enumerate(["open", "corridor"]):
        ax = axes[c]
        sub = df[df["map"] == map_name]  # df.map은 판다스 메서드라 열 이름으로 접근
        for m in METHODS:
            ms = np.sort(sub[sub.method == m].ms.values)
            y = 100 * np.arange(1, len(ms) + 1) / len(ms)
            ax.plot(ms, y, color=COLOR[m], linewidth=2, label=LABEL[m], drawstyle="steps-post")
        ax.axvline(1000, color=AXIS, linewidth=1)
        ax.text(1000, 3, " 1초", color=INK2, fontsize=9)
        ax.set_xscale("log")
        ax.xaxis.set_major_formatter(FuncFormatter(lambda val, _: f"{val:g}"))
        ax.set_xlabel("재계획 1회에 걸린 시간 (ms, 로그 축)")
        ax.set_title(f"64×64 · {MAP_LABEL[map_name]}", loc="left")
        if c == 0:
            ax.set_ylabel("이 시간 안에 끝난 비율 (%)")
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", ncol=4, bbox_to_anchor=(0.5, 1.04))
    fig.suptitle("그림 3. 재계획 시간 분포 — 왼쪽 위로 붙을수록 빠름", y=1.12, x=0.02, ha="left",
                 color=INK, fontsize=13)
    fig.text(0.02, -0.06, "성공·실패 모든 시나리오(로봇 50~200대, 맵별 200개). 선이 오른쪽 끝에서 늦게 100%에 닿을수록 "
             "'가끔 아주 느린' 경우가 있다는 뜻.", color=INK2, fontsize=9)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "overview_3_runtime.png"), dpi=150, bbox_inches="tight")
    plt.close(fig)


def print_table():
    for prefix, scale_label in SCALES:
        df = load(prefix)
        print(f"== {scale_label} ==")
        g = df.groupby("method").agg(
            solved=("solved", "sum"), n=("solved", "size"), changed=("changed", "mean"),
            extra=("extra_cost", "mean"), ms_median=("ms", "median"),
            ms_p95=("ms", lambda s: s.quantile(0.95)), ms_max=("ms", "max"))
        print(g.loc[METHODS].round(2).to_string())


if __name__ == "__main__":
    os.makedirs(OUT, exist_ok=True)
    style()
    fig_success()
    fig_changed()
    fig_runtime()
    print_table()

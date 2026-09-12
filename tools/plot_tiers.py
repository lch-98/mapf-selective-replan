# ─────────────────────────────────────────────────────────────────
# tools/plot_tiers.py
#
# 추적 단계 수(max_escalation_tiers, run_benchmark --tiers K)를 바꿔 돌린
# 벤치마크 CSV 여러 개를 한 그림에 겹쳐서 비교한다. 같은 시나리오(시드)에서
# K만 바꾼 결과이므로 곡선 간 차이는 K의 효과다.
#
# 사용법:
#   python tools/plot_tiers.py <output_dir> 1=path/to/tiers1.csv 2=path/to/tiers2.csv ...
#
# 만드는 그림 3장(맵별로 나란히):
#   tiers_resolved.png : 안전망(전체 재계획) 없이 Tier 0~K 안에서 해결된 비율
#   tiers_runtime.png  : 재계획 1회 시간의 중앙값(성공/실패 무관, 로그 축)
#   tiers_speedup.png  : 같은 시나리오끼리 짝지은 속도 향상(전체 ÷ 선택) 중앙값
# ─────────────────────────────────────────────────────────────────
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import matplotlib.ticker as mticker  # noqa: E402
import pandas as pd  # noqa: E402

# K는 순서가 있는 값이라 한 가지 색(파랑)의 옅은→짙은 단계로 칠한다.
# (dataviz 기본 팔레트의 blue ramp, ordinal 규칙상 step 300부터 사용)
TIER_COLORS = ["#6da7ec", "#3987e5", "#256abf", "#104281"]
FULL_COLOR = "#52514e"   # 기준선(전체 재계획) — 시리즈와 겹치지 않는 회색
INK = "#0b0b0b"
MUTED = "#898781"
GRID = "#e1e0d9"
SURFACE = "#fcfcfb"


def load(specs):
    frames = []
    for spec in specs:
        k, path = spec.split("=", 1)
        df = pd.read_csv(path)
        df = df[df["plan_ok"] == 1].copy()
        df["tiers"] = int(k)
        frames.append(df)
    return pd.concat(frames, ignore_index=True)


def style_axis(ax):
    ax.set_facecolor(SURFACE)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color("#c3c2b7")
    ax.tick_params(colors=MUTED, labelcolor="#52514e")
    ax.grid(axis="y", color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)


def small_multiples(df, title, ylabel, compute, out_path, full_compute=None, log=False, ref_line=None):
    maps = sorted(df["map"].unique())
    tiers = sorted(df["tiers"].unique())
    fig, axes = plt.subplots(1, len(maps), figsize=(6.2 * len(maps), 4.4), sharey=True)
    fig.patch.set_facecolor(SURFACE)
    if len(maps) == 1:
        axes = [axes]
    for ax, map_name in zip(axes, maps):
        style_axis(ax)
        sub = df[df["map"] == map_name]
        agents = sorted(sub["num_agents"].unique())
        if full_compute is not None:
            # 전체 재계획은 K와 무관하므로 모든 실행을 합쳐 한 선으로 그린다.
            ys = [full_compute(sub[sub["num_agents"] == n]) for n in agents]
            ax.plot(agents, ys, color=FULL_COLOR, linestyle="--", linewidth=2,
                    marker="s", markersize=6, label="full_replan")
        for color, k in zip(TIER_COLORS, tiers):
            s = sub[sub["tiers"] == k]
            ys = [compute(s[s["num_agents"] == n]) for n in agents]
            ax.plot(agents, ys, color=color, linewidth=2, marker="o", markersize=6,
                    label=f"selective, tiers={k}")
        if ref_line is not None:
            ax.axhline(ref_line[0], color=MUTED, linewidth=1, linestyle=":")
            # 곡선이 기준선 쪽으로 내려오는 오른쪽 끝과 겹치지 않게 왼쪽 끝에 둔다.
            ax.text(agents[0], ref_line[0], f" {ref_line[1]}", color=MUTED,
                    va="bottom", ha="left", fontsize=9)
        ax.set_xticks(agents)
        ax.set_xlabel("num_agents", color="#52514e")
        ax.set_title(f"map = {map_name}", color=INK)
        if log:
            ax.set_yscale("log")
            ax.yaxis.set_major_formatter(mticker.FuncFormatter(lambda v, _: f"{v:g}"))
            ax.yaxis.set_minor_formatter(mticker.NullFormatter())
    axes[0].set_ylabel(ylabel, color="#52514e")
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="lower center", ncol=len(labels), frameon=False,
               bbox_to_anchor=(0.5, -0.02), labelcolor="#52514e")
    fig.suptitle(title, color=INK)
    fig.tight_layout(rect=(0, 0.07, 1, 1))
    fig.savefig(out_path, dpi=150, facecolor=SURFACE)
    plt.close(fig)


def main():
    if len(sys.argv) < 3:
        print(__doc__ or "usage: python tools/plot_tiers.py <output_dir> K=csv ...")
        print("usage: python tools/plot_tiers.py <output_dir> 1=tiers1.csv 2=tiers2.csv ...")
        sys.exit(1)
    out_dir = Path(sys.argv[1])
    out_dir.mkdir(parents=True, exist_ok=True)
    df = load(sys.argv[2:])

    def resolved(g):  # 안전망 없이 Tier 0~K에서 해결된 비율(%)
        return 100.0 * ((g["selective_replan_ok"] == 1) & (g["escalation_tier"] != -1)).mean()

    def full_success(g):
        return 100.0 * (g["full_replan_ok"] == 1).mean()

    small_multiples(
        df, "Resolved without the safety-net (full replan), by escalation depth",
        "resolved within tier 0..K (%)", resolved, out_dir / "tiers_resolved.png",
        full_compute=full_success)

    small_multiples(
        df, "Median replan time per event (all trials, log scale)",
        "median runtime (ms)", lambda g: g["selective_replan_ms"].median(),
        out_dir / "tiers_runtime.png",
        full_compute=lambda g: g["full_replan_ms"].median(), log=True)

    def speedup(g):  # 같은 시나리오끼리 짝지은 full/selective 비율의 중앙값
        return (g["full_replan_ms"] / g["selective_replan_ms"]).median()

    small_multiples(
        df, "Paired speedup of selective over full replan (median of full/selective)",
        "speedup (x, >1 = selective faster)", speedup, out_dir / "tiers_speedup.png",
        ref_line=(1.0, "same speed"))

    print(f"[ok] saved 3 plots to {out_dir}")


if __name__ == "__main__":
    main()

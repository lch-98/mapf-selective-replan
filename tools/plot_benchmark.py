#!/usr/bin/env python3
# ─────────────────────────────────────────────────────────────────
# tools/plot_benchmark.py
#
# run_benchmark.exe가 만든 CSV(benchmark.cpp 출력)를 읽어서 4종의 그래프를
# PNG로 저장한다:
#   1. success_rate.png   — 맵 x 로봇수별 full_replan/selective_replan 성공률 막대그래프
#   2. runtime_box.png    — 로봇수 증가에 따른 실행시간(ms) 분포 박스플롯
#   3. escalation_tier.png— 선택적 재계획이 Tier 0/1+/안전망 중 어디서 끝났는지 비율
#   4. plan_attempts.png  — 로봇수가 늘수록 초기 plan() 재시도 횟수가 어떻게 변하는지
#
# 사용법: python plot_benchmark.py path/to/out.csv [output_dir]
# ─────────────────────────────────────────────────────────────────
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import matplotlib.ticker as mticker
import pandas as pd

MAP_COLORS = {"open": "#4C72B0", "corridor": "#DD8452"}
METHOD_COLORS = {"full_replan": "#4C72B0", "selective_replan": "#DD8452"}


def load_csv(csv_path: str) -> pd.DataFrame:
    df = pd.read_csv(csv_path)
    return df


def plot_success_rate(df: pd.DataFrame, out_dir: Path) -> None:
    agent_counts = sorted(df["num_agents"].unique())
    maps = sorted(df["map"].unique())

    fig, axes = plt.subplots(1, len(maps), figsize=(6 * len(maps), 4.5), sharey=True)
    if len(maps) == 1:
        axes = [axes]

    bar_width = 0.25
    tier01_color = "#55A868"  # escalation_tier 그래프의 "tier 0" 초록과 계열을 맞춤
    for ax, map_name in zip(axes, maps):
        sub = df[df["map"] == map_name]
        # plan_ok인 행만 대상으로 삼는다 — 초기 계획 자체가 실패한 trial은
        # 방법 A/B 비교와 무관하므로 분모에서 뺀다.
        sub = sub[sub["plan_ok"] == 1]

        # selective_replan_ok=1에는 Tier 0/1(진짜 "선택적" 재계획)뿐 아니라
        # escalation_tier=-1(안전망 = full_replan을 그대로 호출해서 성공)도
        # 섞여 있다. 안전망은 full_replan과 완전히 같은 로직이므로, 이론적으로
        # "selective_replan 전체(안전망 포함) >= full_replan"은 항상 성립한다
        # (안전망이 실패하면 그 trial은 애초에 full_replan도 실패했을 것이므로).
        # 반면 Tier 0/1만 따로 떼어보면 "안전망까지 가야만 풀린 경우"는 빠지므로
        # full_replan보다 낮게 나올 수 있다 — 이건 모순이 아니라 "Tier 0/1
        # 단독으로는 부족했던 사례가 있었다"는 것을 정확히 보여주는 신호다.
        # 세 막대를 나란히 비교해서 이 관계를 한눈에 보여준다.
        tier01_success = (sub["selective_replan_ok"] == 1) & (sub["escalation_tier"] != -1)
        all_success = sub["selective_replan_ok"] == 1

        full_rates = []
        tier01_rates = []
        all_rates = []
        for n in agent_counts:
            group = sub[sub["num_agents"] == n]
            total = len(group)
            full_rates.append(100.0 * group["full_replan_ok"].sum() / total if total else 0.0)
            tier01_rates.append(100.0 * tier01_success[group.index].sum() / total if total else 0.0)
            all_rates.append(100.0 * all_success[group.index].sum() / total if total else 0.0)

        x = range(len(agent_counts))
        ax.bar([i - bar_width for i in x], full_rates, width=bar_width,
               label="full_replan", color=METHOD_COLORS["full_replan"])
        ax.bar(list(x), tier01_rates, width=bar_width,
               label="selective_replan (tier 0/1 only)", color=tier01_color)
        ax.bar([i + bar_width for i in x], all_rates, width=bar_width,
               label="selective_replan (all, incl. safety-net)", color=METHOD_COLORS["selective_replan"])

        ax.set_xticks(list(x))
        ax.set_xticklabels([str(n) for n in agent_counts])
        ax.set_xlabel("num_agents")
        ax.set_title(f"map = {map_name}")
        ax.set_ylim(0, 105)
        ax.grid(axis="y", linestyle="--", alpha=0.4)

    axes[0].set_ylabel("success rate (%)")
    handles, labels = axes[-1].get_legend_handles_labels()
    fig.legend(handles, labels, loc="lower center", ncol=3, bbox_to_anchor=(0.5, -0.05))
    fig.suptitle("Replan success rate: full vs. selective (tier 0/1 only) vs. selective (incl. safety-net)")
    fig.tight_layout()
    fig.savefig(out_dir / "success_rate.png", dpi=150, bbox_inches="tight")
    plt.close(fig)


def plot_runtime_box(df: pd.DataFrame, out_dir: Path) -> None:
    agent_counts = sorted(df["num_agents"].unique())
    maps = sorted(df["map"].unique())

    fig, axes = plt.subplots(1, len(maps), figsize=(6 * len(maps), 4.5), sharey=True)
    if len(maps) == 1:
        axes = [axes]

    for ax, map_name in zip(axes, maps):
        sub = df[(df["map"] == map_name) & (df["plan_ok"] == 1)]

        # 각 로봇 수마다, 성공한 시행의 실행시간만 모은다(실패한 시행은
        # *_ms가 "실패까지 걸린 시간"이라 성공 시행과 의미가 달라 섞으면 안 됨).
        full_data = [sub[(sub["num_agents"] == n) & (sub["full_replan_ok"] == 1)]["full_replan_ms"]
                     for n in agent_counts]
        sel_data = [sub[(sub["num_agents"] == n) & (sub["selective_replan_ok"] == 1)]["selective_replan_ms"]
                    for n in agent_counts]

        positions_full = [i * 3 + 1 for i in range(len(agent_counts))]
        positions_sel = [i * 3 + 2 for i in range(len(agent_counts))]

        bp_full = ax.boxplot(full_data, positions=positions_full, widths=0.7, patch_artist=True,
                              showfliers=False)
        bp_sel = ax.boxplot(sel_data, positions=positions_sel, widths=0.7, patch_artist=True,
                             showfliers=False)

        for box in bp_full["boxes"]:
            box.set_facecolor(METHOD_COLORS["full_replan"])
            box.set_alpha(0.7)
        for box in bp_sel["boxes"]:
            box.set_facecolor(METHOD_COLORS["selective_replan"])
            box.set_alpha(0.7)

        # matplotlib의 median 선 기본색(C1, 주황)은 박스 면 색(set_facecolor)과
        # 무관하게 항상 그대로 남는다 — n=1처럼 박스가 사실상 선 하나로
        # 찌그러지면, full_replan(파란 계열) 자리인데도 기본 주황 median 선만
        # 남아 selective_replan 색처럼 보이는 착시가 생긴다. 두 그룹 모두
        # median 선을 검정으로 고정해서 박스 면 색과 항상 구분되게 한다.
        for median in bp_full["medians"] + bp_sel["medians"]:
            median.set_color("black")

        # 표본 수(n)가 극히 적은 구간(예: corridor+40대는 plan() 자체가 거의
        # 실패해서 성공 표본이 한 자릿수로 떨어짐)을 x축 라벨 아래에 명시한다
        # — 안 그러면 표본 1~2개짜리 "선"과 표본 30개짜리 박스를 같은 무게로
        # 오인하기 쉽다. 그래프 안(y값 기준)이 아니라 axes 좌표(0~1)의 고정된
        # 위치에 둬서, 박스 높이와 무관하게 항상 같은 자리에 나오게 한다.
        for pos, data in zip(positions_full, full_data):
            ax.text(pos, -0.09, f"n={len(data)}", ha="center", va="top",
                    fontsize=7, color="dimgray", transform=ax.get_xaxis_transform())
        for pos, data in zip(positions_sel, sel_data):
            ax.text(pos, -0.09, f"n={len(data)}", ha="center", va="top",
                    fontsize=7, color="dimgray", transform=ax.get_xaxis_transform())

        tick_positions = [i * 3 + 1.5 for i in range(len(agent_counts))]
        ax.set_xticks(tick_positions)
        ax.set_xticklabels([str(n) for n in agent_counts])
        ax.set_xlabel("num_agents", labelpad=14)
        ax.set_title(f"map = {map_name}")
        ax.set_yscale("log")
        # 로그 스케일이라도 눈금은 10^0, 10^1 같은 지수 표기 대신 0.5, 1, 2, 5,
        # 10, 20처럼 실제 ms 값을 그대로 보여준다 — 로그 축의 "넓은 범위를 한
        # 화면에" 장점은 유지하면서, 숫자를 지수 변환 없이 바로 읽게 한다.
        # 눈금 위치를 고정 목록으로 직접 지정해서(자동 minor tick이 만드는
        # 0.6, 0.7, 0.8 같은 지저분한 촘촘한 라벨을 피한다).
        tick_values = [0.2, 0.5, 1, 2, 5, 10, 20, 50]
        ax.set_yticks(tick_values)
        ax.yaxis.set_major_formatter(mticker.ScalarFormatter())
        ax.yaxis.set_minor_formatter(mticker.NullFormatter())
        ax.grid(axis="y", which="major", linestyle="--", alpha=0.4)
    
    axes[0].set_ylabel("runtime (ms, log scale)")

    # 범례는 그림 전체에 한 번만 표시.
    from matplotlib.patches import Patch
    legend_handles = [
        Patch(facecolor=METHOD_COLORS["full_replan"], alpha=0.7, label="full_replan"),
        Patch(facecolor=METHOD_COLORS["selective_replan"], alpha=0.7, label="selective_replan"),
    ]
    axes[0].legend(handles=legend_handles, loc="upper left")
    axes[1].legend(handles=legend_handles, loc="upper left")
    
    fig.suptitle("Runtime distribution (successful trials only, log scale)")
    fig.tight_layout()
    fig.savefig(out_dir / "runtime_box.png", dpi=150)
    plt.close(fig)


def plot_escalation_tier(df: pd.DataFrame, out_dir: Path) -> None:
    agent_counts = sorted(df["num_agents"].unique())
    maps = sorted(df["map"].unique())

    # escalation_tier 값: 0=Tier0, 1+=에스컬레이션 성공, -1=안전망(full_replan) 폴백.
    # selective_replan_ok=1인 행만 의미가 있다(실패한 행의 tier는 항상 0으로 채워져 있어 무의미).
    def tier_label(t: int) -> str:
        if t == -1:
            return "safety-net"
        if t == 0:
            return "tier 0"
        return f"tier {t}+"

    fig, axes = plt.subplots(1, len(maps), figsize=(6 * len(maps), 4.5), sharey=True)
    if len(maps) == 1:
        axes = [axes]

    for ax, map_name in zip(axes, maps):
        sub = df[(df["map"] == map_name) & (df["plan_ok"] == 1) & (df["selective_replan_ok"] == 1)]

        tier_categories = ["tier 0", "tier 1+", "safety-net"]
        bottom = [0.0] * len(agent_counts)
        colors = {"tier 0": "#55A868", "tier 1+": "#DD8452", "safety-net": "#C44E52"}

        for cat in tier_categories:
            heights = []
            for n in agent_counts:
                group = sub[sub["num_agents"] == n]
                total = len(group)
                if total == 0:
                    heights.append(0.0)
                    continue
                if cat == "tier 0":
                    count = (group["escalation_tier"] == 0).sum()
                elif cat == "tier 1+":
                    count = (group["escalation_tier"] > 0).sum()
                else:
                    count = (group["escalation_tier"] == -1).sum()
                heights.append(100.0 * count / total)

            x = range(len(agent_counts))
            ax.bar(list(x), heights, bottom=bottom, label=cat, color=colors[cat])
            bottom = [b + h for b, h in zip(bottom, heights)]

        ax.set_xticks(list(range(len(agent_counts))))
        ax.set_xticklabels([str(n) for n in agent_counts])
        ax.set_xlabel("num_agents")
        ax.set_title(f"map = {map_name}")
        ax.set_ylim(0, 105)
        ax.grid(axis="y", linestyle="--", alpha=0.4)

    axes[0].set_ylabel("share of successful selective_replan runs (%)")
    axes[0].legend(loc="lower left")
    axes[1].legend(loc="lower left")
    fig.suptitle("Where selective_replan succeeded: tier breakdown")
    fig.tight_layout()
    fig.savefig(out_dir / "escalation_tier.png", dpi=150)
    plt.close(fig)


def plot_plan_attempts(df: pd.DataFrame, out_dir: Path) -> None:
    agent_counts = sorted(df["num_agents"].unique())
    maps = sorted(df["map"].unique())

    fig, ax = plt.subplots(figsize=(7, 4.5))

    for map_name in maps:
        sub = df[df["map"] == map_name]
        means = [sub[sub["num_agents"] == n]["plan_attempts"].mean() for n in agent_counts]
        ax.plot(agent_counts, means, marker="o", label=map_name,
                color=MAP_COLORS.get(map_name))

    ax.set_xlabel("num_agents")
    ax.set_ylabel("mean plan_attempts (initial plan() retries)")
    ax.set_title("Initial design difficulty vs. robot count")
    ax.set_xticks(agent_counts)
    ax.grid(axis="y", linestyle="--", alpha=0.4)
    ax.legend()

    fig.tight_layout()
    fig.savefig(out_dir / "plan_attempts.png", dpi=150)
    plt.close(fig)


def main() -> None:
    if len(sys.argv) < 2:
        print("usage: python plot_benchmark.py path/to/out.csv [output_dir]", file=sys.stderr)
        sys.exit(1)

    csv_path = sys.argv[1]
    out_dir = Path(sys.argv[2]) if len(sys.argv) > 2 else Path(__file__).parent / "plots"
    out_dir.mkdir(parents=True, exist_ok=True)

    df = load_csv(csv_path)

    plot_success_rate(df, out_dir)
    plot_runtime_box(df, out_dir)
    plot_escalation_tier(df, out_dir)
    plot_plan_attempts(df, out_dir)

    print(f"[ok] saved 4 plots to {out_dir}")


if __name__ == "__main__":
    main()

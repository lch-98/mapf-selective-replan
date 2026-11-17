"""15장 그림 4~9: 남은 문제 3가지가 얼마나, 어떤 모습으로 나오는가.

사용(리포 루트에서):  python tools/plot_problems.py
읽는 파일: tools/results/problems_default.csv, tools/results/problems_large.csv
          (시나리오마다 원래 방식 / 고정 로봇 먼저 / + 아이디어 3 세 설정의 자세한 값)
          tools/results/examples/*.json (예시 시나리오의 경로 전체)
만드는 파일: tools/plots_study/problem_*.png, example_*.png
"""
import json
import os
from collections import deque

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.ticker import FuncFormatter
from matplotlib.transforms import offset_copy
import numpy as np
import pandas as pd

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RESULTS = os.path.join(ROOT, "tools", "results")
EXAMPLES = os.path.join(RESULTS, "examples")
OUT = os.path.join(ROOT, "tools", "plots_study")

INK, INK2, MUTED, GRID, AXIS, SURFACE = "#0b0b0b", "#52514e", "#898781", "#e1e0d9", "#c3c2b7", "#fcfcfb"
BLUE, ORANGE, AQUA, YELLOW, RED = "#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e34948"
NEUTRAL = "#f0efec"  # 발산형 가운데(0)
WALL = "#d6d5ce"
VARIANT_LABEL = {"priority": "원래 방식", "fixed_first": "고정 로봇 먼저\n(구조 후보: 막혀 본 모든 칸)",
                 "ff_idea3": "고정 로봇 먼저 + 아이디어 3\n(구조 후보: 최단 경로와 부딪힘)"}
SCALES = [("default", "32×32"), ("large", "64×64")]
KEY = ["map", "num_agents", "trial"]


def style():
    plt.rcParams.update({
        "font.family": "Malgun Gothic", "axes.unicode_minus": False,
        "figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "axes.edgecolor": AXIS,
        "axes.labelcolor": INK2, "axes.titlecolor": INK, "xtick.color": MUTED, "ytick.color": MUTED,
        "axes.grid": True, "axes.axisbelow": True, "grid.color": GRID, "grid.linewidth": 0.8,
        "axes.spines.top": False, "axes.spines.right": False, "font.size": 10, "axes.titlesize": 11,
        "legend.frameon": False,
    })


def load(scale):
    return pd.read_csv(os.path.join(RESULTS, f"problems_{scale}.csv"))


def parse_affected(s):
    out = {}
    if isinstance(s, str):
        for part in s.strip(";").split(";"):
            if part:
                i, b, a = part.split(":")
                out[int(i)] = (int(b), int(a))
    return out


def title(fig, text, sub=None, y=1.02):
    """제목은 y 위에, 설명은 y 바로 아래에(점 단위로 띄워서) 놓아 그림 높이와 상관없이 안 겹치게 한다."""
    fig.text(0.02, y, text, ha="left", va="bottom", color=INK, fontsize=13)
    if sub:
        below = offset_copy(fig.transFigure, fig=fig, y=-4, units="points")
        fig.text(0.02, y, sub, ha="left", va="top", color=INK2, fontsize=9.5, transform=below)


PLAIN = FuncFormatter(lambda val, _: f"{val:g}")  # 로그 축 눈금을 10^1 대신 10, 100으로


# ── 문제 1: 길이 없을 때 포기가 늦다 ─────────────────────────────────────
def fig_problem1():
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4.6), gridspec_kw={"width_ratios": [1.35, 1]})
    for scale, label in SCALES:
        df = load(scale)
        g = df[(df.variant == "ff_idea3") & (df.astar_fails > 0)]
        pct = 100 * g.astar_fail_max_expanded / g.state_space
        color = BLUE if scale == "default" else ORANGE
        ax1.scatter(pct, g.astar_fail_ms.clip(lower=0.01), s=46, color=color, edgecolor=SURFACE, linewidth=2,
                    label=f"{label} ({len(g)}개 시나리오)", zorder=3)
        if len(g):  # 맵 크기마다 가장 느린 한 곳만 이름을 붙인다(나머지는 문서의 표)
            top = g.loc[g.astar_fail_ms.idxmax()]
            p = 100 * top.astar_fail_max_expanded / top.state_space
            ax1.annotate(f"{top['map']} {top.num_agents}대 #{top.trial}\n{top.astar_fail_ms:,.0f} ms, {p:.0f}%",
                         (p, top.astar_fail_ms), xytext=(-12, -2), textcoords="offset points", ha="right",
                         va="top", fontsize=8.5, color=INK2)
    ax1.set_yscale("log")
    ax1.yaxis.set_major_formatter(PLAIN)
    ax1.set_xlim(-3, 100)
    ax1.set_xlabel("실패한 A*가 뒤진 상태 수 ÷ 뒤질 수 있는 전체 (칸 × 남은 시간), %")
    ax1.set_ylabel("실패한 A*에 쓴 시간 (ms, 로그 축)")
    ax1.set_title("A. 실패한 A*: 많이 뒤질수록 오래 걸린다", loc="left")
    ax1.legend(loc="upper left")

    # B: 전체 재계획 시간 중 실패한 A*가 차지한 비율
    rows = []
    for scale, label in SCALES:
        df = load(scale)
        for v in ["priority", "fixed_first", "ff_idea3"]:
            g = df[df.variant == v]
            rows.append((f"{label}\n{VARIANT_LABEL[v].splitlines()[0]}", 100 * g.astar_fail_ms.sum() / g.ms.sum(),
                         g.ms.sum() / 1000))
    names = [r[0] for r in rows][::-1]
    vals = [r[1] for r in rows][::-1]
    totals = [r[2] for r in rows][::-1]
    y = np.arange(len(rows))
    ax2.barh(y, vals, height=0.6, color=BLUE, edgecolor=SURFACE, linewidth=2)
    for yi, v, tot in zip(y, vals, totals):
        ax2.text(v + 1.5, yi, f"{v:.0f}%  (전체 {tot:.1f}초 중)", va="center", fontsize=9, color=INK2)
    ax2.set_yticks(y, names, fontsize=8.5)
    ax2.set_xlim(0, 75)
    ax2.set_xlabel("모든 시나리오의 재계획 시간 중 '실패한 A*'가 쓴 비율 (%)")
    ax2.set_title("B. 시간이 어디로 가나", loc="left")
    ax2.grid(axis="y", visible=False)
    title(fig, "그림 4. 문제 1 — 길을 못 찾을 때 포기가 늦다",
          "고정 로봇 먼저 방식에서는 A*가 실패해야 구조 로봇을 부를 수 있다. 실패는 드물고(64×64 400개 중 12개) 대부분 "
          "금방 끝나지만, 몇 번은 뒤질 수 있는 상태의 약 70%를 다 뒤지며 1.5~2.5초를 쓴다.", y=1.08)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "problem_1_failed_astar.png"), dpi=150, bbox_inches="tight")
    plt.close(fig)


# ── 문제 2: 필요 없는 로봇에게도 부탁한다 ────────────────────────────────
def fig_problem2():
    fig, axes = plt.subplots(1, 2, figsize=(12, 3.8), sharey=True)
    variants = ["priority", "fixed_first", "ff_idea3"]
    for ax, (scale, label) in zip(axes, SCALES):
        df = load(scale)
        y = np.arange(len(variants))[::-1]
        for yi, v in zip(y, variants):
            g = df[df.variant == v]
            moved = int(g.rescued.sum() - g.rescued_unchanged.sum())
            idle = int(g.rescued_unchanged.sum())
            ax.barh(yi, moved, height=0.55, color=BLUE, edgecolor=SURFACE, linewidth=2,
                    label="불렀고 실제로 비켜 줌" if yi == y[0] else None)
            ax.barh(yi, idle, left=moved, height=0.55, color=ORANGE, edgecolor=SURFACE, linewidth=2,
                    label="불렀지만 안 움직임(헛부탁)" if yi == y[0] else None)
            total = moved + idle
            share = f"{100 * idle / total:.0f}%" if total else "-"
            ax.text(total + max(1, 0.01 * ax.get_xlim()[1]), yi, f"{total}대 중 헛부탁 {idle}대 ({share})",
                    va="center", fontsize=9, color=INK2)
        ax.set_yticks(y, [VARIANT_LABEL[v] for v in variants], fontsize=8.5)
        ax.set_title(f"{label} (시나리오 400개 합계)", loc="left")
        ax.set_xlabel("불러온 구조 로봇 수")
        ax.grid(axis="y", visible=False)
        ax.set_xlim(0, ax.get_xlim()[1] * 1.45)
    fig.legend(*axes[0].get_legend_handles_labels(), loc="lower center", ncol=2, bbox_to_anchor=(0.5, -0.1))
    title(fig, "그림 5. 문제 2 — 비켜 달라고 불렀는데 결국 안 움직인 로봇",
          "원래 방식의 구조 로봇은 등록이 거절된 로봇이라 항상 움직인다. 고정 로봇 먼저 방식에서는 "
          "'막혀 본 모든 칸'으로 고르면 헛부탁이 많고, 아이디어 3이 이를 크게 줄였다.", y=1.1)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "problem_2_rescue.png"), dpi=150, bbox_inches="tight")
    plt.close(fig)


# ── 문제 3: 중요한 로봇이 양보한다 ──────────────────────────────────────
BINS = [("일찍 도착", lambda d: d < 0), ("같음", lambda d: d == 0), ("1스텝", lambda d: d == 1),
        ("2스텝", lambda d: d == 2), ("3~4스텝", lambda d: 3 <= d <= 4), ("5~9스텝", lambda d: 5 <= d <= 9),
        ("10스텝 이상", lambda d: d >= 10)]


def fig_problem3():
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.2), sharey=True)
    for ax, (scale, label) in zip(axes, SCALES):
        df = load(scale)
        p = df[df.variant == "priority"].set_index(KEY)
        f = df[df.variant == "fixed_first"].set_index(KEY)
        both = (p.ok == 1) & (f.ok == 1) & (p.tier != -1) & (f.tier != -1)
        diffs = []
        for k in both[both].index:
            ap, af = parse_affected(p.loc[k, "affected"]), parse_affected(f.loc[k, "affected"])
            diffs += [af[r][1] - ap[r][1] for r in ap]
        counts = [sum(1 for d in diffs if test(d)) for _, test in BINS]
        colors = [BLUE, NEUTRAL] + [RED] * 5
        x = np.arange(len(BINS))
        bars = ax.bar(x, counts, width=0.7, color=colors, edgecolor=[AXIS if c == NEUTRAL else SURFACE for c in colors],
                      linewidth=1.5)
        for xi, c in zip(x, counts):
            ax.text(xi, c * 1.15 if c else 1.1, str(c), ha="center", va="bottom", fontsize=9, color=INK2)
        ax.set_yscale("log")
        ax.yaxis.set_major_formatter(PLAIN)
        ax.set_ylim(0.8, 2000)
        ax.set_xticks(x, [b[0] for b in BINS], fontsize=8.5)
        ax.set_title(f"{label} · 영향받은 로봇 {len(diffs)}대", loc="left")
        ax.set_xlabel("고정 로봇 먼저 방식에서의 도착이 원래 방식보다 얼마나 늦었나")
        if ax is axes[0]:
            ax.set_ylabel("로봇 수 (로그 축)")
    title(fig, "그림 6. 문제 3 — 장애물 때문에 다시 계획한 로봇이 더 늦게 도착한 경우",
          "두 방식이 모두 안전망 없이 성공한 시나리오만 짝지어 비교. 대부분(95%)은 같고, 늦어진 로봇은 4%, "
          "보통 1~2스텝이지만 드물게 14~66스텝.", y=1.1)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "problem_3_yield.png"), dpi=150, bbox_inches="tight")
    plt.close(fig)


# ── 예시 그림 ────────────────────────────────────────────────────────────
def pos(p, t):
    return tuple(p[min(max(t, 0), len(p) - 1)])


def future(p, ct):
    """current_time부터 도착까지의 칸 목록(연속 중복 = 기다림은 합친다)."""
    cells = [pos(p, t) for t in range(ct, len(p))]
    out = []
    for c in cells:
        if not out or out[-1] != c:
            out.append(c)
    return out or [pos(p, ct)]


def bfs(rows, start, goal, blocked):
    """다른 로봇 무시, 벽 + 새 장애물만 피하는 최단 경로(C++ static_shortest_path와 같은 이웃 순서)."""
    h, w = len(rows), len(rows[0])
    free = lambda x, y: 0 <= x < w and 0 <= y < h and rows[y][x] != "#" and (x, y) not in blocked
    parent = {start: None}
    q = deque([start])
    while q:
        c = q.popleft()
        if c == goal:
            break
        x, y = c
        for n in NEIGHBOR_ORDER(x, y):
            if n not in parent and free(*n):
                parent[n] = c
                q.append(n)
    if goal not in parent:
        return []
    route, c = [], goal
    while c is not None:
        route.append(c)
        c = parent[c]
    return route[::-1]


def NEIGHBOR_ORDER(x, y):  # map.cpp Map::neighbors와 같은 순서(대기 제외): 오른쪽, 왼쪽, 아래, 위
    return [(x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)]


def auto_crop(d, variants, roles, extra=(), pad=1):
    """그릴 로봇들의 원래·재계획 경로와 목적지가 모두 들어가는 가장 작은 사각형."""
    ct, rows = d["current_time"], d["rows"]
    agents = {a["id"]: a for a in d["agents"]}
    xs, ys = [], []
    for rid in roles:
        cells = list(future(agents[rid]["initial"], ct)) + [tuple(agents[rid]["goal"])]
        for v in variants:
            vv = d["variants"][v]
            if vv["ok"]:
                cells += future(vv["paths"][str(rid)], ct)
        xs += [c[0] for c in cells]
        ys += [c[1] for c in cells]
    xs += [c[0] for c in extra]
    ys += [c[1] for c in extra]
    h, w = len(rows), len(rows[0])
    return (max(0, min(xs) - pad), min(w - 1, max(xs) + pad), max(0, min(ys) - pad), min(h - 1, max(ys) + pad))


def draw_panel(ax, d, variant, roles, crop, note=None, extra_lines=()):
    rows, ct = d["rows"], d["current_time"]
    x0, x1, y0, y1 = crop
    for y in range(y0, y1 + 1):
        for x in range(x0, x1 + 1):
            if rows[y][x] == "#":
                ax.add_patch(Rectangle((x - 0.5, y - 0.5), 1, 1, color=WALL, linewidth=0))
    for ox, oy in d["obstacles"]:
        if x0 <= ox <= x1 and y0 <= oy <= y1:
            ax.add_patch(Rectangle((ox - 0.5, oy - 0.5), 1, 1, color=INK, linewidth=0))
            ax.text(ox, oy, "X", ha="center", va="center", color=SURFACE, fontsize=8, fontweight="bold")
    agents = {a["id"]: a for a in d["agents"]}
    v = d["variants"][variant]
    for xs, ys, color, lab in extra_lines:
        ax.plot(xs, ys, color=color, linewidth=1.2, linestyle=(0, (2, 2)), zorder=2, label=lab)
    for rid, (color, lab) in roles.items():
        a = agents[rid]
        old = future(a["initial"], ct)
        new = future(v["paths"][str(rid)], ct) if v["ok"] else old
        if old != new:
            ax.plot([c[0] for c in old], [c[1] for c in old], color=MUTED, linewidth=1, alpha=0.8, zorder=2)
        ax.plot([c[0] for c in new], [c[1] for c in new], color=color, linewidth=2.4, zorder=3,
                solid_capstyle="round", label=lab)
        sx, sy = pos(a["initial"], ct)
        gx, gy = a["goal"]
        ax.scatter([sx], [sy], s=110, color=color, edgecolor=SURFACE, linewidth=2, zorder=4)
        ax.text(sx, sy, str(rid), ha="center", va="center", fontsize=6.5, color=SURFACE, fontweight="bold", zorder=5)
        if (gx, gy) != (sx, sy):
            ax.scatter([gx], [gy], s=70, marker="s", facecolor=SURFACE, edgecolor=color, linewidth=2, zorder=4)
    ax.set_xlim(x0 - 0.5, x1 + 0.5)
    ax.set_ylim(y1 + 0.5, y0 - 0.5)
    ax.set_aspect("equal")
    step = 2 if max(x1 - x0, y1 - y0) <= 16 else 5
    ax.set_xticks(range(x0, x1 + 1, step))
    ax.set_yticks(range(y0, y1 + 1, step))
    ax.tick_params(labelsize=7)
    ax.grid(True, color=GRID, linewidth=0.5)
    if note:
        ax.set_title(note, loc="left", fontsize=10)


def legend_below(fig, axes):
    seen = {}
    for ax in np.atleast_1d(axes):
        for h, l in zip(*ax.get_legend_handles_labels()):
            if l and l not in seen:
                seen[l] = h
    fig.legend(seen.values(), seen.keys(), loc="lower center", ncol=min(4, len(seen)), bbox_to_anchor=(0.5, -0.06),
               fontsize=9)


def example_p1():
    d = json.load(open(os.path.join(EXAMPLES, "default_corridor_20_36.json"), encoding="utf-8"))
    fig, axes = plt.subplots(1, 2, figsize=(12, 5.2))
    call = next(c for c in d["variants"]["ff_idea3"]["astar"] if not c["ok"])
    roles = {13: (BLUE, "로봇 13: 장애물에 막힌 로봇"), 15: (ORANGE, "로봇 15: 13의 우회로 위에 서 있는 로봇")}
    crop = auto_crop(d, ["priority", "ff_idea3"], roles, extra=[tuple(o) for o in d["obstacles"] if o[0] >= 20])
    total = d["variants"]["priority"]["ms"]
    draw_panel(axes[0], d, "priority", roles, crop,
               f"원래 방식: 13이 먼저 계획 → 15의 등록이 거절돼\n15를 불러 비키게 함 (전체 {total:.0f} ms)")
    draw_panel(axes[1], d, "ff_idea3", roles, crop,
               f"고정 로봇 먼저: 15가 먼저 등록 → 13의 A*가 {call['expanded']:,}개\n"
               f"상태(전체의 85%)를 뒤지고 실패 ({call['ms']:.0f} ms) → 그제야 15를 부름")
    legend_below(fig, axes)
    title(fig, "그림 7. 문제 1 예시 — corridor 20대 #36 (지금 시각 t=10)",
          "회색 칸 = 벽, 검은 X = 새 장애물, 동그라미 = 지금 위치, 네모 = 목적지, 회색 선 = 원래 경로, "
          "색 선 = 재계획 경로. 결과는 둘 다 같지만, 오른쪽은 '실패'를 확인하는 데 시간을 다 쓴다.", y=1.06)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "example_1_slow_failure.png"), dpi=150, bbox_inches="tight")
    plt.close(fig)


def example_p2():
    d = json.load(open(os.path.join(EXAMPLES, "default_corridor_40_39.json"), encoding="utf-8"))
    agents = {a["id"]: a for a in d["agents"]}
    ct = d["current_time"]
    start = pos(agents[7]["initial"], ct)
    route = bfs(d["rows"], start, tuple(agents[7]["goal"]), {tuple(o) for o in d["obstacles"]})
    fig, axes = plt.subplots(1, 2, figsize=(12, 5.4))
    roles_b = {7: (BLUE, "로봇 7: 장애물에 막힌 로봇"), 30: (AQUA, "불렀고 실제로 비켜 준 로봇")}
    roles_p = dict(roles_b)
    for rid in (31, 1, 20):
        roles_p[rid] = (ORANGE, "불렀지만 안 움직인 로봇(헛부탁)")
    crop = auto_crop(d, ["fixed_first", "ff_idea3"], roles_p, extra=route)
    line = ([c[0] for c in route], [c[1] for c in route], INK2, "아이디어 3이 가정한 최단 경로(쉬지 않고 감)")
    draw_panel(axes[0], d, "fixed_first", roles_b, crop, "막혀 본 모든 칸으로 고름: 30 한 대만 부름")
    draw_panel(axes[1], d, "ff_idea3", roles_p, crop, "최단 경로와 부딪히는 로봇: 30, 31, 1, 20을 부름 → 31·1·20은 그대로",
               extra_lines=[line])
    legend_below(fig, axes)
    title(fig, "그림 8. 문제 2 예시 — corridor 40대 #39 (지금 시각 t=29)",
          "아이디어 3은 '쉬지 않고 가는 최단 경로'와 부딪히는 로봇을 부른다. 그런데 로봇 7은 실제로는 다른 길로 "
          "돌아가서, 목적지에 서 있던 31·1·20은 비켜 줄 필요가 없었다.", y=1.06)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "example_2_idle_rescue.png"), dpi=150, bbox_inches="tight")
    plt.close(fig)


def example_p3():
    d = json.load(open(os.path.join(EXAMPLES, "large_corridor_150_49.json"), encoding="utf-8"))
    fig, axes = plt.subplots(1, 2, figsize=(12, 6.2))
    roles = {67: (BLUE, "로봇 67: 장애물에 막힌 로봇 (앞 순서 = 더 중요)"),
             114: (AQUA, "로봇 114: 자기 목적지에 서 있는 로봇 (뒤 순서 = 덜 중요)")}
    crop = auto_crop(d, ["priority", "fixed_first"], roles, extra=[tuple(o) for o in d["obstacles"] if o[0] > 40])
    draw_panel(axes[0], d, "priority", roles, crop,
               "원래 방식: 67이 먼저 계획 → 114의 등록이 거절돼\n114를 불러 비키게 함 → 67은 t=82 도착")
    draw_panel(axes[1], d, "fixed_first", roles, crop,
               "고정 로봇 먼저: 114가 먼저 등록돼 그대로 서 있음\n→ 67이 114를 피해 크게 돌아감 → t=148 도착 (+66)")
    legend_below(fig, axes)
    title(fig, "그림 9. 문제 3 예시 — 64×64 corridor 150대 #49 (지금 시각 t=59)",
          "더 중요한 로봇(67)이 덜 중요한 로봇(114)의 원래 경로를 지키느라 66스텝 늦게 도착한다. "
          "400개 중 가장 크게 늦어진 경우다(보통은 1~2스텝).", y=1.06)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "example_3_yield.png"), dpi=150, bbox_inches="tight")
    plt.close(fig)


if __name__ == "__main__":
    os.makedirs(OUT, exist_ok=True)
    style()
    fig_problem1()
    fig_problem2()
    fig_problem3()
    example_p1()
    example_p2()
    example_p3()
    print("saved to", OUT)

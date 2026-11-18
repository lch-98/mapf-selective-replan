"""23장 실험 ②: 서 있는 로봇이 우회로에 있는 장면(standing)에서 작업 시간이 결과를 바꾸는가.

사용: python tools/analyze_standing.py
 - path 모드(기존): tools/results/v3_service/ (idea3_path_t1, idea3_large_path_t1, small_s*, large_s*)
 - standing 모드:  tools/results/v4_standing/{small,large}_s{0,1_10,30_60}.csv
"""
import csv
import os
import statistics as st

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
V3 = os.path.join(ROOT, "tools", "results", "v3_service")
V4 = os.path.join(ROOT, "tools", "results", "v4_standing")
RESULT = ["full_replan_ok", "selective_replan_ok", "escalation_tier", "full_replan_soc", "selective_replan_soc",
          "selective_replanned", "full_replan_delayed", "selective_delayed", "full_replan_away",
          "selective_away", "selective_rescued"]


def load(path):
    with open(path, encoding="utf-8") as f:
        return list(csv.DictReader(f))


def summary(label, rows):
    n = len(rows)
    both = [r for r in rows if r["full_replan_ok"] == "1" and r["selective_replan_ok"] == "1"]
    m = lambda c: st.mean(int(r[c]) for r in both)
    alone = sum(r["selective_replan_ok"] == "1" and int(r["escalation_tier"]) >= 0 for r in rows)
    rescued = sum(r["selective_replan_ok"] == "1" and int(r["selective_rescued"]) > 0 for r in rows)
    working = st.mean(int(r["working_at_t"]) / int(r["num_agents"]) for r in rows)
    print(f"  {label:16s} 성공 전체 {sum(r['full_replan_ok']=='1' for r in rows):3d} 우리 {sum(r['selective_replan_ok']=='1' for r in rows):3d}"
          f" (안전망없이 {alone:3d}) | 구조 로봇 부른 시나리오 {rescued:3d} | 작업 중 비율 {100*working:4.1f}%"
          f" | 늦어짐 {m('full_replan_delayed'):.2f}/{m('selective_delayed'):.2f}"
          f" | 목적지밖 {m('full_replan_away'):.0f}/{m('selective_away'):.0f} (전체/우리, 둘 다 성공 {len(both)}개)")


def changed(base, other):
    rows = [(a, b) for a, b in zip(base, other) if any(a[c] != b[c] for c in RESULT)]
    full = sum(1 for a, b in rows if any(a[c] != b[c] for c in RESULT if c.startswith("full")))
    ours = sum(1 for a, b in rows if any(a[c] != b[c] for c in RESULT if not c.startswith("full")))
    return full, ours


def main():
    for size, path_base in (("small", "idea3_path_t1"), ("large", "idea3_large_path_t1")):
        print(f"\n===== {size} =====")
        print("[path 모드(기존)]")
        p0 = load(os.path.join(V3, path_base + ".csv"))
        summary("작업 0", p0)
        for lv in ("1_10", "30_60"):
            pl = load(os.path.join(V3, f"{size}_s{lv}.csv"))
            summary(f"작업 U[{lv.replace('_', ',')}]", pl)
            f, o = changed(p0, pl)
            print(f"  {'':16s} → 작업 0과 결과가 달라진 시나리오: 전체 재계획 {f} / 우리 {o} (400개 중)")
        print("[standing 모드(새)]")
        s0 = load(os.path.join(V4, f"{size}_s0.csv"))
        summary("작업 0", s0)
        for lv in ("1_10", "30_60"):
            sl = load(os.path.join(V4, f"{size}_s{lv}.csv"))
            same_scen = all(a["initial_soc"] == b["initial_soc"] and a["current_time"] == b["current_time"]
                            for a, b in zip(s0, sl))
            summary(f"작업 U[{lv.replace('_', ',')}]", sl)
            f, o = changed(s0, sl)
            print(f"  {'':16s} → 작업 0과 결과가 달라진 시나리오: 전체 재계획 {f} / 우리 {o} (400개 중), 시나리오 동일: {same_scen}")
        print(f"  standing 평균 장애물 시각 {st.mean(int(r['current_time']) for r in s0):.1f} "
              f"(path {st.mean(int(r['current_time']) for r in p0):.1f}), "
              f"재시도 평균 {st.mean(int(r['plan_attempts']) for r in s0):.2f} (path {st.mean(int(r['plan_attempts']) for r in p0):.2f})")


if __name__ == "__main__":
    main()

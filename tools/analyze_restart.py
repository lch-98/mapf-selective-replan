"""30장: Tier 1/2 × 안전망 ②(순서 바꿔 다시, --order-restarts) 비교.
스스로 = 안전망 없이 푼 수, 안전망② = 순서 바꿔 다시로 푼 수, 성공 = 전체 성공. 시간은 우리 방법(ms)."""
import csv, os, statistics as st, sys
R = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "tools", "results")
BASE = {"base64": ("v3_service", "idea3_large_path_t1.csv"), "s64": ("v3_service", "large_s30_60.csv"),
        "st64s0": ("v4_standing", "large_s0.csv"), "st64s30": ("v4_standing", "large_s30_60.csv"),
        "st32s0": ("v4_standing", "small_s0.csv")}
def load(*p):
    path = os.path.join(R, *p)
    return list(csv.DictReader(open(path, encoding="utf-8"))) if os.path.exists(path) and os.path.getsize(path) else None
def q(v, p): v = sorted(v); return v[min(len(v) - 1, int(len(v) * p))]
for name, base in BASE.items():
    sets = {"T1 (기존)": load(*base), "T2": load("v7_restart", f"{name}_t2_r0.csv"),
            "T1+②3": load("v7_restart", f"{name}_t1_r3.csv"), "T2+②3": load("v7_restart", f"{name}_t2_r3.csv")}
    print(f"\n== {name} ==")
    for k, rows in sets.items():
        if not rows: print(f"  {k:10s} (없음)"); continue
        ok = [r for r in rows if r["selective_replan_ok"] == "1"]
        self_ok = sum(r["escalation_tier"] != "-1" for r in ok)
        rs = sum(int(r.get("order_restart") or 0) > 0 for r in ok)
        full = sum(r["full_replan_ok"] == "1" for r in rows)
        ms = [float(r["selective_replan_ms"]) for r in rows]
        print(f"  {k:10s} 스스로 {self_ok:3d}/{len(rows)}  안전망②로 성공 {rs}  성공 {len(ok)} (전체 재계획 {full})  "
              f"시간 중앙 {st.median(ms):6.1f} p95 {q(ms,.95):7.1f} 최대 {max(ms):7.1f} ms")

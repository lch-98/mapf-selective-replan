"""6번 Park식 비교(28장): 안전망 없이 스스로 푼 수, 다시 계획한 로봇 수, 회차 수, 늦어진 로봇 수.
사용: python tools/analyze_park.py"""
import csv
import os
import statistics as st

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
R = os.path.join(ROOT, "tools", "results")


def load(*p):
    with open(os.path.join(R, *p), encoding="utf-8") as f:
        return list(csv.DictReader(f))


def summary(rows):
    n = len(rows)
    ok = [r for r in rows if r["selective_replan_ok"] == "1"]
    self_ok = sum(1 for r in ok if r["escalation_tier"] != "-1")
    conf = sum(int(r["selective_conflicts"]) for r in ok)
    rep = st.mean(int(r["selective_replanned"]) for r in ok if r["escalation_tier"] != "-1") if self_ok else float("nan")
    ev = st.mean(int(r.get("selective_events", 1) or 1) for r in ok)
    dl = st.mean(int(r["selective_delayed"]) for r in ok)
    return n, self_ok, len(ok), rep, ev, dl, conf


def main():
    for size, ours, orig in (("small", "idea3_path_t1", "order_priority_t1"), ("large", "idea3_large_path_t1", "order_large_priority_t1")):
        base = {"우리 방법": load("v3_service", ours + ".csv"), "원래 방식(Tier1)": load("v3_service", orig + ".csv")}
        for w in (10, 20, 40):
            base[f"Park 섞어 W{w}"] = load("v6_park", f"park_mixed_{size}_w{w}.csv")
            base[f"Park 고정먼저 W{w}"] = load("v6_park", f"park_fixed_{size}_w{w}.csv")
        full = base["우리 방법"]
        print(f"\n=== {size} ===")
        for (m, n_ag) in sorted({(r["map"], int(r["num_agents"])) for r in full}):
            fr = [r for r in full if r["map"] == m and int(r["num_agents"]) == n_ag]
            print(f"-- {m} {n_ag}대: 전체 재계획 성공 {sum(r['full_replan_ok']=='1' for r in fr)}/{len(fr)}")
            for name, rows in base.items():
                rs = [r for r in rows if r["map"] == m and int(r["num_agents"]) == n_ag]
                n, so, ok, rep, ev, dl, conf = summary(rs)
                print(f"   {name:18s} 스스로 {so:3d}/{n}  성공 {ok:3d}  다시계획(스스로) {rep:5.1f}  회차 {ev:4.2f}  늦어진 로봇 {dl:5.1f}  충돌 {conf}")


if __name__ == "__main__":
    main()

"""23장 실험 ①: 계산 시간을 로봇 손해로 바꿔 잰다.

규칙(두 방법에 똑같이): 한 스텝 = Δ초. 재계획에 c초 걸리면 모든 로봇이 k = max(0, ceil(c/Δ) - 1) 스텝 멈춘다.
손해 = (재계획 결과의 도착 시각 합 - 처음 계획의 도착 시각 합) + k × moving_at_t.

사용: python tools/analyze_latency.py CSV [CSV ...]   (--threads 1로 잰 결과)
"""
import csv
import math
import statistics as st
import sys

DELTAS = [0.25, 0.5, 1.0, 2.0]


def stall_steps(ms, delta_s):
    return max(0, math.ceil(ms / 1000.0 / delta_s) - 1)


def main():
    for path in sys.argv[1:]:
        rows = list(csv.DictReader(open(path, encoding="utf-8")))
        print(f"\n== {path}")
        print("로봇 수 | Δ(초) | 멈춘 시나리오 % (전체/선택) | 멈춘 스텝 평균 (전체/선택) | "
              "로봇당 늦어짐 평균: 경로 + 기다림 = 합 (전체 | 선택) | 선택이 덜 늦은 시나리오 %")
        counts = sorted({r["num_agents"] for r in rows}, key=int)
        for n in counts:
            both = [r for r in rows if r["num_agents"] == n
                    and r["full_replan_ok"] == "1" and r["selective_replan_ok"] == "1"]
            for d in DELTAS:
                fk = [stall_steps(float(r["full_replan_ms"]), d) for r in both]
                sk = [stall_steps(float(r["selective_replan_ms"]), d) for r in both]
                f_path = [int(r["full_replan_soc"]) - int(r["initial_soc"]) for r in both]
                s_path = [int(r["selective_replan_soc"]) - int(r["initial_soc"]) for r in both]
                moving = [int(r["moving_at_t"]) for r in both]
                f_wait = [k * m for k, m in zip(fk, moving)]
                s_wait = [k * m for k, m in zip(sk, moving)]
                f_tot = [a + b for a, b in zip(f_path, f_wait)]
                s_tot = [a + b for a, b in zip(s_path, s_wait)]
                agents = int(n)
                better = sum(1 for a, b in zip(f_tot, s_tot) if b < a)
                worse = sum(1 for a, b in zip(f_tot, s_tot) if b > a)
                print(f"{n:>5s} | {d:4.2f} | {100*sum(k>0 for k in fk)/len(both):5.1f} / {100*sum(k>0 for k in sk)/len(both):4.1f} | "
                      f"{st.mean(fk):4.2f} / {st.mean(sk):4.2f} | "
                      f"{st.mean(f_path)/agents:5.2f} + {st.mean(f_wait)/agents:5.2f} = {st.mean(f_tot)/agents:5.2f} | "
                      f"{st.mean(s_path)/agents:5.2f} + {st.mean(s_wait)/agents:5.2f} = {st.mean(s_tot)/agents:5.2f} | "
                      f"{100*better/len(both):4.1f}% (더 늦음 {100*worse/len(both):.1f}%)")


if __name__ == "__main__":
    main()

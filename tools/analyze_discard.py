"""버린 배치를 이유별로 센다: plan_failures(처음 PP 계획 실패) / obstacle_retries(장애물이 아무도 안 막음).
사용: python tools/analyze_discard.py tools/results/v5_discard/*.csv   (26장 26.6)"""
import csv, sys, collections
for path in sys.argv[1:]:
    rows = list(csv.DictReader(open(path, encoding='utf-8')))
    bad = [r for r in rows if r['plan_ok']=='1' and int(r['plan_attempts']) != int(r['plan_failures'])+int(r['obstacle_retries'])+1]
    print(path.split('/')[-1], 'rows', len(rows), 'invariant violations', len(bad), 'plan_ok=0', sum(r['plan_ok']!='1' for r in rows))
    g = collections.defaultdict(list)
    for r in rows: g[(r['map'], int(r['num_agents']))].append(r)
    for (m, n), rs in sorted(g.items(), key=lambda x:(x[0][0], x[0][1])):
        tried = sum(int(r['plan_attempts']) for r in rs)
        pf = sum(int(r['plan_failures']) for r in rs); ob = sum(int(r['obstacle_retries']) for r in rs)
        # 처음 PP 계획까지 간 배치 중 실패 비율 = pf / tried (모든 시도는 plan()을 거친다)
        print(f"  {m:10s} {n:5d}  tried {tried:5d}  plan_fail {pf:5d} ({100*pf/tried:5.1f}%)  obstacle {ob:5d} ({100*ob/tried:5.1f}%)  max_pf {max(int(r['plan_failures']) for r in rs)}")

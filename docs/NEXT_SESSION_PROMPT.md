# 다음 세션 이어받기 프롬프트 (2026-11-27 작성, 7-6까지 끝난 뒤)

아래 상자 안의 글을 새 세션 첫 메시지로 그대로 붙여 넣는다. (Ubuntu에서 이어서 할 때 기준. Windows에서는 3D를 못 돌리므로 문서 작업만.)

```
이 저장소(MAPF 선택적 재계획 연구)를 이어서 하자. 일정 7(ROS2 + Gazebo 3D)은 7-0~7-6까지 끝났어.

먼저 읽을 것(이 순서로):
1. CLAUDE.md — 규칙, 용어, "지금 위치"
2. docs/PROGRESS.md 맨 아래 기록(2026-11-25 ~ 11-27)
3. docs/32_3d_experiment_results.md — 7-6 본 실험 결과 전부(32.0 쉬운 요약, 32.6 "해석 3" 결정 거리)
4. docs/31_3d_gazebo_design.md 31.24(곧게 지나가기·비상 정지 v2), 31.25(LiDAR 발견) — 설계가 바뀐 곳
5. ros2/README.md — 빌드·실행 명령(맨 아래 31.24·31.25 옵션)

다 읽으면 지금 상태를 짧게 요약하고, 빌드와 테스트가 통과하는지 확인해 줘:
- 코어: cmake --build build -j, ./build/tests/run_tests(88개), python3 python/smoke_test.py
- 3D: cd ros2 && source /opt/ros/humble/setup.bash && colcon build --symlink-install && source install/setup.bash,
  ./build/mapf_exec/test_mapf_exec(57개), ./build/mapf_exec/test_analyze_run(2개), python3 -m pytest mapf_gazebo/test -q(6개)

그다음 할 일:
1. 32.6에서 내가 고른 안(A/B/C)에 맞춰 진행. (A면 논문용 문장·표 정리, B면 "우회 길이 상한" 스위치 설계부터 — 설계 → 내가 이해 → 코드 순서)
2. 시연 영상(31.12 7-6): 녹화(ros2/results/7_6/**/bag/, 이 컴퓨터에만)를 RViz로 다시 보며 화면 녹화. 어떤 장면을 쓸지 같이 정하자
   (후보: 7_5/hold/corridor10_r6_s1_hold_ours — HOLD 후 막다른 통로에서 되돌아 나옴, 7_6/base/corridor10_r6_s2_ob1_ours/full 나란히).
3. 8번(실제 로봇) 준비: 실험 바닥 크기를 알려 줄게. 위치 추정 방법(31.8)부터 정하자.

진행 방식: 한꺼번에 구현한 뒤 결과를 모아 보고. 나는 다시 보기(RViz)로 결과를 따라가며 이상한 장면이 있으면 추가 검증을 부탁할게.
테스트·변이 검사·문서 기록과 커밋은 지금처럼(한국어 메시지, push는 내가 말할 때만). 칸 위치는 "9행 7열"처럼 행·열로 말해 줘.
Gazebo를 띄우고 끌 때는 CLAUDE.md 규칙(내가 띄운 Gazebo가 있는지 먼저 확인, 자기 PID/프로세스 그룹으로만 끄기, 여러 실행은 run_batch)을 지켜 줘.
```

## 참고: 지난 세션에서 정한 것 한눈에

- 3D 실행 = ADG + 곧게 지나가기(약속이 다 지켜진 같은 방향 다음 칸은 미리 내줌). 재계획 = 지금 모습 = 새 0번 + 약속을 지키는 번호 다시 붙이기 → 코어 `replan()`/`full_replan()`.
- 장애물 발견 = LiDAR(다른 로봇 ± 0.2 m·선반/벽 면 4 cm 안 점 버림, 3점 이상, 최근 T_wait 90% → 확정, 1초 안 보이면 버림). 흉내 규칙은 `detection: simulated`(7-4).
- 비상 정지 v2 = 앞으로 갈 거리 안 + 광선 2개 이상. 세 스위치 모두 코드 기본값은 꺼짐, 실험은 `run_batch --pass-through --robust-estop --detection lidar`.
- 재계획마다 `replan_<n>.json` 저장 → `check_replan_input`(3D = 2D 확인), `same_input_compare`(같은 입력에서 두 방법).
- 7-6 결과: 70번 안전 위반 0, 성공 우리 50/50 · 전체 48/50(막다른 통로 끝 장애물), 바뀐 로봇 3.2~3.4 vs 3.8~4.2, 계산 1 ms 안팎(차이 없음).
- 녹화 파일(`ros2/results/**/bag/`)은 git에 없음 — 이 컴퓨터에만 있다. 실제 로봇은 6대, 실험 바닥 크기는 아직 모름.

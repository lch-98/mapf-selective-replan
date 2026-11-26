# cell_follower → 실행기: 동작 상태 (docs/31 31.10)
uint8 ACTIVE=0        # 받아서 하는 중
uint8 DONE=1          # 끝남 (MOVE: 목표 칸 중심 도착, WAIT: 시간 다 됨)
uint8 HELD=2          # 앞에 장애물이 보여 시작을 보류함 (7-4에서 씀)
uint8 ESTOP=3         # 비상 정지 (정면 가까이에 무언가) — 실패로 센다(31.4)

uint32 seq
uint8 state
float64 pos_err_m     # DONE일 때 목표 칸 중심까지 거리(m)

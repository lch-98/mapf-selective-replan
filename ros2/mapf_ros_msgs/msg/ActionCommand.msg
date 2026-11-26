# 실행기(executive) → cell_follower: "이 동작 해도 됨" (docs/31 31.10)
# 좌표는 월드 m. 칸 번호(cell_x = 열, cell_y = 행)는 기록용.
uint8 MOVE=0          # from(출발 칸 중심) → target(이웃 칸 중심)으로 돌고-가기
uint8 WAIT=1          # 제자리에서 duration_s초 (작업, 대기)

uint32 seq            # 로봇마다 0부터 1씩 늘어나는 동작 번호
uint8 kind
int32 cell_x          # 목표 칸 열
int32 cell_y          # 목표 칸 행
float64 from_x
float64 from_y
float64 target_x
float64 target_y
float64 duration_s

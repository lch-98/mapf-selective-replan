"""RViz2 설정(.rviz)을 로봇 수에 맞춰 만든다 — 3D 실험을 한 창에서 보기 (docs/31 31.23).

보이는 것: 바닥 격자(0.5 m 칸), 실행기 마커(/mapf/markers: 선반·장애물·가진 칸·남은 경로·상태 원판과 글자·양보 화살표·맨 위 정보 줄),
로봇마다 3D 모델(robot_state_publisher의 robot_description, TF 접두어 robot_i)과 LiDAR 점.
시점: 위에서 내려다봄(위 = +y = 2D 그림의 0행 쪽, 오른쪽 = +x). 고정 프레임 world(Gazebo odom = 월드 좌표라 world → robot_i/odom은 그대로).
"""

COLORS = ['31; 119; 180', '255; 127; 14', '44; 160; 44', '214; 39; 40', '148; 103; 189', '140; 86; 75',
          '227; 119; 194', '127; 127; 127', '188; 189; 34', '23; 190; 207']


def make_rviz(names, width_m, height_m):
    robots = ''
    for i, n in enumerate(names):
        robots += f'''
    - Class: rviz_default_plugins/RobotModel
      Name: {n} 모델
      Enabled: true
      Description Source: Topic
      Description Topic:
        Value: /{n}/robot_description
        Depth: 5
        Durability Policy: Transient Local
        History Policy: Keep Last
        Reliability Policy: Reliable
      TF Prefix: {n}
      Visual Enabled: true
      Collision Enabled: false
      Alpha: 1
    - Class: rviz_default_plugins/LaserScan
      Name: {n} LiDAR
      Enabled: true
      Topic:
        Value: /{n}/scan
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Best Effort
      Style: Points
      Size (Pixels): 2
      Size (m): 0.02
      Color Transformer: FlatColor
      Color: {COLORS[i % len(COLORS)]}
      Alpha: 0.6
      Decay Time: 0'''
    cells = int(max(width_m, height_m) / 0.5) + 2
    return f'''Panels:
  - Class: rviz_common/Displays
    Name: Displays
    Help Height: 0
Visualization Manager:
  Class: ""
  Displays:
    - Class: rviz_default_plugins/Grid
      Name: 칸(0.5 m)
      Enabled: true
      Cell Size: 0.5
      Plane Cell Count: {cells}
      Normal Cell Count: 0
      Line Style:
        Line Width: 0.03
        Value: Lines
      Color: 120; 120; 120
      Alpha: 0.5
      Plane: XY
      Reference Frame: <Fixed Frame>
      Offset:
        X: {width_m / 2}
        Y: {height_m / 2}
        Z: 0
    - Class: rviz_default_plugins/MarkerArray
      Name: MAPF 상태
      Enabled: true
      Topic:
        Value: /mapf/markers
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable{robots}
  Enabled: true
  Global Options:
    Background Color: 40; 40; 40
    Fixed Frame: world
    Frame Rate: 30
  Views:
    Current:
      Class: rviz_default_plugins/TopDownOrtho
      Name: Current View
      Target Frame: <Fixed Frame>
      Angle: 0
      Scale: {int(800 / max(width_m, height_m, 1))}
      X: {width_m / 2}
      Y: {height_m / 2}
      Near Clip Distance: 0.01
      Invert Z Axis: false
Window Geometry:
  Width: 1100
  Height: 950
'''

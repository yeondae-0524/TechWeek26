# 공통 Interface 규격

모든 모듈은 이 규격을 따른다. 코드 정의: `controllers/rescue_robot/interfaces.py`.
규격을 바꾸려면 **이 문서 + interfaces.py + tests/test_interfaces.py** 를 같이 바꾸고 팀에 공유한다.

## 1. 좌표계 (Webots R2025a에서 실측 확인)

| 항목 | 규칙 |
|---|---|
| World frame | Webots 기본 `ENU`: **x = 동(+x)**, **y = 북(+y, x의 왼쪽)**, **z = 위**. 단위 m |
| `theta = 0` | 로봇이 **+x** 방향을 바라봄 |
| `theta` 증가 방향 | **반시계(CCW, +z축 기준)** = 왼쪽 회전 시 증가. 범위 `(-pi, pi]` |
| Webots `rotation 0 0 1 a` | 로봇 heading `theta = a` |

검증 근거 (`scripts/verify_baseline.py --webots --mode CONTROL_TEST`, rescue_baseline.wbt):

- 시작 pose `(-0.5, -0.8, 0)`에서 forward → x 증가 (odom +0.060 m, GPS +0.059 m)
- `rotate_left` 1.5 s → odom theta +64.6° (명령값 0.75 rad/s × 1.5 s = 64.5°). GPS 궤적도 CCW 회전과 일치
- LiDAR 시작 거리: front 0.425 m(빨간 박스 면), left 0.280 m(low wall), back 0.500 m(arena 벽), right 0.200 m(arena 벽). world 배치에서 계산한 값과 정확히 일치

> 주의: practice e-puck의 GPS는 turret에 달려 있어 로봇 중심보다 0.0095 m 뒤에 있다
> (heading 0일 때 GPS x = pose x − 0.0095). GPS는 **디버그 전용**이며 localization 입력으로 쓰지 않는다.

## 2. Occupancy Grid

```python
UNKNOWN = -1
FREE = 0
OCCUPIED = 1

grid[row][col]          # list of lists, grid[row] 가 한 행
resolution = 0.05       # m / cell  (config.GRID_RESOLUTION)
```

- **row는 +y 방향**, **col은 +x 방향**으로 증가한다. 이미지처럼 출력하면 위아래가 뒤집혀 보이므로,
  사람이 볼 때는 `reversed(grid)`로 출력한다(`OccupancyGrid.save_pgm`이 그렇게 한다).
- **grid origin** `(ox, oy)` = cell `(0, 0)`의 **왼쪽 아래 모서리**의 world 좌표.
  - 기본값(`config.GRID_ORIGIN = None`)은 grid 중심이 `home_pose`에 오도록 자동 계산:
    `ox = home_x − GRID_WIDTH·res/2`, `oy = home_y − GRID_HEIGHT·res/2`
- 크기: `GRID_WIDTH` = column 수(x), `GRID_HEIGHT` = row 수(y). 기본 160×160 = 8 m × 8 m

### World ↔ Grid 변환

```python
col = floor((x - ox) / res)
row = floor((y - oy) / res)          # world_to_grid(x, y) -> (row, col)

x = ox + (col + 0.5) * res
y = oy + (row + 0.5) * res           # grid_to_world(row, col) -> 셀 "중심" (x, y)
```

- `world_to_grid`는 범위를 벗어난 index도 반환한다. 사용 전 `in_bounds(row, col)` 확인
  (`0 <= row < GRID_HEIGHT`, `0 <= col < GRID_WIDTH`).
- `world_to_grid(grid_to_world(r, c)) == (r, c)` 가 항상 성립한다.

## 3. Robot Pose

```python
pose = (x, y, theta)    # m, m, rad   world frame, theta in (-pi, pi]
home_pose               # 시작 pose (대회 당일 제공값 → config.START_POSE)
```

`interfaces.make_pose()`로 만들면 theta가 자동 정규화된다.

## 4. Detection

```python
target = {
    "found": False,      # bool
    "cx": None,          # int, 이미지 x 픽셀 (found=True일 때)
    "direction": None,   # "LEFT" | "CENTER" | "RIGHT" (found=True일 때)
    "area": 0.0,         # float, 픽셀 면적
}
```

- `direction`: 이미지 가로폭을 3등분. `cx < W/3` → LEFT, `cx < 2W/3` → CENTER, 나머지 RIGHT
- 입력 frame: NumPy `(H, W, 3)` BGR uint8 (`Devices.read_camera_frame()`), 카메라가 없으면 `None`
- `detect_target()`는 호출마다 **새 dict**를 반환한다.

## 5. Planning

```python
path = [(row1, col1), (row2, col2), ...]   # start 포함, goal 포함
```

- 경로 없음 → `[]`, `start == goal` → `[start]`
- 4-neighbour 이동, OCCUPIED/grid 밖 통과 금지. UNKNOWN은 기본적으로 통과 가능(`allow_unknown`)
- Frontier: **FREE cell 중 4-neighbour에 UNKNOWN이 있는 cell**. `find_frontiers(grid) -> [(row, col), ...]`,
  `cluster_frontiers(frontiers) -> [[(row, col), ...], ...]`(8-연결, 큰 cluster 먼저)

## 6. Control

```python
waypoint = (x, y)       # world m
```

- `set_velocity(v, w)`: v [m/s] 전진 +, w [rad/s] 반시계(왼쪽) +
- wheel 속도 단위: rad/s (Webots RotationalMotor)

## 7. LiDAR 각도 규칙 (practice robot, 실측)

```python
angle_i = LIDAR_FIRST_ANGLE + LIDAR_ANGLE_DIRECTION * i * fov / N   # robot frame, 0 = 전방, + = 왼쪽
# practice: LIDAR_FIRST_ANGLE = pi, LIDAR_ANGLE_DIRECTION = -1
# -> index 0 = 후방, index N/4 = 왼쪽, N/2 = 전방, 3N/4 = 오른쪽 (시계방향 sweep)
```

당일 LiDAR가 다르면 `config.py`의 두 값만 바꾸고, 부팅 로그 `[lidar] ranges: front=.. left=.. back=.. right=..`
가 실제 배치와 맞는지 확인한다.

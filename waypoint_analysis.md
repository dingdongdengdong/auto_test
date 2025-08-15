# Pure Pursuit Waypoint 제어 시스템 분석

## 개요
이 문서는 `simple_pure_pursuit.py`에서 구현된 waypoint 기반 자율주행 제어 시스템의 동작 원리와 로직을 상세히 분석합니다. L자형 코스를 위한 GPS + IMU + 카메라 융합 기반의 Pure Pursuit 알고리즘을 중심으로 설명합니다.

## 1. Waypoint 시스템 개요

### 1.1 Waypoint란?
- **정의**: 차량이 따라가야 할 경로상의 목표 지점들
- **형식**: GPS 좌표 (위도, 경도)가 UTM 좌표계로 변환되어 사용
- **총 개수**: 133개의 waypoint로 구성된 L자형 코스
- **파일**: `simple_L_course.txt`에 저장된 경로 데이터

### 1.2 좌표계 변환
```python
# GPS → UTM 변환
self.proj_UTM = Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)
xy_zone = self.proj_UTM(self.longitude, self.latitude)
```

## 2. Waypoint 추적 메커니즘

### 2.1 현재 Waypoint 계산
```python
local_path, current_waypoint = findLocalPath(self.global_path, self.odom_msg)
```

**핵심 프로세스:**
1. **거리 기반 계산**: 현재 차량 위치에서 가장 가까운 waypoint를 찾음
2. **Local Path 생성**: 현재 위치 기준으로 로컬 경로 생성
3. **Pure Pursuit 적용**: Look-Forward Distance(LFD)를 사용한 목표점 계산

### 2.2 거리 계산 알고리즘
```python
def find_current_waypoint(self):
    current_x = self.odom_msg.pose.pose.position.x
    current_y = self.odom_msg.pose.pose.position.y
    min_dist = float('inf')
    current_waypoint = 0
    
    for i, pose in enumerate(self.global_path.poses):
        dx = current_x - pose.pose.position.x
        dy = current_y - pose.pose.position.y
        dist = sqrt(dx*dx + dy*dy)
        
        if dist < min_dist:
            min_dist = dist
            current_waypoint = i
```

## 3. 미션 단계별 Waypoint 제어 전략

### 3.1 전체 구간 분석
L자형 코스는 4개의 주요 구간으로 나뉩니다:

| 구간 | Waypoint 범위 | 특성 | 속도 | LFD | 특별 제어 |
|------|---------------|------|------|-----|-----------|
| Straight-1 | 1-40 | 첫 직선 구간 | 25.0 km/h | 18m | 고속 접근 |
| Left-Turn | 40-80 | 좌회전 구간 | 12.0 km/h | 10m | 카메라+GPS 융합 |
| Straight-2 | 80-110 | 두 번째 직선 | 20.0 km/h | 15m | 고속 순항 |
| Final-Straight | 110-133 | 마지막 구간 | 15.0 km/h | 12m | 안전한 완주 |

### 3.2 구간별 상세 제어 로직

#### 3.2.1 직선 구간 1 (Waypoint 1-40)
```python
if 1 <= current_waypoint <= 40:
    self.setMotorMsgWithVel(25.0)  # 고속 주행
    self.setServoMsgWithLfd(18)    # 큰 LFD로 안정성 확보
    mission_phase = "Straight-1"
```

**특징:**
- **최고 속도**: 25 km/h로 빠른 접근
- **큰 LFD**: 18m로 직선에서의 안정성 확보
- **제어 방식**: GPS 기반 Pure Pursuit만 사용

#### 3.2.2 좌회전 구간 (Waypoint 40-80) - 핵심 구간
```python
elif 40 < current_waypoint <= 80:
    self.setMotorMsgWithVel(12.0)  # 회전 안전 속도
    self.setServoMsgWithLfd(10)    # 작은 LFD로 정밀 제어
    
    # 카메라 + GPS 융합 제어
    if self.use_camera_steering:
        gps_steering = self.servo_msg
        
        # 안전성 검증
        if abs(self.camera_servo_msg - gps_steering) < self.max_steering_deviation:
            # 안전한 경우: 카메라 80% + GPS 20%
            camera_weight = self.lane_safety_factor  # 0.8
            self.servo_msg = camera_weight * self.camera_servo_msg + (1-camera_weight) * gps_steering
            mission_phase = "Left-Turn-Safe-Hybrid"
        else:
            # 위험한 경우: GPS 80% + 카메라 20%
            self.servo_msg = 0.8 * gps_steering + 0.2 * self.camera_servo_msg
            mission_phase = "Left-Turn-GPS-Safe"
```

**하이브리드 제어 시스템:**
1. **기본 GPS 제어**: Pure Pursuit으로 기본 조향각 계산
2. **카메라 보정**: 차선 검출 결과로 미세 조정
3. **안전성 검증**: 두 값의 차이가 `max_steering_deviation` (0.3) 미만일 때만 융합
4. **가중치 적용**: 안전할 때 카메라 80%, 위험할 때 GPS 80%

#### 3.2.3 직선 구간 2 (Waypoint 80-110)
```python
elif 80 < current_waypoint <= 110:
    self.setMotorMsgWithVel(20.0)  # 회복된 고속 주행
    self.setServoMsgWithLfd(15)    # 중간 LFD
    mission_phase = "Straight-2"
```

**특징:**
- **회복 속도**: 20 km/h로 다시 가속
- **중간 LFD**: 15m로 균형잡힌 제어
- **안정성**: 회전 후 직선에서의 안정화

#### 3.2.4 최종 구간 (Waypoint 110-133)
```python
elif 110 < current_waypoint <= 133:
    self.setMotorMsgWithVel(15.0)  # 안전한 완주 속도
    self.setServoMsgWithLfd(12)
    mission_phase = "Final-Straight"

# 최종 접근 (120번 이후)
if current_waypoint > 120:
    self.setMotorMsgWithVel(6.0)   # 감속
    self.setServoMsgWithLfd(8)     # 작은 LFD로 정밀 제어
    mission_phase = "Final-Approach"
```

## 4. Pure Pursuit 알고리즘 구현

### 4.1 Look-Forward Distance (LFD)
```python
def setServoMsgWithLfd(self, lfd):
    steering, target_x, target_y = self.pure_pursuit.steering_angle(lfd)
    raw_steering = steering * self.steering_offset
    
    # 안전 제한 적용
    max_steering = 0.3
    self.servo_msg = max(-max_steering, min(max_steering, raw_steering))
```

**LFD의 역할:**
- **정의**: 현재 위치에서 목표점까지의 거리
- **효과**: 클수록 안정적이지만 덜 민감, 작을수록 민감하지만 불안정할 수 있음
- **구간별 최적화**: 직선에서는 크게, 회전에서는 작게 설정

### 4.2 조향각 계산 과정
1. **목표점 선정**: 현재 위치에서 LFD만큼 떨어진 경로상의 점
2. **각도 계산**: 현재 위치와 목표점 사이의 각도 계산
3. **Pure Pursuit 공식**: `steering = 2 * sin(alpha) * L / ld²`
   - `alpha`: 차량 방향과 목표점 방향의 각도차
   - `L`: 차량의 휠베이스
   - `ld`: Look-Forward Distance

### 4.3 안전성 제한
```python
# 조향각 제한
max_steering = 0.3  # ±17.2도
if abs(self.servo_msg) > 0.25:
    self.servo_msg = 0.25 if self.servo_msg > 0 else -0.25
```

## 5. 센서 융합 및 안전성

### 5.1 GPS + IMU 기본 제어
- **GPS**: 위치 정보 (위도, 경도, 고도)
- **IMU**: 자세 정보 (특히 yaw 각도)
- **융합**: UTM 좌표계에서 정확한 위치와 방향 계산

### 5.2 카메라 보조 제어
```python
def laneCB(self, msg):
    self.camera_servo_msg = msg.steering
    self.camera_motor_msg = msg.velocity
```

**카메라 제어 활성화 조건:**
```python
lane_detection_active = (20 < current_waypoint <= 100)
self.mission_pub.publish(Bool(lane_detection_active))
```

### 5.3 안전성 메커니즘
1. **조향각 제한**: 최대 ±0.3 라디안
2. **속도 제한**: 구간별 최적 속도 설정
3. **센서 검증**: 카메라와 GPS 값의 차이 검증
4. **비상 정지**: 경로 끝에서 자동 정지

## 6. 제어 메시지 발행

### 6.1 주요 토픽들
```python
# 발행하는 토픽들
self.global_path_pub = rospy.Publisher('/global_path', Path, queue_size=1)
self.ctrl_cmd_pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=1)
self.waypoint_pub = rospy.Publisher('/waypoint', Int64, queue_size=1)
self.mission_pub = rospy.Publisher('/mission', Bool, queue_size=1)
```

### 6.2 제어 명령 구조
```python
def publishCtrlCmd(self, motor_msg, servo_msg, brake_msg):
    self.ctrl_cmd_msg.velocity = motor_msg    # 속도 [km/h]
    self.ctrl_cmd_msg.steering = servo_msg    # 조향각 [rad]
    self.ctrl_cmd_msg.brake = brake_msg       # 브레이크 [0-1]
```

## 7. 모니터링 및 디버깅

### 7.1 로그 출력
```python
if current_waypoint % 5 == 0:  # 5 waypoint마다
    rospy.loginfo(f"[{mission_phase}] Waypoint: {current_waypoint}/{len(self.global_path.poses)}, "
                 f"Steering: {self.servo_msg:.3f}, Velocity: {self.motor_msg:.1f}, "
                 f"LFD: {lfd}, Yaw: {self.yaw:.1f}")
```

### 7.2 TF 발행
```python
self.br.sendTransform(
    (self.odom_msg.pose.pose.position.x, self.odom_msg.pose.pose.position.y, 0),
    quat,
    rospy.Time.now(),
    "base_link",
    "map"
)
```

## 8. 성능 최적화 요소

### 8.1 속도 프로필 최적화
- **직선 구간**: 최대 25 km/h로 시간 단축
- **회전 구간**: 12 km/h로 안전성 확보
- **점진적 변화**: 급격한 속도 변화 방지

### 8.2 LFD 동적 조정
- **구간별 최적화**: 각 구간의 특성에 맞는 LFD 설정
- **안정성 vs 반응성**: 직선에서는 안정성, 회전에서는 반응성 우선

### 8.3 하이브리드 제어
- **다중 센서 활용**: GPS, IMU, 카메라의 장점 결합
- **상황 적응적**: 구간별로 다른 제어 전략 적용

## 9. 결론

이 waypoint 제어 시스템은 다음과 같은 특징을 가집니다:

1. **정밀한 경로 추종**: Pure Pursuit 알고리즘 기반
2. **적응적 제어**: 구간별 최적화된 파라미터
3. **안전성 우선**: 다중 안전 메커니즘 구현
4. **센서 융합**: GPS + IMU + 카메라의 효과적 결합
5. **실시간 모니터링**: 상세한 로깅 및 상태 발행

이러한 설계를 통해 L자형 코스에서 안전하고 효율적인 자율주행을 달성할 수 있습니다.

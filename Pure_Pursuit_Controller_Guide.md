# Pure Pursuit Controller for Road Network Navigation

자율주행 시뮬레이션을 위한 Pure Pursuit 경로 추적 알고리즘 구현

---

## 📋 목차

1. [프로젝트 개요](#프로젝트-개요)
2. [Pure Pursuit 알고리즘 원리](#pure-pursuit-알고리즘-원리)
3. [프로젝트 구조](#프로젝트-구조)
4. [설치 및 요구사항](#설치-및-요구사항)
5. [빠른 시작 가이드](#빠른-시작-가이드)
6. [상세 사용법](#상세-사용법)
7. [코드 구조 설명](#코드-구조-설명)
8. [매개변수 조정](#매개변수-조정)
9. [결과 해석](#결과-해석)
10. [문제 해결](#문제-해결)
11. [확장 가능성](#확장-가능성)

---

## 🎯 프로젝트 개요

본 프로젝트는 **도로 네트워크 JSON 데이터**를 활용하여 **Pure Pursuit 알고리즘**을 구현한 자율주행 시뮬레이션 시스템입니다.

### 🔧 주요 기능

- **📍 경로 추적**: 지정된 도로 링크를 순서대로 따라가는 경로 추적
- **🎛️ 조향 제어**: Pure Pursuit 기하학을 이용한 자동 조향
- **⚡ 속도 제어**: 도로별 제한속도를 고려한 자동 속도 조절
- **📊 실시간 모니터링**: 차량 상태 및 진행률 실시간 추적
- **🎨 시각화**: 경로와 차량 위치를 그래픽으로 표시

### 🛣️ 테스트 경로

**A2229B000001** → **A2229B000023** → **A2229B000013**
- 총 거리: **224.14m**
- 총 웨이포인트: **451개**
- 링크 수: **3개**

---

## 🧠 Pure Pursuit 알고리즘 원리

### 기본 개념

Pure Pursuit은 **lookahead point**(미리보기 점)를 이용하여 차량이 부드럽게 경로를 따라갈 수 있도록 하는 경로 추적 알고리즘입니다.

### 🔄 알고리즘 단계

1. **현재 위치 파악**: 차량의 현재 위치를 경로상에서 찾기
2. **Lookahead Point 찾기**: 현재 위치에서 일정 거리 앞의 목표점 선정
3. **조향각 계산**: 목표점으로 향하는 조향각 계산
4. **차량 제어**: 계산된 조향각으로 차량 조향

### 📐 수학적 공식

```
조향각 = arctan(2 × 축거 × sin(α) / lookahead_distance)

여기서:
- α: 차량 방향과 목표점 방향의 각도 차이
- 축거: 차량의 앞바퀴와 뒷바퀴 사이의 거리
- lookahead_distance: 미리보기 거리
```

### 🎯 장점

- **부드러운 경로 추적**: 급격한 방향 변화 방지
- **구현 간단**: 직관적이고 이해하기 쉬운 알고리즘
- **안정성**: 다양한 경로 형태에 적응 가능
- **실시간 처리**: 빠른 계산으로 실시간 제어 가능

---

## 📁 프로젝트 구조

```
catkin_ws/
├── link_set.json                          # 도로 네트워크 데이터
├── route_specific_pure_pursuit.py         # 특정 경로용 컨트롤러 (메인)
├── pure_pursuit_controller.py             # 범용 Pure Pursuit 컨트롤러
├── path_planner.py                        # 경로 계획 알고리즘
├── test_pure_pursuit.py                   # 테스트 및 시각화
├── requirements.txt                       # Python 의존성
├── route_visualization.png                # 생성된 경로 시각화
└── Pure_Pursuit_Controller_Guide.md       # 본 가이드 문서
```

### 📄 파일별 설명

| 파일명 | 용도 | 추천 사용 |
|--------|------|-----------|
| `route_specific_pure_pursuit.py` | **메인 컨트롤러** | ⭐ 시작점 |
| `pure_pursuit_controller.py` | 범용 컨트롤러 | 일반적 사용 |
| `path_planner.py` | 경로 탐색 | 경로 연구 |
| `test_pure_pursuit.py` | 테스트 도구 | 디버깅 |

---

## ⚙️ 설치 및 요구사항

### 🐍 Python 버전
- **Python 3.6 이상** 필요

### 📦 의존성 설치

```bash
# 의존성 설치
pip install -r requirements.txt

# 또는 개별 설치
pip install numpy matplotlib
```

### 📋 필요 파일 확인

```bash
# 필수 파일들이 있는지 확인
ls -la link_set.json route_specific_pure_pursuit.py
```

---

## 🚀 빠른 시작 가이드

### 1️⃣ 기본 실행

```bash
# 메인 시뮬레이션 실행
python3 route_specific_pure_pursuit.py
```

### 2️⃣ 기대 결과

```
🛣️  Route-Specific Pure Pursuit Controller Demo
============================================================
Route: A2229B000001 → A2229B000023 → A2229B000013
============================================================
✅ Loaded 3 route links
📍 Created continuous path with 451 waypoints
✅ Route loaded: 451 total waypoints

📊 Route Information:
   Total Distance: 224.14 m
   Total Waypoints: 451
   Number of Links: 3
```

### 3️⃣ 시각화 확인

실행 후 `route_visualization.png` 파일이 생성되어 경로와 차량 위치를 확인할 수 있습니다.

---

## 📖 상세 사용법

### 🔧 기본 사용법

```python
from route_specific_pure_pursuit import RouteSpecificPurePursuit

# 컨트롤러 초기화
controller = RouteSpecificPurePursuit(
    link_set_file="link_set.json",
    lookahead_distance=8.0  # 미리보기 거리 (미터)
)

# 차량 상태 (시뮬레이션에서 제공되어야 함)
vehicle_pos = [168.1, 70.2, 0.0]  # [x, y, z] 좌표
vehicle_heading = 0.0  # 방향각 (라디안)

# 제어 명령 계산
steering_angle, target_speed, debug_info = controller.update(
    vehicle_pos, 
    vehicle_heading
)

print(f"조향각: {math.degrees(steering_angle):.2f}°")
print(f"목표속도: {target_speed:.1f} m/s")
print(f"진행률: {debug_info['progress_percent']:.1f}%")
```

### 🎨 시각화 사용법

```python
# 경로 시각화
controller.visualize_route(
    vehicle_pos=vehicle_pos,    # 차량 위치 표시
    show_waypoints=True,        # 웨이포인트 표시
    save_fig=True              # 이미지 파일로 저장
)
```

### 📊 경로 정보 확인

```python
# 상세 경로 정보
route_info = controller.get_route_info()

print(f"총 거리: {route_info['total_distance']:.2f}m")
print(f"웨이포인트 수: {route_info['total_waypoints']}")

for detail in route_info['link_details']:
    print(f"링크: {detail['id']}")
    print(f"  거리: {detail['distance']:.2f}m")
    print(f"  최대속도: {detail['max_speed']}")
```

---

## 🏗️ 코드 구조 설명

### 🧩 핵심 클래스: `RouteSpecificPurePursuit`

#### 주요 메서드

| 메서드명 | 기능 | 입력 | 출력 |
|----------|------|------|------|
| `__init__()` | 컨트롤러 초기화 | JSON 파일, 설정값 | - |
| `update()` | **메인 제어 루프** | 차량 위치, 방향 | 조향각, 속도, 디버그 정보 |
| `find_lookahead_waypoint()` | 목표점 탐색 | 차량 위치 | 목표점 좌표 |
| `calculate_steering_angle()` | 조향각 계산 | 위치, 방향, 목표점 | 조향각 |
| `visualize_route()` | 경로 시각화 | 선택적 매개변수 | 그래프 출력 |

#### 🔄 제어 루프 흐름

```python
def update(self, vehicle_pos, vehicle_heading):
    # 1. 목표점 찾기
    lookahead_waypoint, waypoint_idx = self.find_lookahead_waypoint(vehicle_pos)
    
    # 2. 진행상황 업데이트
    self.current_waypoint_index = waypoint_idx
    self.update_current_link(waypoint_idx)
    
    # 3. 제어 명령 계산
    steering_angle = self.calculate_steering_angle(vehicle_pos, vehicle_heading, lookahead_waypoint)
    target_speed = self.get_current_speed_limit()
    
    # 4. 디버그 정보 생성
    debug_info = {...}
    
    return steering_angle, target_speed, debug_info
```

### 🎛️ 핵심 알고리즘

#### Pure Pursuit 조향 계산

```python
def calculate_steering_angle(self, vehicle_pos, vehicle_heading, target_waypoint):
    # 차량에서 목표점으로의 벡터
    dx = target_waypoint[0] - vehicle_pos[0]
    dy = target_waypoint[1] - vehicle_pos[1]
    
    # 목표점 방향각 계산
    alpha = math.atan2(dy, dx) - vehicle_heading
    
    # 각도 정규화 [-π, π]
    while alpha > math.pi:
        alpha -= 2 * math.pi
    while alpha < -math.pi:
        alpha += 2 * math.pi
    
    # Pure Pursuit 공식
    steering_angle = math.atan2(
        2 * self.wheelbase * math.sin(alpha), 
        self.lookahead_distance
    )
    
    # 조향각 제한
    return max(-self.max_steering_angle, 
              min(self.max_steering_angle, steering_angle))
```

---

## ⚙️ 매개변수 조정

### 🎯 주요 매개변수

| 매개변수 | 기본값 | 단위 | 설명 | 조정 효과 |
|----------|--------|------|------|-----------|
| `lookahead_distance` | 8.0 | 미터 | 미리보기 거리 | 클수록 부드러움, 작을수록 정확함 |
| `wheelbase` | 2.7 | 미터 | 차량 축거 | 차량 크기에 따라 조정 |
| `max_steering_angle` | 30° | 도 | 최대 조향각 | 차량 조향 한계 |
| `max_speed` | 20.0 | m/s | 최대 속도 | 안전 속도 제한 |

### 🔧 매개변수 조정 가이드

#### Lookahead Distance 조정

```python
# 부드러운 주행 (큰 차량, 고속)
controller = RouteSpecificPurePursuit("link_set.json", lookahead_distance=12.0)

# 정밀한 주행 (작은 차량, 저속)
controller = RouteSpecificPurePursuit("link_set.json", lookahead_distance=4.0)

# 기본 설정 (일반적)
controller = RouteSpecificPurePursuit("link_set.json", lookahead_distance=8.0)
```

#### 차량별 설정 예시

```python
# 승용차
controller.wheelbase = 2.7
controller.max_steering_angle = math.radians(35)
controller.max_speed = 25.0

# 트럭
controller.wheelbase = 4.5
controller.max_steering_angle = math.radians(25) 
controller.max_speed = 15.0

# 스포츠카
controller.wheelbase = 2.3
controller.max_steering_angle = math.radians(40)
controller.max_speed = 35.0
```

---

## 📊 결과 해석

### 🎮 시뮬레이션 출력 해석

```
Step  1:
  Position: (168.1, 70.2)      # 차량 현재 위치 (x, y)
  Steering: +23.84°            # 조향각 (+ 우회전, - 좌회전)
  Speed: 20.0 m/s             # 목표 속도
  Progress: 3.8%              # 전체 경로 진행률
  Current Link: A2229B000001  # 현재 주행 중인 도로 링크
  Waypoint: 17/451            # 현재 목표 웨이포인트 번호
```

### 📈 성능 지표

#### 좋은 성능 지표
- **조향각 변화**: 부드럽게 변화 (급격한 변화 없음)
- **진행률**: 꾸준히 증가
- **속도**: 안정적 유지
- **웨이포인트**: 순차적 진행

#### 문제 지표
- **조향각**: 급격한 변화 또는 최대값 지속
- **진행률**: 정체 또는 역행
- **웨이포인트**: 건너뛰기 또는 역행

### 🎨 시각화 해석

#### 경로 시각화 요소
- **파란색 선**: 계획된 경로
- **초록색 점**: 시작점
- **빨간색 X**: 종료점
- **보라색 삼각형**: 현재 차량 위치
- **점선 원**: Lookahead 거리

---

## 🔧 문제 해결

### ❗ 일반적인 문제들

#### 1. 파일을 찾을 수 없음
```
❌ Error loading route data: [Errno 2] No such file or directory: 'link_set.json'
```
**해결책**:
```bash
# 현재 디렉토리 확인
ls -la link_set.json

# 올바른 디렉토리로 이동
cd /path/to/your/project
```

#### 2. 의존성 모듈 없음
```
❌ ModuleNotFoundError: No module named 'matplotlib'
```
**해결책**:
```bash
pip install matplotlib numpy
```

#### 3. 경로를 찾을 수 없음
```
❌ No lookahead waypoint found
```
**해결책**:
- Lookahead distance 줄이기
- 차량 위치가 경로 근처에 있는지 확인
- JSON 파일의 경로 데이터 검증

#### 4. 조향각이 너무 큼
```
Steering: +30.00° (지속됨)
```
**해결책**:
```python
# Lookahead distance 증가
controller = RouteSpecificPurePursuit("link_set.json", lookahead_distance=12.0)

# 또는 최대 조향각 제한
controller.max_steering_angle = math.radians(20)
```

### 🔍 디버깅 팁

#### 상세 디버그 정보 출력
```python
steering_angle, target_speed, debug_info = controller.update(vehicle_pos, vehicle_heading)

print("=== 디버그 정보 ===")
for key, value in debug_info.items():
    print(f"{key}: {value}")
```

#### 단계별 실행
```python
# 1단계: 경로 로드 확인
print(f"로드된 링크 수: {len(controller.route_data)}")
print(f"총 웨이포인트 수: {len(controller.waypoints)}")

# 2단계: 목표점 찾기 테스트
lookahead_point, idx = controller.find_lookahead_waypoint(vehicle_pos)
print(f"목표점: {lookahead_point}")
print(f"목표점 인덱스: {idx}")

# 3단계: 조향각 계산 테스트
if lookahead_point:
    angle = controller.calculate_steering_angle(vehicle_pos, vehicle_heading, lookahead_point)
    print(f"조향각: {math.degrees(angle):.2f}°")
```

---

## 🚀 확장 가능성

### 🔮 개선 방향

#### 1. 고급 제어 알고리즘
- **Model Predictive Control (MPC)** 통합
- **Stanley Controller** 대안 구현
- **Adaptive Lookahead** 거리 조정

#### 2. 실제 차량 통합
- **ROS (Robot Operating System)** 연동
- **CAN 통신** 인터페이스
- **실시간 센서 데이터** 처리

#### 3. 다중 경로 지원
```python
# 여러 경로 중 선택
routes = [
    ['A2229B000001', 'A2229B000023', 'A2229B000013'],  # 경로 1
    ['A2229B000002', 'A2229B000024', 'A2229B000014'],  # 경로 2
]

controller = MultiRoutePurePursuit(routes, current_route=0)
```

#### 4. 동적 장애물 회피
```python
# 장애물 정보 입력
obstacles = [{'pos': [x, y], 'radius': r}]
steering_angle, speed = controller.update_with_obstacles(
    vehicle_pos, vehicle_heading, obstacles
)
```

### 🔧 커스터마이징 예시

#### 사용자 정의 차량 모델
```python
class CustomVehicle(RouteSpecificPurePursuit):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        
        # 커스텀 차량 파라미터
        self.wheelbase = 3.2        # 대형 SUV
        self.max_steering_angle = math.radians(25)
        self.max_speed = 30.0
        
        # 추가 기능
        self.comfort_mode = True
        
    def calculate_steering_angle(self, *args):
        angle = super().calculate_steering_angle(*args)
        
        # 컴포트 모드: 부드러운 조향
        if self.comfort_mode:
            angle *= 0.8
            
        return angle
```

#### 성능 모니터링 추가
```python
class MonitoredPurePursuit(RouteSpecificPurePursuit):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.performance_log = []
        
    def update(self, *args):
        result = super().update(*args)
        
        # 성능 데이터 기록
        self.performance_log.append({
            'timestamp': time.time(),
            'steering_angle': result[0],
            'speed': result[1],
            'progress': result[2]['progress_percent']
        })
        
        return result
        
    def save_performance_log(self, filename):
        import json
        with open(filename, 'w') as f:
            json.dump(self.performance_log, f, indent=2)
```

---

## 📚 참고 자료

### 📖 알고리즘 참조
- **Pure Pursuit Algorithm**: Coulter, R. Craig. "Implementation of the pure pursuit path tracking algorithm." (1992)
- **Autonomous Vehicle Control**: Rajamani, Rajesh. "Vehicle dynamics and control." (2011)

### 🔗 관련 프로젝트
- **AtsushiSakai/PythonRobotics**: Pure Pursuit 참조 구현
- **CARLA Simulator**: 자율주행 시뮬레이션 환경
- **Apollo Auto**: 오픈소스 자율주행 플랫폼

### 💻 추가 학습 자료
- **ROS Navigation Stack**: 실제 로봇 네비게이션
- **MATLAB Automated Driving Toolbox**: 자율주행 알고리즘 시뮬레이션
- **Udacity Self-Driving Car Nanodegree**: 자율주행 온라인 강의

---

## 📞 지원 및 기여

### 🐛 버그 리포트
문제 발생 시 다음 정보와 함께 리포트해주세요:
- Python 버전
- 운영체제
- 오류 메시지
- 재현 단계

### 💡 기능 제안
새로운 기능 아이디어나 개선사항이 있으시면 언제든 제안해주세요!

### 🤝 기여 방법
1. 코드 개선
2. 문서 보완
3. 테스트 케이스 추가
4. 예제 시나리오 작성

---

**🎉 축하합니다! Pure Pursuit Controller를 성공적으로 구현하였습니다!**

이 시스템을 통해 자율주행의 기본 원리를 이해하고, 실제 차량 제어 시스템의 기초를 다질 수 있습니다. 추가 질문이나 개선사항이 있으시면 언제든 문의해주세요.

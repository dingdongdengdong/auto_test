## Lane Detection (faster) 사용 가이드

이 문서는 `lane_detection_faster.launch`로 구동되는 빠른 차선 인식 노드(`line_detector_faster.py`)의 사용법과 내부 처리 파이프라인을 설명합니다.

### 실행

```bash
roslaunch lane_detection lane_detection_faster.launch
```

- 노드 파일: `lane_detection/src/line_detector_faster.py`
- 런치 파일: `lane_detection/launch/lane_detection_faster.launch`

### 구독/발행 토픽

- 구독
  - `/image_jpeg/compressed` (sensor_msgs/CompressedImage): 카메라 영상 입력
  - `/gps` (morai_msgs/GPSMessage): 위치 정보 (실행 조건 판단에 사용)
  - `/imu` (sensor_msgs/Imu): 자세 정보 → yaw 계산
  - `/waypoint` (std_msgs/Int64): 전역 웨이포인트 인덱스
  - `/mission` (std_msgs/Bool): 미션 on/off 신호

- 발행
  - `/cam_steer` (morai_msgs/CtrlCmd): Morai 제어 명령(velocity/steering/accel)

### 처리 파이프라인 개요

1. CompressedImage → OpenCV BGR 영상 변환
2. BGR → HSV 변환 후 흰색 차선 마스크 추출
3. 가우시안 블러 + 이진화로 노이즈 저감
4. 원근 변환(`warper.py`)으로 탑뷰(버드뷰) 생성
5. 슬라이딩 윈도우(`slide_window_faster.py`)로 차선 중심 x 좌표 계산 및 현재 추종 차선(LEFT/RIGHT/MID) 판별
6. 색상 마스크 + HoughLinesP로 대표 선분 각도(커브 방향) 추정
7. 중앙 오차와 각도, yaw를 이용해 속도/조향값 산출 후 `/cam_steer` 발행

### 슬라이딩 윈도우의 역할

원근 변환된 이진 차선 이미지에서 유효 픽셀(non-zero)을 수평 창(window)으로 단계적으로 추적하여 현재 프레임의 차선 위치와 차량의 도로 중앙 기준 x 좌표를 안정적으로 추정합니다.

- 초기 탐색과 추종 방향 결정
  - 좌/우 시작 영역에서 유효 픽셀 수를 비교하여 `LEFT` 또는 `RIGHT`를 선택합니다.
  - 급좌회전 상황에서 yaw가 작게(예: `yaw < -40`) 나오고 우측 픽셀이 충분하면 `RIGHT`를 우선 선택합니다.

- 창 이동과 중심 갱신
  - 현재 창의 x 주변(`margin=20`)에 있는 유효 픽셀들의 평균으로 다음 창의 x를 갱신합니다.
  - 유효 픽셀이 부족하면 과거 누적 인덱스로 다항식 보간 또는 이전 프레임의 값(`x_previous`)을 사용해 급격한 점프를 방지합니다.

- 도로 중앙 x 좌표 계산(`x_location`)
  - 특정 y 범위(약 `y=340` 부근)에서의 차선 위치를 기준으로 도로 폭 비율을 적용해 중앙 x를 추정합니다.
    - `LEFT` 추종: `x_location = x_current + int(width*0.24) (+ 소오프셋)`
    - `RIGHT` 추종: `x_location = x_current - int(width*0.24)`
  - 검출 실패 시 `x_previous`를 사용하여 연속성을 유지합니다.

- 출력과 제어 연계
  - 반환값: 시각화 이미지(`out_img`), 중앙 x 좌표(`x_location`), 추종 상태(`current_line`).
  - `x_location`은 메인 노드에서 `error_lane = 320 - x_location` 계산에 사용되어 속도/조향을 결정합니다.

- 핵심 파라미터(기본값)
  - `window_height=15`, `nwindows=15`, `margin=20`, `minpix=0`
  - 시작 영역(픽셀): 좌(120~260, y=380~465), 우(380~540, y=380~465)

- 장점/유의점
  - 장점: 노이즈/부분 단절 환경에서도 점진적 추적으로 견고, 계산량이 전체 스캔 대비 적음.
  - 유의: 도로 폭·카메라 시점 변화 시 시작 영역/마진/원근변환 파라미터를 함께 조정해야 안정적입니다.

### 주요 파라미터/상수

- HSV 차선(흰색) 범위: `lower_lane=[0, 0, 126]`, `upper_lane=[255, 64, 180]`
- 원근 변환 기준점(`warper.py`)
  - src: `[ [0,450], [160,300], [480,300], [640,450] ]`
  - dst: `[ [160,h], [160,300], [480,300], [480,h] ]` where `h=480`
- 슬라이딩 윈도우(`slide_window_faster.py`)
  - 윈도우 높이/개수: `window_height=15`, `nwindows=15`
  - 초기 탐색 영역: 좌(120~260, y=380~465), 우(380~540, y=380~465)
  - 마진/최소픽셀: `margin=20`, `minpix=0`

### 제어 로직(요약)

- 화면 중앙(320px) 대비 차선 중심 x(`slide_x_location`) 오차 `error_lane=320 - x` 계산
- 중앙 벗어남(270 미만 또는 370 초과): 감속(`velocity=3`) + 큰 조향(`steer=error*0.003`)
- 중앙 근처: 가속(`velocity=5`) + 작은 조향(`steer=error*0.001`)
- 좌회전(yaw < -25)이고 선분 각도가 유효(|angle| ≤ 75): 조향을 각도 기반으로 보강

### 의존성

- ROS Python: `rospy`, `cv_bridge`, `sensor_msgs`, `std_msgs`, `nav_msgs`, `tf`
- Morai 메시지: `morai_msgs`
- Vision/수치: `opencv-python`, `numpy`
- 시각화(옵션): `matplotlib`, 보간(옵션): `scipy` (`slide_window_faster.py`에서 임포트)

### 튜닝 포인트

- 차선 색상 환경이 다르면 HSV 범위를 조정하세요.
- 카메라 높이/각도가 다르면 `warper.py`의 `src/dst` 포인트를 보정하세요.
- 도로 폭/시야가 다르면 슬라이딩 윈도우의 초기 탐색 영역과 `margin`을 조정하세요.

### 트러블슈팅

- 영상 미수신: `/image_jpeg/compressed`가 실제 퍼블리시되는지 확인 (`rostopic echo`, `rqt_image_view`).
- 메시지 임포트 경고: 워크스페이스 `source devel/setup.bash`(또는 `install/setup.bash`) 후 실행.
- 조향이 급격: `steer` 게인(0.003/0.001)과 속도를 상황에 맞게 낮춰 테스트.
- 탑뷰 왜곡: `warper.py`의 `src/dst` 포인트 시각적 점검 후 조정.

### 참고 파일

- `lane_detection/src/line_detector_faster.py`: 메인 노드
- `lane_detection/src/slide_window_faster.py`: 슬라이딩 윈도우/커브 추정
- `lane_detection/src/warper.py`: 원근 변환
- `lane_detection/src/sensor_parms.json`: 센서 파라미터(본 노드에서 직접 사용하진 않지만 시스템 캘리브레이션 참고)



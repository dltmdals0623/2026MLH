# 2026MLH

**IROC(국제로봇올림피아드위원회) `Multi-legged Hurdle` 종목 출전용 다족 로봇 자율주행 소프트웨어**

카메라 영상을 실시간으로 분석해 주행 코스(차선)를 인식하고, 장애물을 회피하며 목표 지점까지 자율 주행하는 4족 보행 로봇의 제어 코드입니다. 영상 인식(라인/장애물 검출) → 조향각 계산 → 시리얼 패킷 전송의 파이프라인으로 동작합니다.

---

## 1. 종목 개요

- **종목명**: Multi-legged Hurdle
- **참가 구분**: 주니어 / 시니어
- **인원 규정**: 1~2인, 팀
- **제작 방법**: 사전 제작 (대회 당일 하드웨어 제작 시간 없음)
- **미션**: 3족 이상의 다족 로봇이 주어진 주행 코스(통로/구간)를 카메라 기반 자율 판단으로 주행하며, 코스 위 장애물을 회피하여 도착지에 최대한 빠르고 멀리 도달

### 대회 주요 규정

- **로봇 제어**: 제어 프로그램은 로봇에 임베디드 되어야 하며, 실제 경기 중 노트북/PC 등 외부 장치를 통한 판단·제어·원격 개입은 금지됩니다.
- **로봇 통신**: 카메라·센서 통신은 로봇 내부 컴퓨팅 장치와의 통신으로 한정되며, 외부 장치와의 직접 통신(Wi-Fi 등)은 금지됩니다.
- **로봇 규격**: 로봇은 다족 보행 방식이어야 하며, 발바닥 대각선 `15cm` 이내 / 로봇 크기 `30×30×30cm` 이내 규격을 따라야 합니다.
- **경기장 규격**: 최대 `320×240cm(오차 ±10%)`, 필드는 흰색 바탕의 패트지 재질이며 미션맵은 경기 당일 현장에서 공개됩니다.
- **장애물 규격**: `6×6×6cm(오차 ±10%)`, 색상은 대회 당일 랜덤 지정됩니다.
- **제한 시간**: 미션별 최대 2분 이내.
- **경기 방식**: 총 2회 기록 경기이며, 두 시기 중 더 좋은 기록을 최종 기록으로 인정합니다.
- **실격 사유**: 심판 허가 없는 로봇 터치, 경기 중 부품 교체·수리, 배정된 경기장 미준수 등.

## 2. 시스템 구조

```bash
2026MLH/
├── MLH.py           # 메인 실행 파일 (카메라 캡처 → 인식 → 조향각 계산 → 시리얼 송신)
├── lane.py          # 컨투어 기반 차선(주행 코스) 검출 및 조향각(theta) 계산
├── ik.py            # 4족 로봇 역운동학(IK) 및 cmd_vel 기반 다리 오프셋 계산
├── packet.py        # 모터 제어 보드와의 시리얼(UART) 패킷 송수신 프로토콜
├── param.py         # HSV 색상 범위, 마스킹 한계값, 다리 링크 길이 등 파라미터
├── colorpicker.py   # 라인/장애물 인식용 HSV 값 캘리브레이션 도구
├── csicam/          # Rpi5용 CSI 카메라 캡처 모듈 (CsiCamCapture)
│    ├── __init__.py
│    ├── csi_camera_example.py
│    └── CsiCamCapture.py
├── tracker/         # 추적/검출 등 영상 처리 유틸리티 (made by sonminjun0302)
│    ├── __init__.py
│    ├── calc_utils.py
│    ├── contour.py
│    └── rectrangle.py
├── hsv_sav.pkl      # colorpicker.py로 저장한 HSV 캘리브레이션 값
└── .gitignore
```

### 동작 흐름 (MLH.py)

1. **카메라 입력**: Windows에서는 `cv2.VideoCapture`, Rpi5에서는 `csicam.CsiCamCapture`로 프레임을 읽습니다.
2. **전처리 및 색상 검출**: 가우시안 블러 → HSV 변환 후, `param.py`에 정의된 `MIN/MAX_LINE_HSV`, `MIN/MAX_OBJ_HSV` 범위로 차선과 장애물을 각각 마스킹합니다.
3. **컨투어 추출**: `tracker.extract_contours`로 차선/장애물 영역의 컨투어를 추출합니다.
4. **차선 검출 및 중심점 계산**: `lane.extract_lines` → `lane.select_main_lanes`가 카메라 중심을 기준으로 좌/우 차선을 선별하고, `lane.vsl_mean`으로 두 차선의 중점(목표 지점)을 계산합니다.
5. **조향각(theta) 계산**: `lane.get_theta`로 카메라 하단 기준점과 목표 지점 간 각도를 계산합니다.
6. **오프셋 변환 및 전송**: 계산된 theta를 기반으로 조향 오프셋을 산출해 `packet.serial_raw`로 모터 제어 보드에 전송합니다.
7. **프레임 화면 출력**: 매 프레임 결과 창을 통해 시각적으로 디버깅할 수 있으며, `q` 키로 종료합니다.

### 역기구학 모듈(미사용) (ik.py)

- `calculate_ik_3dof(x, y, z)`: 다리 1개(Coxa–Femur–Tibia, 3자유도)의 목표 좌표에 대한 관절 각도를 계산합니다. 링크 길이는 `param.py`의 `L1`, `L2`, `L3` 값을 사용합니다.
- `cal_leg_offset`: IK 계산 결과(라디안 각도)를 모터 오프셋 값(512 중심, 0~1024 범위)으로 변환합니다.
- `calculate_cmd_vel_offsets`: 선속도(v_x, v_y) 및 각속도(w_z, 회전)를 입력받아 4개 다리 각각이 지지 구간(Stance Phase) 동안 이동해야 할 목표 오프셋을 계산합니다.
- `cal_offset_with_cmd_vel`: 위 계산 결과를 다리별 모터 ID(Coxa/Femur/Tibia)에 매핑하여 `packet.py`로 전송 가능한 `(모터 ID, 값)` 리스트로 변환합니다.

### 시리얼 통신 프로토콜 (packet.py)

- Windows에서는 `COM4`, 그 외에서는 `/dev/ttyUSB0` 포트를 57600bps로 사용합니다.
- `serial_raw(val)`: 16비트 값을 `[255, 85, low, 255-low, high, 255-high]` 형식의 6바이트 패킷으로 전송합니다.
- `serial_write(key, data)`: 모터 ID(0~31)와 목표값(0~2046)을 하나의 패킷 값으로 인코딩합니다.
- `packet_write(*args)`: ACK(`65535`) 기반 핸드셰이크로 여러 모터 값을 순차 전송하며, 타임아웃 시 실패 코드를 반환합니다.
- 백그라운드 스레드(`tx_queue`)를 통해 논블로킹 방식으로 패킷을 전송할 수 있습니다.
- 시리얼 포트 연결이 없으면 `ser_enabled=False` 상태로 동작하며, `retry_serial()`로 재연결을 시도합니다.

## 3. 실행 환경

- Python 3.13.3
- 필수 패키지: [`opencv-python`](https://opencv.org), [`numpy`](https://numpy.org), [`pyserial`](https://github.com/pyserial/pyserial)
- Rpi5 + CsiCam 사용 시 [`csicam`](https://github.com/sonminjun0302/CsiCamCapture) 모듈 및 관련 GStreamer 환경 필요

```bash
pip install opencv-python numpy pyserial
```

## 4. 사용 방법

### 4.1 HSV 색상 캘리브레이션

실제 경기장 조명/바닥 색상에 맞춰 차선(라인)과 장애물 인식 색상 범위를 먼저 보정합니다.

```bash
python colorpicker.py
```

- 영상 위를 클릭하며 원하는 색상 영역을 드래그하듯 여러 번 클릭하면 최소/최대 HSV 범위가 갱신됩니다.
- `l` 키: 현재 선택 범위를 라인(차선) HSV로 저장
- `o` 키: 현재 선택 범위를 장애물(Object) HSV로 저장
- `r` 키: 선택 초기화
- `q` 키: 종료
- 저장된 값은 `hsv_sav.pkl`에 기록되며, 다음 실행 시 `param.py`가 이를 불러와 기존 값과 비교 후 적용 여부를 선택할 수 있습니다.

### 4.2 자율 주행 실행

```bash
python MLH.py
```

- 카메라가 인식되면 `PRESS ENTER 2 START` 안내에 따라 Enter를 눌러 주행을 시작합니다.
- 실행 중 터미널에 실시간 조향각(THETA)과 전송 오프셋 값이 출력됩니다.
- `q` 키를 누르면 종료됩니다.

## 5. 참고

- 대회 규정: [Multi-legged Hurdle 종목 규정집](https://docs.google.com/document/d/1pdP1lD6wMBAFU6S_4TWf2BjD4LqxvYibxoKeYqnFiF0/edit?tab=t.0)
- 저장소: [dltmdals0623/2026MLH](https://github.com/dltmdals0623/2026MLH)

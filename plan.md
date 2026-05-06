# MoveLite 구현 계획

## 프로젝트 개요

ESP32-WROOM 기반 2륜 차동 구동 로봇이 소규모 환경을 자율적으로 맵핑하고, 웹 UI에서 목표 지점을 지정하면 스스로 이동하는 시스템.

---

## 하드웨어 구성

| 컴포넌트 | 사양 |
|---|---|
| MCU 보드 | MKS-ESP32-FOC V1.0 (ESP32-WROOM, PSRAM 없음) |
| IMU | MPU6050 (I2C) |
| 휠 엔코더 | AS5600 x2 (자기식, I2C) |
| 거리 센서 | 1D ToF x2 (전방/후방) |
| 모터 | BLDC x2 (SimpleFOC, 전압 모드) |

### MKS-ESP32-FOC V1.0 핀 배치

V1.0 레퍼런스 코드(`4_close_loop_velocity_example.ino`)에서 확인한 실제 핀:

```
// Motor 0 (M0) — 좌 또는 우 휠
BLDCDriver3PWM driver0 = BLDCDriver3PWM(32, 33, 25, 22);  // UH, VH, WH, Enable

// Motor 1 (M1) — 반대쪽 휠
BLDCDriver3PWM driver1 = BLDCDriver3PWM(26, 27, 14, 12);  // UH, VH, WH, Enable

// AS5600 엔코더 (듀얼 I2C 버스)
I2C Bus 0 (Motor 0): SDA=19, SCL=18   → TwoWire(0)
I2C Bus 1 (Motor 1): SDA=23, SCL=5    → TwoWire(1)
```

> ⚠️ **V1.0은 인라인 전류 센싱 없음** → `TorqueControlType::voltage` 모드 사용 (SimpleFOC)

### 추가 센서 I2C 연결

V1.0은 보드에 I2C 버스 2개가 AS5600용으로 점유됨.  
MPU6050과 ToF 센서는 **Bus 0에 병렬 추가** (주소 충돌 없음):
- MPU6050: 0x68 (AD0=GND) — Bus 0 (Wire)
- ToF 전방 (VL53L0X): 0x29 → 부팅 시 0x30으로 재할당 (XSHUT 핀 필요)
- ToF 후방 (VL53L0X): 0x29 → 부팅 시 0x31으로 재할당 (XSHUT 핀 필요)

**단, AS5600과 같은 버스 사용 시 I2C 속도를 400kHz로 통일해야 함.**

---

## 소프트웨어 아키텍처

### ESP32 듀얼 코어 분리

```
Core 0 (프로토콜 태스크)
├── WiFi / TCP 스택
├── HTTP 서버 (웹 UI 서빙 — LittleFS)
└── WebSocket 서버 (맵/상태 스트리밍, 커맨드 수신)

Core 1 (실시간 제어 태스크, 높은 우선순위)
├── SimpleFOC loopFOC() + move()   — 최대 주파수
├── 오도메트리 업데이트             — 100Hz
├── MPU6050 Yaw 융합                — 100Hz
├── ToF 센서 폴링                   — 20Hz
├── Occupancy Grid 업데이트         — 20Hz
└── 경로 추종 컨트롤러              — 20Hz
```

### 로봇 상태 머신

```
IDLE
 └─[start_mapping]→ MAPPING (수동 조종으로 환경 탐색)
                       └─[save_map]→ MAP_READY
                                       └─[set_goal]→ NAVIGATING
                                                       ├─[arrived]→ MAP_READY
                                                       └─[obstacle]→ REPLANNING → NAVIGATING
```

### 소프트웨어 모듈 구조

```
src/
├── main.cpp
├── motor/
│   ├── foc_motor.h/.cpp           # SimpleFOC 래퍼 (V1.0 전압 모드)
│   └── differential_drive.h/.cpp  # (v, ω) ↔ (ωL, ωR) 변환
├── sensor/
│   ├── imu.h/.cpp                 # MPU6050 + 상보 필터 (Yaw)
│   └── tof.h/.cpp                 # VL53L0X 전방/후방 (주소 재할당)
├── localization/
│   └── odometry.h/.cpp            # 차동 구동 오도메트리 (x, y, θ)
├── mapping/
│   └── occupancy_grid.h/.cpp      # 2D 격자 맵 + 레이캐스팅 업데이트
├── navigation/
│   ├── astar.h/.cpp               # A* 경로 계획 (인플레이션 레이어 포함)
│   └── path_follower.h/.cpp       # Pure Pursuit 경로 추종
└── web/
    ├── web_server.h/.cpp          # ESPAsyncWebServer + WebSocket
    └── data/                      # LittleFS 업로드 대상
        ├── index.html
        ├── app.js
        └── style.css
```

---

## 구현 단계

### Phase 1: 프로젝트 기반 세팅

- [ ] PlatformIO 프로젝트 초기화 (`platformio.ini`)
- [ ] 의존 라이브러리 등록 (아래 참고)
- [ ] I2C 스캔으로 전체 센서 연결 확인
- [ ] V1.0 기준 FOC 오픈루프 → 클로즈루프 단계 검증

### Phase 2: 모터 제어 & 오도메트리

- [ ] SimpleFOC 전압 모드 속도 제어 구현 (V1.0 기준 핀)
- [ ] AS5600 듀얼 버스 엔코더 읽기
- [ ] 차동 구동 오도메트리 (x, y, θ) 구현
- [ ] MPU6050 Yaw 상보 필터 융합
- [ ] 직선/원호 주행 후 오도메트리 정밀도 검증

### Phase 3: 센서 & 맵핑

- [ ] VL53L0X 전방/후방 주소 재할당 및 드라이버 구현
- [ ] Occupancy Grid 설계:
  - 해상도: **10cm/cell** (WROOM SRAM 제약 — PSRAM 없음)
  - 맵 크기: **100×100 cells = 10m×10m = 10,000 bytes**
  - 셀 값: `0`=unknown, `127`=free, `255`=occupied
- [ ] 레이캐스팅(Bresenham) 기반 맵 업데이트
- [ ] 맵핑 모드: 웹 조이스틱으로 수동 조종하며 스캔

### Phase 4: 경로 계획 & 자율 주행

- [ ] Occupancy Grid 인플레이션 레이어 (로봇 반경 팽창)
- [ ] A* 경로 계획 구현
- [ ] Pure Pursuit 경로 추종 컨트롤러
- [ ] 목표 도달 판정 및 장애물 감지 시 재계획

### Phase 5: 웹 인터페이스

- [ ] LittleFS에 웹 파일 업로드 설정
- [ ] WebSocket 프로토콜:
  ```json
  // ESP32 → 브라우저 (20Hz)
  { "type": "state", "x": 1.2, "y": 0.8, "theta": 1.57,
    "map_dirty": true, "mode": "NAVIGATING" }

  // 맵 전송 (on_demand, delta)
  { "type": "map", "w": 100, "h": 100, "data": "<base64>" }

  // 브라우저 → ESP32
  { "type": "goal", "gx": 5, "gy": 7 }          // 격자 좌표
  { "type": "cmd", "action": "start_mapping" }
  { "type": "joystick", "v": 0.3, "omega": 0.1 } // 수동 조종
  ```
- [ ] 웹 UI:
  - HTML5 Canvas: 격자 맵 + 로봇 위치/방향 + 계획 경로 렌더링
  - 클릭으로 목표 격자 지점 설정
  - 수동 조종 가상 조이스틱 (맵핑 모드)
  - 실시간 센서 수치 표시 (ToF 거리, 속도, 헤딩)

---

## 기술적 리스크 & 대응

| 리스크 | 대응 |
|---|---|
| WROOM SRAM 부족 | 10cm/cell 해상도, 100×100 맵으로 제한 (10KB) |
| 오도메트리 드리프트 | MPU6050 Yaw 융합; 장기적으로는 ToF 벽 매칭 보정 고려 |
| ToF 전/후방만으로 측면 맹점 | 맵핑 시 회전-스캔 루틴으로 보완 |
| WiFi + 실시간 루프 간섭 | `xTaskCreatePinnedToCore`로 Core 분리, FOC는 Core 1 고정 |
| V1.0 전류 피드백 없음 | 전압 모드로 시작; 과전류는 `voltage_limit`으로 보호 |
| 경로 계획 시간 (A* 100×100) | 최악 10,000 노드; ESP32에서 수십ms 이내 예상, 별도 Task 처리 |

---

## 의존성 (platformio.ini)

```ini
[env:mks_esp32foc_v1]
platform = espressif32
board = esp32dev
framework = arduino
lib_deps =
    simplefoc/Simple FOC @ ^2.3.3
    pololu/VL53L0X @ ^1.3.1
    esphome/ESPAsyncWebServer-esphome @ ^3.1.0
    AsyncTCP
    ; MPU6050: I2C 직접 구현 (레지스터 레벨, 외부 라이브러리 미사용)
monitor_speed = 115200
board_build.filesystem = littlefs
build_flags =
    -DCORE_DEBUG_LEVEL=0   ; 릴리즈 시 비활성화
```

# GUGU System Rover

STM32F405 기반의 고정밀 RTK (Real-Time Kinematic) GNSS 로버 시스템입니다. 다중 무선 통신 모듈과 센서 통합을 통해 센티미터 수준의 정밀 위치 측정이 가능합니다.

## 목차

- [주요 기능](#주요-기능)
- [하드웨어 사양](#하드웨어-사양)
- [소프트웨어 스택](#소프트웨어-스택)
- [프로젝트 구조](#프로젝트-구조)
- [보드 설정](#보드-설정)
- [빌드 및 설치](#빌드-및-설치)
- [하드웨어 연결](#하드웨어-연결)
- [모듈 설명](#모듈-설명)
- [설정 방법](#설정-방법)
- [디버깅](#디버깅)
- [라이선스](#라이선스)

---

## 주요 기능

### GPS/GNSS 시스템 (RTK 위치 측정)
- **듀얼 수신기 지원**: Base Station + Rover 구성
- **수신기 옵션**:
  - u-blox F9P (고정밀 듀얼 주파수 GNSS)
  - Unicore UM982 (멀티밴드 GNSS 수신기)
- **UBX 프로토콜 파싱** (u-blox 수신기용)
- **NTRIP 클라이언트**: 셀룰러 네트워크를 통한 보정 데이터 수신
- **수동 위치 설정**: Base Station 고정 좌표 설정
- **헤딩 계산**: Rover 상대 위치 기반 방향 계산
- **RTCM 데이터 처리**

### 무선 통신 모듈

| 모듈 | 기능 | 특징 |
|------|------|------|
| **LoRa** | 장거리 무선 통신 | P2P/LoRaWAN, SF7-12, 최대 118바이트 전송 |
| **GSM/LTE** | 셀룰러 통신 | NTRIP 지원, RTK 보정 데이터 다운로드 |
| **BLE** | 블루투스 저에너지 | AT 명령 모드, 바이패스 모드 |
| **RS485** | 시리얼 통신 | 소프트웨어 UART, 트랜시버 제어 |

### 상태 및 제어
- **3개 RGB LED**: 시스템 상태 표시 (D1, D2, D3)
- **배터리 전압 모니터링**: ADC 기반
- **GPIO 제어 신호**

---

## 하드웨어 사양

### 마이크로컨트롤러
| 항목 | 사양 |
|------|------|
| MCU | STM32F405RGT6 |
| 코어 | ARM Cortex-M4 |
| 클럭 | 168 MHz |
| Flash | 1 MB |
| SRAM | 192 KB |
| 패키지 | LQFP64 |

### 주변장치 할당

| UART | 핀 | 용도 | 보드레이트 |
|------|-----|------|-----------|
| USART1 | PA9/PA10 | LTE/GSM 모듈 | 115200 |
| USART2 | PA2/PA3 | RTK Base GPS | 115200 |
| UART4 | PA0/PA1 | RTK2/Rover GPS | 115200 |
| UART5 | PC12/PD2 | RS485 | 설정값 |
| USART3 | PB10/PB11 | LoRa 모듈 | 9600 |
| USART6 | PC6/PC7 | 디버그 출력 | 115200 |

### 기타 주변장치
- **ADC1**: 배터리 전압 모니터링 (PA6/ADC_CHANNEL_6)
- **DMA**: UART RX용 5개 채널 (순환 버퍼링)
- **GPIO**: LED, 리셋, 인터럽트 핀

---

## 소프트웨어 스택

```
┌─────────────────────────────────────────┐
│           Application Layer             │
│   (GPS App, GSM App, LoRa App, BLE App) │
├─────────────────────────────────────────┤
│            Module Layer                 │
│  (gps, gsm, lora, ble, rs485, params)   │
├─────────────────────────────────────────┤
│            Library Layer                │
│   (gps lib, log, led, parser, etc.)     │
├─────────────────────────────────────────┤
│              FreeRTOS                   │
├─────────────────────────────────────────┤
│           STM32F4xx HAL                 │
├─────────────────────────────────────────┤
│         CMSIS / Hardware                │
└─────────────────────────────────────────┘
```

### 사용 기술
- **언어**: C
- **IDE**: STM32CubeIDE
- **RTOS**: FreeRTOS LTS
- **디버깅**: SEGGER SystemView
- **HAL**: STM32F4xx Hardware Abstraction Layer

---

## 프로젝트 구조

```
gugu_system_rover/
├── Core/                          # 코어 애플리케이션
│   ├── Inc/                       # 헤더 파일
│   ├── Src/                       # 소스 파일
│   │   ├── main.c                 # 엔트리 포인트
│   │   ├── hookfunction.c         # FreeRTOS 훅 함수
│   │   ├── adc.c                  # ADC 초기화
│   │   ├── dma.c                  # DMA 설정
│   │   ├── gpio.c                 # GPIO 설정
│   │   └── usart.c                # UART 설정
│   └── Startup/                   # 스타트업 어셈블리
│
├── Drivers/                       # HAL 드라이버
│   ├── STM32F4xx_HAL_Driver/      # STM32 HAL 라이브러리
│   └── CMSIS/                     # CMSIS 라이브러리
│
├── modules/                       # 애플리케이션 모듈
│   ├── gps/                       # GPS/GNSS 모듈 (UM982, F9P)
│   ├── gsm/                       # GSM/LTE 모듈 (NTRIP 지원)
│   ├── lora/                      # LoRa 모듈
│   ├── ble/                       # BLE 모듈
│   ├── rs485/                     # RS485 인터페이스
│   └── params/                    # Flash 파라미터 저장
│
├── lib/                           # 유틸리티 라이브러리
│   ├── gps/                       # GPS 파싱 (UBX, NMEA, RTCM)
│   ├── log/                       # 로깅 유틸리티
│   ├── led/                       # LED 제어
│   ├── parser/                    # 데이터 파서
│   ├── gsm/                       # GSM 유틸리티
│   ├── ble/                       # BLE 유틸리티
│   ├── lora/                      # LoRa 드라이버
│   └── rs485/                     # RS485 유틸리티
│
├── third_party/                   # 서드파티 라이브러리
│   ├── FreeRTOS-LTS/              # 실시간 운영체제
│   └── SystemView/                # RTOS 디버깅/프로파일링
│
├── config/                        # 설정 헤더
│   ├── board_config.h             # 보드 타입 매핑
│   ├── board_type.h               # 보드 타입 선택
│   ├── FreeRTOSConfig.h           # RTOS 설정
│   └── gps_config.h               # GPS 설정
│
├── docs/                          # 문서
│   ├── gsm.md                     # GSM/NTRIP 문서
│   └── lora.md                    # LoRa 문서
│
├── gugu_system_rover.ioc          # STM32CubeMX 프로젝트
├── gugu_system_rover.launch       # 디버그 설정
├── STM32F405RGTX_FLASH.ld         # Flash 링커 스크립트
└── STM32F405RGTX_RAM.ld           # RAM 링커 스크립트
```

---

## 보드 설정

시스템은 4가지 보드 프로파일을 지원합니다:

| 보드 타입 | GPS 수신기 | 통신 모듈 | 용도 |
|----------|-----------|----------|------|
| `BOARD_TYPE_BASE_UNICORE` | UM982 | BLE + GSM | Base Station |
| `BOARD_TYPE_BASE_UBLOX` | F9P | BLE + GSM | Base Station |
| `BOARD_TYPE_ROVER_UNICORE` | UM982 | RS485 + GSM | Rover |
| `BOARD_TYPE_ROVER_UBLOX` | 듀얼 F9P | RS485 + GSM | Rover (현재 선택) |

### 보드 타입 변경

`/config/board_type.h` 파일에서 설정:

```c
// 사용할 보드 타입의 주석을 해제하세요
// #define BOARD_TYPE_BASE_UNICORE
// #define BOARD_TYPE_BASE_UBLOX
// #define BOARD_TYPE_ROVER_UNICORE
#define BOARD_TYPE_ROVER_UBLOX    // 현재 선택됨
```

---

## 빌드 및 설치

### 필수 조건
- **STM32CubeIDE** (최신 버전 권장)
- **ST-Link V2/V2.1** 프로그래머/디버거
- **USB 케이블** (ST-Link 연결용)

### 빌드 방법

1. **프로젝트 열기**
   ```
   STM32CubeIDE 실행
   → File → Open Projects from File System
   → 프로젝트 폴더 선택
   ```

2. **빌드**
   ```
   프로젝트 우클릭 → Build Project
   또는 Ctrl+B
   ```

3. **Flash 다운로드**
   ```
   프로젝트 우클릭 → Run As → STM32 Application
   또는 F11
   ```

### 빌드 설정

| 설정 | Debug | Release |
|------|-------|---------|
| 최적화 | -O0 | -O2 |
| 디버그 정보 | 포함 | 제외 |
| SystemView | 활성화 | 비활성화 |

---

## 하드웨어 연결

### 핀 배치도

```
                    STM32F405RGT6
                   ┌─────────────┐
            GPS1 ← │ PA2   PA9   │ → GSM TX
            GPS1 → │ PA3   PA10  │ ← GSM RX
            GPS2 ← │ PA0   PB10  │ → LoRa TX
            GPS2 → │ PA1   PB11  │ ← LoRa RX
         ADC_BAT ← │ PA6   PC6   │ → Debug TX
                   │      PC7   │ ← Debug RX
           RS485 ← │ PC12  PD2   │ → RS485
                   └─────────────┘
```

### LED 연결

| LED | 빨강 핀 | 녹색 핀 | 기능 |
|-----|---------|---------|------|
| D1 | LED_D1_R | LED_D1_G | GPS 상태 |
| D2 | LED_D2_R | LED_D2_G | 통신 상태 |
| D3 | LED_D3_R | LED_D3_G | 시스템 상태 |

---

## 모듈 설명

### GPS 모듈 (`modules/gps/`)

RTK 위치 측정을 위한 GPS/GNSS 수신기 제어

**주요 파일:**
- `gps_app.c` - GPS 태스크 및 명령 처리
- `gps_app.h` - GPS 모듈 인터페이스

**기능:**
- UBX/NMEA 프로토콜 파싱
- RTCM 데이터 수신 및 전달
- 위치/속도/시간 데이터 처리
- 헤딩 계산

### GSM 모듈 (`modules/gsm/`)

셀룰러 네트워크를 통한 NTRIP 연결

**주요 파일:**
- `gsm_app.c` - GSM 태스크
- `gsm_app.h` - GSM 모듈 인터페이스

**기능:**
- NTRIP 서버 연결
- RTK 보정 데이터 다운로드
- AT 명령 인터페이스
- 전원 관리

### LoRa 모듈 (`modules/lora/`)

장거리 무선 통신

**주요 파일:**
- `lora_app.c` - LoRa 태스크
- `lora_app.h` - LoRa 모듈 인터페이스

**기능:**
- P2P 및 LoRaWAN 모드
- Base/Rover 역할 전환
- RTCM 프래그먼트 재조립
- 주파수/SF 설정

### BLE 모듈 (`modules/ble/`)

블루투스 저에너지 통신

**기능:**
- AT 명령 모드
- 바이패스 모드
- 비동기 명령 처리
- 연결 상태 추적

### RS485 모듈 (`modules/rs485/`)

산업용 시리얼 통신

**기능:**
- 소프트웨어 UART
- DE/RE 핀 제어
- Rover 통신

---

## 설정 방법

### NTRIP 서버 설정

`main.c`에서 NTRIP 서버 정보 설정:

```c
// NTRIP 설정 (main.c:107-111)
#define NTRIP_HOST      "your.ntrip.server.com"
#define NTRIP_PORT      2101
#define NTRIP_MOUNTPOINT "RTCM3_MOUNT"
#define NTRIP_USER      "username"
#define NTRIP_PASSWORD  "password"
```

### FreeRTOS 설정

`/config/FreeRTOSConfig.h`에서 주요 설정:

```c
#define configTOTAL_HEAP_SIZE          ((size_t)(30 * 1024))
#define configMAX_PRIORITIES           (7)
#define configMINIMAL_STACK_SIZE       ((uint16_t)128)
#define configUSE_PREEMPTION           1
```

### GPS 설정

`/config/gps_config.h`에서 GPS 관련 설정을 조정할 수 있습니다.

---

## 디버깅

### 시리얼 디버그 출력

- **포트**: USART6 (PC6/PC7)
- **보드레이트**: 115200
- **설정**: 8N1

터미널 프로그램 (PuTTY, Tera Term 등)으로 연결하여 로그 확인

### SystemView 프로파일링

SEGGER SystemView를 사용하여 RTOS 태스크 실행 분석:

1. SystemView 설치
2. J-Link 연결
3. SystemView 실행 및 연결
4. 태스크 타이밍, CPU 사용률 분석

### LED 상태 표시

| 상태 | D1 | D2 | D3 |
|------|-----|-----|-----|
| 초기화 중 | 빨강 점멸 | - | - |
| GPS Fix 없음 | 빨강 | - | - |
| GPS Fix 획득 | 녹색 | - | - |
| RTK Float | 녹색 점멸 | - | - |
| RTK Fixed | 녹색 고정 | - | - |
| 통신 활성 | - | 녹색 점멸 | - |
| 오류 | 빨강 | 빨강 | 빨강 |

---

## 태스크 구조

### FreeRTOS 태스크 생성 흐름

```
main()
  └→ vTaskStartScheduler()
       └→ initThread()
            ├→ flash_params_init()    # Flash 파라미터 초기화
            ├→ led_init()             # LED 초기화
            ├→ gps_init_all()         # GPS 수신기 초기화
            ├→ lora_instance_init()   # LoRa 모듈 초기화
            ├→ gsm_task_create()      # GSM/NTRIP 태스크 생성
            ├→ ble_init_all()         # BLE 초기화
            └→ rs485_app_init()       # RS485 초기화
```

### 태스크 우선순위

| 태스크 | 우선순위 | 스택 크기 | 설명 |
|--------|----------|-----------|------|
| GPS | High | 1024 | GPS 데이터 처리 |
| GSM/NTRIP | Medium | 2048 | 네트워크 통신 |
| LoRa | Medium | 512 | 무선 통신 |
| BLE | Low | 512 | 블루투스 통신 |
| RS485 | Low | 256 | 시리얼 통신 |

---

## 버전 정보

- **현재 버전**: V0.0.1
- **STM32CubeMX 버전**: 호환
- **FreeRTOS 버전**: LTS

---

## 문서

추가 문서는 `/docs/` 폴더를 참조하세요:

- [GSM/NTRIP 가이드](docs/gsm.md)
- [LoRa 가이드](docs/lora.md)

---

## 라이선스

이 프로젝트는 내부 사용 목적으로 개발되었습니다.

### 서드파티 라이선스
- **FreeRTOS**: MIT License
- **STM32 HAL**: BSD 3-Clause License
- **CMSIS**: Apache License 2.0
- **SEGGER SystemView**: SEGGER License

---

## 기여

버그 리포트 및 기능 요청은 이슈를 통해 제출해 주세요.

## 연락처

프로젝트 관련 문의사항이 있으시면 담당자에게 연락해 주세요.

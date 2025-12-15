# GUGU System Rover

STM32F405 기반 RTK GNSS 시스템. Base Station과 Rover 모드를 지원하며, 다중 무선 통신(LoRa, LTE, BLE)을 통해 센티미터급 정밀 측위 가능.

**버전**: V0.0.1

## 빠른 시작

### 1. 보드 타입 설정

`config/board_type.h`에서 하나만 선택:

```c
// #define BOARD_TYPE_BASE_UNICORE   // Base + UM982
// #define BOARD_TYPE_BASE_UBLOX     // Base + F9P
// #define BOARD_TYPE_ROVER_UNICORE  // Rover + UM982
#define BOARD_TYPE_ROVER_UBLOX       // Rover + 듀얼 F9P (기본값)
```

### 2. 빌드 및 업로드

1. STM32CubeIDE에서 프로젝트 열기
2. `Ctrl+B`로 빌드
3. `F11`로 플래시

## 보드 구성

| 타입 | GPS | LoRa | GSM | BLE | RS485 |
|------|-----|------|-----|-----|-------|
| BASE_UNICORE | UM982 x1 | Base | O | O | - |
| BASE_UBLOX | F9P x1 | Base | O | O | - |
| ROVER_UNICORE | UM982 x1 | Rover | O | - | O |
| ROVER_UBLOX | F9P x2 | Rover | O | - | O |

## 하드웨어

### MCU
- **STM32F405RGT6**: ARM Cortex-M4, 168MHz, 1MB Flash, 192KB SRAM

### 핀 할당

| 기능 | UART | TX | RX |
|------|------|-----|-----|
| LTE/GSM (EC25) | USART1 | PA9 | PA10 |
| GPS1 (Base) | USART2 | PA2 | PA3 |
| GPS2 (Rover) | UART4 | PA0 | PA1 |
| LoRa (RAK3172) | USART3 | PB10 | PB11 |
| RS485 | UART5 | PC12 | PD2 |
| Debug | USART6 | PC6 | PC7 |

## 초기화 흐름

```
main() → vTaskStartScheduler() → initThread()
    ├── flash_params_init()
    ├── led_init()
    ├── gps_init_all()
    │
    ├── [Base 모드]
    │   ├── gsm_task_create()    // manual position이 false일 때
    │   └── lora_instance_init()
    │
    ├── [RS485 사용 시]
    │   └── rs485_app_init()
    │
    └── [BLE 사용 시]
        └── ble_init_all()
```

## 설정

### NTRIP 설정

`main.c`에서 기본값 설정 (Flash에 저장됨):

```c
flash_params_set_ntrip_url("ntrip.hi-rtk.io");
flash_params_set_ntrip_port("2101");
flash_params_set_ntrip_mountpoint("RTK_SMT_MSG");
flash_params_set_ntrip_id("iotiz1");
flash_params_set_ntrip_pw("1234");
```

### Flash 파라미터 (`flash_params.h`)

```c
typedef struct {
    char ntrip_url[64];
    char ntrip_port[8];
    char ntrip_id[32];
    char ntrip_pw[32];
    char ntrip_mountpoint[32];

    uint32_t use_manual_position;  // Base 수동 좌표 사용 여부
    char lat[16];
    char lon[16];
    char alt[8];

    float baseline_len;            // 헤딩용 베이스라인 길이
    char ble_device_name[32];
} user_params_t;
```

## 프로젝트 구조

```
├── Core/Src/           # main.c, 주변장치 초기화
├── config/             # 보드/RTOS 설정
│   ├── board_type.h    # ★ 보드 타입 선택
│   ├── board_config.h  # 보드별 기능 매핑
│   └── FreeRTOSConfig.h
├── modules/            # 애플리케이션 모듈
│   ├── gps/            # GPS (F9P, UM982)
│   ├── gsm/            # LTE/NTRIP
│   ├── lora/           # LoRa P2P
│   ├── ble/            # BLE
│   ├── rs485/          # RS485
│   └── params/         # Flash 파라미터
├── lib/                # 저수준 라이브러리
├── third_party/        # FreeRTOS, SystemView
└── docs/               # 상세 문서
```

## 상세 문서

모듈별 상세 가이드:

- **[GSM/NTRIP 가이드](docs/gsm.md)** - LTE 초기화, TCP 통신, NTRIP 클라이언트
- **[LoRa 가이드](docs/lora.md)** - P2P 설정, RTCM 전송, API 레퍼런스

## 디버깅

### 시리얼 로그
- **포트**: USART6 (PC6/PC7)
- **설정**: 115200 8N1

### SystemView
SEGGER SystemView로 RTOS 태스크 프로파일링 가능. J-Link 연결 필요.

## FreeRTOS 설정 (실제값)

```c
#define configTOTAL_HEAP_SIZE      ((size_t)80 * 1024)
#define configMAX_PRIORITIES       (56)
#define configMINIMAL_STACK_SIZE   ((uint16_t)512)
#define configTICK_RATE_HZ         ((TickType_t)1000)
```

## 라이선스

- **FreeRTOS**: MIT
- **STM32 HAL**: BSD 3-Clause
- **CMSIS**: Apache 2.0
- **SystemView**: SEGGER License

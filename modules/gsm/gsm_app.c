#include "gsm_app.h"
#include "FreeRTOS.h"
#include "gsm.h"
#include "gsm_port.h"
#include "led.h"
#include "lte_init.h"
#include "ntrip_app.h"
#include "timers.h"
#include <string.h>

#define TAG "GSM"

#include "log.h"

void gsm_socket_monitor_start(void);

char gsm_mem[2048];

gsm_t gsm_handle;
QueueHandle_t gsm_queue;
static bool gsm_task_created = false;

void gsm_socket_monitor_stop(void);
void gsm_socket_update_recv_time(uint8_t connect_id);

static void gsm_process_task(void *pvParameter);
static void gsm_at_cmd_process_task(void *pvParameters);

/**
 * @brief GSM 태스크 생성
 *
 * @param arg
 */
void gsm_task_create(void *arg) {
  if (!gsm_task_created) {
    xTaskCreate(gsm_process_task, "gsm", 1536, arg, tskIDLE_PRIORITY + 1, NULL);
    gsm_task_created = true;
  }
}

void gsm_start_rover(void) {

  LOG_INFO("Rover 모드 LTE 시작");

  // GSM 태스크 생성 (처음 호출 시만 생성)
  if(!gsm_task_created)
  {
    gsm_task_create(NULL);
  }
  else
  {
    gsm_port_power_on();
  }

  LOG_INFO("LTE 전원 ON 완료, RDY 대기 중...");
}

static TaskHandle_t ntrip_task_handle = NULL;
static bool ntrip_should_restart = false;
static bool is_first_boot = true; // 첫 부팅 플래그

static void gsm_evt_handler(gsm_evt_t evt, void *args) {
  switch (evt) {
  case GSM_EVT_RDY: {
    LOG_INFO("RDY 수신");

    // 첫 부팅 시에만 자동 초기화 (Airplane control 설정까지)
    if (is_first_boot && lte_get_init_state() == LTE_INIT_IDLE) {
      LOG_INFO("첫 부팅: LTE 기본 설정 초기화");
      lte_init_start();
      is_first_boot = false;
    }
    // 하드웨어 리셋 후에는 재초기화
    else if (lte_get_retry_count() == LTE_INIT_MAX_RETRY + 1) {
      LOG_INFO("하드웨어 리셋 후 LTE 재초기화");
      lte_init_start();
    }
    else {
      LOG_INFO("RDY 수신 완료 (초기화는 GUGUSTART 명령 대기)");
    }
    break;
  }

  case GSM_EVT_INIT_OK: {
    LOG_INFO("LTE 초기화 성공");

    // 재시작 플래그가 설정된 경우에만 자동으로 NTRIP 시작
    if (ntrip_should_restart) {
      ntrip_should_restart = false;
      ntrip_task_create(&gsm_handle);
      led_set_color(LED_ID_1, LED_COLOR_GREEN);
      LOG_INFO("NTRIP 태스크 재생성 완료");
    } else {
      LOG_INFO("LTE 초기화 완료 (NTRIP 시작은 GUGUSTART 명령 대기)");
    }
    break;
  }

  case GSM_EVT_INIT_FAIL: {
    LOG_ERR("LTE 초기화 실패");
    led_set_color(LED_ID_1, LED_COLOR_RED);
    // 여기서 재시도 로직 등 구현 가능
    break;
  }

  case GSM_EVT_TCP_CLOSED:
    uint8_t connect_id = args ? *(uint8_t *)args : 0;
    LOG_WARN("TCP 연결 종료 (connect_id=%d)", connect_id);

    // NTRIP 소켓이 닫힌 경우 LED 노란색
    if (connect_id == 0) { // NTRIP_CONNECT_ID
      led_set_color(LED_ID_1, LED_COLOR_RED);
    }
    break;

  case GSM_EVT_PDP_DEACT:
    uint8_t context_id = args ? *(uint8_t *)args : 0;
    LOG_ERR("PDP context 비활성화 (context_id=%d)", context_id);

    // LED 노란색 (네트워크 문제)
    led_set_color(LED_ID_1, LED_COLOR_YELLOW);

    // NTRIP 태스크 종료 요청
    // (태스크가 TCP closed 이벤트를 받고 자동 종료됨)
    ntrip_should_restart = true;

    // APN부터 재초기화
    lte_reinit_from_apn();
    break;

  case GSM_EVT_POWERED_DOWN:
    // lte_reset_state();
	  LOG_INFO("GSM POWERED DOWN");
	 break;

  default:
    break;
  }
}

/**
 * @brief GSM 태스크
 *
 * @param pvParameter
 */
static void gsm_process_task(void *pvParameter) {
  size_t pos = 0;
  size_t old_pos = 0;
  uint8_t dummy = 0;
  size_t total_received = 0;

  gsm_queue = xQueueCreate(10, 1);

  // 네트워크 체크 타이머 생성 (한 번만, 재사용)
  TimerHandle_t network_timer =
      xTimerCreate("lte_net_chk", pdMS_TO_TICKS(LTE_NETWORK_CHECK_INTERVAL_MS),
                   pdFALSE, // one-shot
                   NULL, lte_network_check_timer_callback);

  gsm_init(&gsm_handle, gsm_evt_handler, NULL);
  gsm_port_init();
  gsm_start();

  // LTE 초기화 모듈 설정
  lte_set_gsm_handle(&gsm_handle);
  lte_set_network_check_timer(network_timer);

  // AT 커맨드 처리 태스크 생성
  xTaskCreate(gsm_at_cmd_process_task, "gsm_at_cmd", 1536, &gsm_handle,
              tskIDLE_PRIORITY + 2, NULL);

  led_set_color(1, LED_COLOR_RED);
  led_set_state(1, true);

  while (1) {
    xQueueReceive(gsm_queue, &dummy, portMAX_DELAY);
    led_set_toggle(1);
    pos = gsm_get_rx_pos();

    if (pos != old_pos) {
      if (pos > old_pos) {
        size_t len = pos - old_pos;
        total_received = len;
        LOG_DEBUG("RX: %u bytes", len);
//        LOG_DEBUG_RAW("RAW: ", &gsm_mem[old_pos], len);
        gsm_parse_process(&gsm_handle, &gsm_mem[old_pos], pos - old_pos);
      } else {
        size_t len1 = sizeof(gsm_mem) - old_pos;
        size_t len2 = pos;
        total_received = len1 + len2;
        LOG_DEBUG("RX: %u bytes (wrapped: %u+%u)", total_received, len1, len2);

//        LOG_DEBUG_RAW("RAW: ", &gsm_mem[old_pos], len1);
        gsm_parse_process(&gsm_handle, &gsm_mem[old_pos],
                          sizeof(gsm_mem) - old_pos);
        if (pos > 0) {
//          LOG_DEBUG_RAW("RAW: ", gsm_mem, len2);

          gsm_parse_process(&gsm_handle, gsm_mem, pos);
        }
      }
      old_pos = pos;
      if (old_pos == sizeof(gsm_mem)) {
        old_pos = 0;
      }
    }
  }

  vTaskDelete(NULL);
}

static void gsm_at_cmd_process_task(void *pvParameters) {
  gsm_t *gsm = (gsm_t *)pvParameters;
  gsm_at_cmd_t at_cmd;

  while (1) {
    if (xQueueReceive(gsm->at_cmd_queue, &at_cmd, portMAX_DELAY) == pdTRUE) {
      // 1. 수신 받은 커맨드가 정상인지 확인
      if ((at_cmd.cmd >= GSM_CMD_MAX) || (at_cmd.cmd == GSM_CMD_NONE)) {
        if (at_cmd.sem != NULL) {
          xSemaphoreGive(at_cmd.sem);
          vSemaphoreDelete(at_cmd.sem);
        }

        continue;
      }

      // 2. current_cmd 설정 (스택 변수를 직접 가리킴)
      // ★ 중요: AT 명령 전송 전에 current_cmd를 먼저 설정해야 함
      //          빠른 응답 수신 시 current_cmd가 NULL이면 응답을 놓칠 수 있음
      // ★ 안전성: Producer Task는 아래 producer_sem을 받을 때까지 블로킹되므로
      //           at_cmd 스택 변수가 덮어써질 위험이 없음 → 직접 포인터 사용
      //           가능
      if (xSemaphoreTake(gsm->cmd_mutex, portMAX_DELAY) == pdTRUE) {
        gsm->current_cmd = &at_cmd;

        // ★ lwcell 방식: msg union 초기화
        memset(&gsm->current_cmd->msg, 0, sizeof(gsm->current_cmd->msg));

        xSemaphoreGive(gsm->cmd_mutex);
      }

      // 3. at 커맨드 전송
      const char *at_mode = NULL;

      switch (at_cmd.at_mode) {
      case GSM_AT_EXECUTE:
        break;

      case GSM_AT_WRITE:
        at_mode = "=";
        break;

      case GSM_AT_READ:
        at_mode = "?";
        break;

      case GSM_AT_TEST:
        at_mode = "=?";
        break;

      default:
        break;
      }

      gsm->ops->send(gsm->at_tbl[at_cmd.cmd].at_str,
                     strlen(gsm->at_tbl[at_cmd.cmd].at_str));

      if (at_mode != NULL) {
        gsm->ops->send(at_mode, strlen(at_mode));
      }

      if (at_cmd.params[0] != '\0') {
        gsm->ops->send(at_cmd.params, strlen(at_cmd.params));
      }
      gsm->ops->send("\r\n", 2);

      uint32_t timeout_ms = gsm->at_tbl[at_cmd.cmd].timeout_ms;
      if (timeout_ms == 0) {
        timeout_ms = 5000; // 기본 타임아웃 5초
      }

      TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);
      TickType_t start_tick = xTaskGetTickCount();

      TickType_t elapsed_ticks = xTaskGetTickCount() - start_tick;
      TickType_t remaining_ticks =
          (timeout_ticks > elapsed_ticks) ? (timeout_ticks - elapsed_ticks) : 0;

      if (xSemaphoreTake(gsm->producer_sem, remaining_ticks) != pdTRUE) {
        if (xSemaphoreTake(gsm->cmd_mutex, portMAX_DELAY) == pdTRUE) {
          // 현재 명령이 우리가 보낸 명령인지 확인 (race condition 방지)
          if (gsm->current_cmd == &at_cmd) {
            gsm->current_cmd = NULL;

            // ★ 타임아웃 상태 설정 (OK/ERROR 체크 가능하도록)
            gsm->status.is_ok = 0;
            gsm->status.is_err = 1;

            // Caller가 대기 중이면 깨워줌 (동기식인 경우)
            if (at_cmd.sem) {
              xSemaphoreGive(at_cmd.sem);
            }
            // 비동기식이면 콜백 실행 (에러 처리)
            else if (at_cmd.callback) {
              at_cmd.callback(gsm, at_cmd.cmd, NULL, false);
            }

            // ★ TX pbuf 해제 (타임아웃 시에도 메모리 누수 방지)
            if (at_cmd.tx_pbuf) {
              tcp_pbuf_free(at_cmd.tx_pbuf);
            }
          }
          xSemaphoreGive(gsm->cmd_mutex);
        }
      }

      // 응답 완료 (또는 타임아웃)
      // 다음 명령 처리 가능
    }
  }

  vTaskDelete(NULL);
}

// 소켓 상태 모니터링 (AT 커맨드 동작 확인 + 소켓 상태)

//=============================================================================

#define SOCKET_STATE_CHECK_INTERVAL_MS 10000 // 10초마다 상태 확인

static TimerHandle_t socket_state_timer = NULL;

static TickType_t last_recv_tick[GSM_TCP_MAX_SOCKETS] = {0};

static TickType_t last_qistate_request_tick = 0; // 요청 시간 기록

static uint32_t qistate_timeout_count = 0; // 연속 타임아웃 횟수

// 소켓 상태 문자열 변환

static const char *socket_state_to_str(uint8_t state) {

  switch (state) {

  case 0:
    return "Initial";

  case 1:
    return "Opening";

  case 2:
    return "Connected";

  case 3:
    return "Listening";

  case 4:
    return "Closing";

  default:
    return "Unknown";
  }
}

// QISTATE 응답 콜백

static void socket_state_check_callback(gsm_t *gsm, gsm_cmd_t cmd, void *msg,
                                        bool is_ok) {

  TickType_t now = xTaskGetTickCount();

  uint32_t response_time_ms =
      (now - last_qistate_request_tick) * portTICK_PERIOD_MS;

  if (!is_ok) {

    qistate_timeout_count++;

    LOG_ERR("❌ AT 커맨드 응답 실패! (연속 %lu회)", qistate_timeout_count);

    LOG_ERR("   → 데드락 또는 모뎀 응답 없음 의심");

    if (qistate_timeout_count >= 3) {

      LOG_ERR("🚨 AT 커맨드 3회 연속 실패 - 시스템 점검 필요!");
    }

    return;
  }

  // 응답 성공 - 카운터 리셋

  qistate_timeout_count = 0;

  LOG_INFO("✅ AT 응답 정상 (응답시간: %lums)", response_time_ms);

  if (!msg || cmd != GSM_CMD_QISTATE) {

    LOG_INFO("   소켓 상태: 활성 소켓 없음");

    return;
  }

  gsm_msg_t *m = (gsm_msg_t *)msg;

  LOG_INFO("   [소켓 %d] %s | %s:%d | 상태: %s",

           m->qistate.connect_id,

           m->qistate.service_type,

           m->qistate.remote_ip,

           m->qistate.remote_port,

           socket_state_to_str(m->qistate.socket_state));

  // 마지막 수신 시간 확인

  uint8_t cid = m->qistate.connect_id;

  if (cid < GSM_TCP_MAX_SOCKETS && last_recv_tick[cid] != 0) {

    uint32_t elapsed_ms = (now - last_recv_tick[cid]) * portTICK_PERIOD_MS;

    LOG_INFO("   마지막 데이터 수신: %lu초 전", elapsed_ms / 1000);

    // 경고: 30초 이상 데이터 없음

    if (elapsed_ms > 30000 && m->qistate.socket_state == 2) {

      LOG_WARN("   ⚠️ 30초 이상 데이터 수신 없음!");
    }
  }
}

// 타이머 콜백 - 소켓 상태 확인 요청

static void socket_state_timer_callback(TimerHandle_t xTimer) {

  last_qistate_request_tick = xTaskGetTickCount();

  LOG_DEBUG("📡 AT+QISTATE 요청 전송...");

  // 비동기로 상태 확인 (connect_id=0 기준)

  gsm_send_at_qistate(&gsm_handle, 1, 0, socket_state_check_callback);
}

// 소켓 상태 모니터링 시작

void gsm_socket_monitor_start(void) {

  if (socket_state_timer == NULL) {

    socket_state_timer = xTimerCreate(

        "sock_mon",

        pdMS_TO_TICKS(SOCKET_STATE_CHECK_INTERVAL_MS),

        pdTRUE, // auto-reload

        NULL,

        socket_state_timer_callback

    );
  }

  if (socket_state_timer != NULL) {

    qistate_timeout_count = 0;

    xTimerStart(socket_state_timer, 0);

    LOG_INFO("소켓 상태 모니터링 시작 (주기: %dms)",
             SOCKET_STATE_CHECK_INTERVAL_MS);
  }
}

// 소켓 상태 모니터링 중지

void gsm_socket_monitor_stop(void) {

  if (socket_state_timer != NULL) {

    xTimerStop(socket_state_timer, 0);

    LOG_INFO("소켓 상태 모니터링 중지");
  }
}

// 수신 시간 업데이트 (외부에서 호출)

void gsm_socket_update_recv_time(uint8_t connect_id) {

  if (connect_id < GSM_TCP_MAX_SOCKETS) {

    last_recv_tick[connect_id] = xTaskGetTickCount();
  }
}


void gsm_at_power_off(uint8_t mode)
{
  gsm_send_at_qpowd(&gsm_handle, mode, NULL);
}

/**
 * @brief Airplane 모드 활성화 (무선 통신 차단)
 *
 * NTRIP 태스크를 중지하고 Airplane 모드를 활성화합니다.
 * AT+QCFG="airplanecontrol",1 설정이 완료된 상태에서만 동작합니다.
 */
void gsm_airplane_mode_enable(void) {
  LOG_INFO("Airplane 모드 활성화");

  // NTRIP 태스크 중지 요청
  // TODO: ntrip_task_stop() 함수 호출 필요

  // Airplane 모드 활성화 (W_DISABLE 핀 HIGH)
  gsm_port_set_airplane_mode(1);

  led_set_color(LED_ID_1, LED_COLOR_OFF);
}

/**
 * @brief Airplane 모드 비활성화 (무선 통신 재개)
 *
 * Airplane 모드를 해제하고 네트워크 재연결 후 NTRIP 태스크를 시작합니다.
 * AT+QCFG="airplanecontrol",1 설정이 완료된 상태에서만 동작합니다.
 */
void gsm_airplane_mode_disable(void) {
  LOG_INFO("Airplane 모드 비활성화");

  // Airplane 모드 비활성화 (W_DISABLE 핀 LOW)
  gsm_port_set_airplane_mode(0);

  // 네트워크 재연결 대기 (약 1-2초)
  vTaskDelay(pdMS_TO_TICKS(2000));

  // 네트워크 상태 확인 후 NTRIP 시작
  if (lte_get_init_state() == LTE_INIT_DONE) {
    LOG_INFO("네트워크 재연결 완료, NTRIP 태스크 시작");
    ntrip_task_create(&gsm_handle);
    led_set_color(LED_ID_1, LED_COLOR_GREEN);
  } else {
    LOG_WARN("네트워크 미연결 상태");
    led_set_color(LED_ID_1, LED_COLOR_YELLOW);
  }
}
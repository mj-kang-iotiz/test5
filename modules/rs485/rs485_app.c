#include "softuart.h"
#include "rs485_app.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "flash_params.h"
#include "board_type.h"
#include "board_config.h"
#include "gps_app.h"
#include "lora_app.h"
#include "gsm_app.h"
#include "gsm.h"
#include "lte_init.h"

#ifndef TAG
#define TAG "RS485_APP"
#endif

#include "log.h"

char *INIT_Notify = "+READY\r";
char *GPS_Notify = "+GPS,\r";
char *F_Version_Response = "+V0.0.1\r";
char *AT_Response = "+OK\r";
char *ATZ_Response = "+RESET\r";
char *ATnF_Response = "+CONFIGINIT\r";
char *GPSMANUF_UM982_Response = "+Unicore\r";
char *GPSMANUF_F9P_Response = "+Ublox\r";
char *CONFIG_Response = "+CONFIG=Hello1234567890abcdefghijklmnopqrstuvwyzABCDEFGHIJKLMNOPQRSTUVWXYZ\r"; // need make CONFIG Variable
char *SETBASELINE_Response = "+SETBASELINE=\r";                                                         // need make setbaseline variable
char *CASTER_Response = "+CASTER=\r";                                                                   // need make caster variable
char *ID_Response = "+ID=\r";                                                                           // need make ID variable
char *MOUNTPOINT_Response = "+MOUNTPOINT=\r";                                                           // need make MOUNTPOINT variable
char *PASSWORD_Response = "+PASSWORD=\r";                                                               // need make PASSWORD variable
char *START_Response = "+GUGUSTART\r";
char *STOP_Response = "+GUGUSTOP\r";

char *ERROR_Response = "+ERROR\r"; // ETC
char *ERROR1_Response = "+E01\r";  // DO NOT KNOW ERROR
char *ERROR2_Response = "+E02\r";  // Parameter ERROR
char *ERROR3_Response = "+E03\r";  // NO ready device ERROR

static SemaphoreHandle_t rs485_tx_mutex;

volatile bool is_gugu_started = false;

void delay_170ns(void);

// 약 900ns 지연
void delay_170ns(void)
{
  __asm volatile(
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t");
}

void RS485_SetTransmitMode(void)
{
  HAL_GPIO_WritePin(RS485_DE_PORT, RS485_DE_PIN, GPIO_PIN_SET); // DE=1, RE=1 (송신)
  HAL_GPIO_WritePin(RS485_RE_PORT, RS485_RE_PIN, GPIO_PIN_SET); // DE=1, RE=1 (송신)
  delay_170ns();
  delay_170ns();
  delay_170ns();
  delay_170ns();
  delay_170ns();
}

void RS485_SetReceiveMode(void)
{
  delay_170ns();
  HAL_GPIO_WritePin(RS485_DE_PORT, RS485_DE_PIN, GPIO_PIN_RESET); // DE=0, RE=0 (수신)
  HAL_GPIO_WritePin(RS485_RE_PORT, RS485_RE_PIN, GPIO_PIN_RESET); // DE=0, RE=0 (수신)
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1)
  {
    SoftUartHandler();
  }
}

int get_line(uint8_t SoftUartNumber, char *buffer, int maxlen)
{
  int index = 0;
  uint8_t ch;

  while (index < maxlen - 1)
  {
    while (SoftUartRxAlavailable(SoftUartNumber) == 0)
    {
      vTaskDelay(pdMS_TO_TICKS(3));
    }
    SoftUartReadRxBuffer(SoftUartNumber, &ch, 1);

    buffer[index++] = ch;

    if (ch == '\r') // 종료 문자
      break;
  }

  buffer[index] = '\0'; // 문자열 종료
  return index;
}

void RS485_Send(uint8_t *data, uint8_t len)
{
  xSemaphoreTake(rs485_tx_mutex, portMAX_DELAY);

  RS485_SetTransmitMode();
  SoftUartPuts(0, data, len);
  SoftUartWaitUntilTxComplate(0);
  RS485_SetReceiveMode();

  xSemaphoreGive(rs485_tx_mutex);
}

volatile bool base_init_finished = false;
static void base_config_complete(bool success, void *user_data)
{
  gps_id_t id = (gps_id_t)(uintptr_t)user_data;
  LOG_INFO("GPS[%d] Fix mode init %s", id, success ? "succeeded" : "failed");

  base_init_finished = true;
}

static void send_gps_task(void *pvParameters)
{
  char buf[120];
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (1)
  {
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(2000));
    if (is_gugu_started)
    {
      gps_format_position_data(buf);
      RS485_Send((uint8_t *)buf, strlen(buf));
    }
  }
}

typedef enum
{
  RTK_ACTIVE_STATUS_NONE = 0,
  RTK_ACTIVE_STATUS_LORA,
  RTK_ACTIVE_STATUS_GSM,
}rtk_active_status_t;

static void rs485_task(void *pvParameter)
{
  char ch;
  char rx_buffer[64];
  uint8_t init_f = 0;
  char buf[64];
  TickType_t current_tick = xTaskGetTickCount();

  const board_config_t *config = board_get_config();
  user_params_t *params = flash_params_get_current();

  uint8_t dummy_buf[64];

  rtk_active_status_t active_status = RTK_ACTIVE_STATUS_NONE;

  // 시스템 부팅 후 3초 대기 (초기화 완료 대기)

  vTaskDelay(pdMS_TO_TICKS(2900));

  // RX 버퍼 완전히 비우기 (리셋 후 남아있을 수 있는 쓰레기 데이터 제거)
  while (SoftUartRxAlavailable(0) > 0)
  {
    uint8_t available = SoftUartRxAlavailable(0);
    SoftUartReadRxBuffer(0, dummy_buf, available > 64 ? 64 : available);
    vTaskDelay(pdMS_TO_TICKS(1));
  }

  // TX 경로 클리어를 위한 더미 전송 (첫 바이트 손실 방지)
  RS485_SetTransmitMode();
  vTaskDelay(pdMS_TO_TICKS(10)); // RS485 DE/RE 핀 안정화
  uint8_t dummy_tx = '\0';
  SoftUartPuts(0, &dummy_tx, 1);
  SoftUartWaitUntilTxComplate(0);
  vTaskDelay(pdMS_TO_TICKS(10)); // 전송 완료 대기
  RS485_SetReceiveMode();

  // 추가 안정화 대기
  vTaskDelay(pdMS_TO_TICKS(50));
  RS485_Send((uint8_t *)INIT_Notify, strlen(INIT_Notify));

  while (1)
  {
    int len = get_line(0, rx_buffer, sizeof(rx_buffer)); // 문자열 수신

    if (strcmp(rx_buffer, "AT\r") == 0)
    {
      RS485_Send((uint8_t *)AT_Response, strlen(AT_Response));
    }
    else if (strcmp(rx_buffer, "ATZ\r") == 0)
    {
      user_params_t *current_params = flash_params_get_current();
      // if(flash_params_save(current_params) != HAL_OK)
      // {
      //     RS485_Send((uint8_t*)ERROR_Response, strlen(ERROR_Response));
      // }
      // else
      // {
      RS485_Send((uint8_t *)ATZ_Response, strlen(ATZ_Response));
      vTaskDelay(pdMS_TO_TICKS(5));
      NVIC_SystemReset();
      // }
    }
    else if (strcmp(rx_buffer, "AT&F\r") == 0)
    {
      if (flash_params_erase() == HAL_OK)
      {
        RS485_Send((uint8_t *)ATnF_Response, strlen(ATnF_Response));
        vTaskDelay(pdMS_TO_TICKS(5));
        NVIC_SystemReset();
      }
      else
      {
        RS485_Send(ERROR_Response, strlen(ERROR_Response));
      }
    }
    else if (strcmp(rx_buffer, "AT+VER?\r") == 0)
    {
      sprintf(buf, "+%s\r", BOARD_VERSION);
      RS485_Send((uint8_t *)buf, strlen(buf));
    }
    else if (strcmp(rx_buffer, "AT+GPSMANUF?\r") == 0)
    {
      if (config->board == BOARD_TYPE_BASE_UM982 || config->board == BOARD_TYPE_ROVER_UM982)
      {
        RS485_Send((uint8_t *)GPSMANUF_UM982_Response, strlen(GPSMANUF_UM982_Response));
      }
      else if (config->board == BOARD_TYPE_BASE_F9P || config->board == BOARD_TYPE_ROVER_F9P)
      {
        RS485_Send((uint8_t *)GPSMANUF_F9P_Response, strlen(GPSMANUF_F9P_Response));
      }
    }
    else if (strcmp(rx_buffer, "AT+CONFIG?\r") == 0)
    {
      // RS485_Send((uint8_t*)CONFIG_Response, strlen(CONFIG_Response));
    }
    else if (strncmp(rx_buffer, "AT+SETBASELINE:", 15) == 0)
    {
      float baseline_value = strtof(rx_buffer + 15, NULL);

      if (baseline_value != 0)
      {
        flash_params_set_baseline_len(baseline_value);
        gps_set_heading_length();

        RS485_Send((uint8_t *)AT_Response, strlen(AT_Response));
      }
      else
      {
        RS485_Send((uint8_t *)ERROR2_Response, strlen(ERROR2_Response));
      }
    }
    else if (strncmp(rx_buffer, "AT+CASTER:", 10) == 0)
    {
      char caster_host[64];
      char caster_port[8];

      char *param = (char *)rx_buffer + 10;
      char *port_sep = strrchr(param, ':'); // 마지막 ':' 찾기
      if (port_sep)
      {
        *port_sep = '\0'; // host와 port 분리
        strncpy(caster_host, param, sizeof(caster_host) - 1);

        char *end = strchr(port_sep + 1, '\r');
        if (end)
          *end = '\0';
        strncpy(caster_port, port_sep + 1, sizeof(caster_port) - 1);
      }

      int host_len = strlen(caster_host);
      int port_len = strlen(caster_port);

      if (host_len == 0 || port_len == 0 || host_len >= 63 || port_len >= 7)
      {
        RS485_Send(ERROR2_Response, strlen(ERROR2_Response));
      }
      else
      {
        flash_params_set_ntrip_url((const char *)caster_host);
        flash_params_set_ntrip_port((const char *)caster_port);

        sprintf(buf, "+CASTER=%s:%s\r", params->ntrip_url, params->ntrip_port);
        RS485_Send((uint8_t *)buf, strlen(buf));
      }
    }
    else if (strncmp(rx_buffer, "AT+ID=", 6) == 0)
    {
      char ntrip_id[32];
      char *param = (char *)rx_buffer + 6;
      char *end = strchr(param, '\r');
      if (end)
        *end = '\0';
      strncpy(ntrip_id, param, sizeof(ntrip_id) - 1);

      int id_len = strlen(ntrip_id);
      if (id_len == 0 || id_len >= 31)
      {
        RS485_Send(ERROR2_Response, strlen(ERROR2_Response));
      }
      else
      {
        flash_params_set_ntrip_id(ntrip_id);

        sprintf(buf, "+ID=%s\r", params->ntrip_id);
        RS485_Send((uint8_t *)buf, strlen(buf));
      }
    }
    else if (strncmp(rx_buffer, "AT+MOUNTPOINT=", 14) == 0)
    {
      char mountpoint[32];
      char *param = (char *)rx_buffer + 14;
      char *end = strchr(param, '\r');
      if (end)
        *end = '\0';
      strncpy(mountpoint, param, sizeof(mountpoint) - 1);

      int mountpoint_len = strlen(mountpoint);

      if (mountpoint_len == 0 || mountpoint_len >= 31)
      {
        RS485_Send(ERROR2_Response, strlen(ERROR2_Response));
      }
      else
      {
        flash_params_set_ntrip_mountpoint(mountpoint);

        sprintf(buf, "+MOUNTPOINT=%s\r", params->ntrip_mountpoint);
        RS485_Send((uint8_t *)buf, strlen(buf));
      }
    }
    else if (strncmp(rx_buffer, "AT+PASSWORD=", 12) == 0)
    {
      char password[32];
      char *param = (char *)rx_buffer + 12;
      char *end = strchr(param, '\r');
      if (end)
        *end = '\0';
      strncpy(password, param, sizeof(password) - 1);

      int password_len = strlen(password);

      if (password_len == 0 || password_len >= 31)
      {
        RS485_Send(ERROR2_Response, strlen(ERROR2_Response));
      }
      else
      {
        flash_params_set_ntrip_pw(password);

        sprintf(buf, "+PASSWORD=%s\r", params->ntrip_pw);
        RS485_Send((uint8_t *)buf, strlen(buf));
      }
    }
    else if (strncmp(rx_buffer, "AT+GUGUSTART:", 13) == 0)
    {
      char *param = (char *)rx_buffer + 13;
      char *end = strchr(param, '\r');
      if (end)
        *end = '\0';

      char cmd[10];
      strncpy(cmd, param, sizeof(cmd) - 1);

      if (strncmp(cmd, "LTE", 3) == 0)
      {
        if(active_status == RTK_ACTIVE_STATUS_LORA)
        {
          lora_instance_deinit();
        }
        gsm_start_rover();
        RS485_Send("+GUGUSTART-LTE", strlen("+GUGUSTART-LTE"));
        active_status = RTK_ACTIVE_STATUS_GSM;
        is_gugu_started = true;
      }
      else if (strncmp(cmd, "LORA", 4) == 0)
      {
        if(active_status == RTK_ACTIVE_STATUS_GSM)
        {
          ntrip_stop();
          vTaskDelay(pdMS_TO_TICKS(100));
          gsm_port_set_airplane_mode(true);
        }
        lora_start_rover();
        RS485_Send("+GUGUSTART-LORA", strlen("+GUGUSTART-LORA"));
        active_status = RTK_ACTIVE_STATUS_LORA;
        is_gugu_started = true;
      }
      else
      {
        RS485_Send((uint8_t *)ERROR2_Response, strlen(ERROR2_Response));
      }
    }
    else if (strncmp(rx_buffer, "AT+GUGUSTOP", 11) == 0)
    {
      if(active_status == RTK_ACTIVE_STATUS_LORA)
      {
        lora_instance_deinit();
        is_gugu_started = false;
        active_status = RTK_ACTIVE_STATUS_NONE;
        RS485_Send((uint8_t *)STOP_Response, strlen(STOP_Response));
      }
      else if(active_status == RTK_ACTIVE_STATUS_GSM)
      {
        if(lte_get_init_state() == LTE_INIT_DONE)
        {
          // gsm 초기화
          is_gugu_started = false;
          ntrip_stop();
          vTaskDelay(pdMS_TO_TICKS(100));
          gsm_port_set_airplane_mode(true);
          active_status = RTK_ACTIVE_STATUS_NONE;
          RS485_Send((uint8_t *)STOP_Response, strlen(STOP_Response));
        }
        else
        {
          RS485_Send(ERROR3_Response, strlen(ERROR3_Response));
        }
      }
      else
      {
        is_gugu_started = false;
        RS485_Send((uint8_t *)STOP_Response, strlen(STOP_Response));
      }
    }
    else if (strncmp(rx_buffer, "AT+SAVE\r", 8) == 0)
    {
      user_params_t *params = flash_params_get_current();
      if (flash_params_erase() != HAL_OK)
      {
        RS485_Send(ERROR_Response, strlen(ERROR_Response));
      }
      else
      {
        if (flash_params_write(params) != HAL_OK)
        {
          RS485_Send(ERROR_Response, strlen(ERROR_Response));
        }
        else
        {
          RS485_Send((uint8_t *)"+SAVE\r", strlen("+SAVE\r"));
          NVIC_SystemReset();
        }
      }
    }
    else
    {
      RS485_Send((uint8_t *)ERROR1_Response, strlen(ERROR1_Response));
    }
  }
}

void rs485_app_init(void)
{
  rs485_tx_mutex = xSemaphoreCreateMutex();
  xTaskCreate(rs485_task, "RS485_Task", 512, NULL, tskIDLE_PRIORITY + 1, NULL);
  xTaskCreate(send_gps_task, "send_gps", 512, NULL, tskIDLE_PRIORITY + 1, NULL);
}

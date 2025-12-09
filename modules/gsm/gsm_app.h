#ifndef GSM_APP_H
#define GSM_APP_H

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

extern QueueHandle_t gsm_queue;

void gsm_task_create(void *arg);
void gsm_socket_monitor_start(void);
void gsm_start_rover(void);
void gsm_at_power_off(uint8_t mode);

/**
 * @brief Airplane 모드 활성화 (무선 통신 차단)
 */
void gsm_airplane_mode_enable(void);

/**
 * @brief Airplane 모드 비활성화 (무선 통신 재개)
 */
void gsm_airplane_mode_disable(void);

#endif

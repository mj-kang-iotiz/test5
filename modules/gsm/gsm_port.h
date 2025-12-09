#ifndef GSM_PORT_H
#define GSM_PORT_H

#include "gsm_app.h"

void gsm_dma_init(void);
void gsm_uart_init(void);
void gsm_port_comm_start(void);
void gsm_port_gpio_start(void);
uint32_t gsm_get_rx_pos(void);
void gsm_port_init(void);
void gsm_start(void);

void gsm_port_power_off(void);
int gsm_port_power_on(void);

/**
 * @brief EC25 모듈 하드웨어 리셋 (HAL ops 콜백)
 *
 * RST 핀을 이용한 하드웨어 리셋 수행
 * 리셋 후 모듈 부팅 완료까지 대기
 *
 * @return int 0: 성공
 */
int gsm_port_reset(void);

/**
 * @brief Airplane 모드 GPIO 제어
 *
 * W_DISABLE 핀을 통해 Airplane 모드를 제어합니다.
 * EC25 모듈의 AT+QCFG="airplanecontrol",1 설정 후 사용 가능
 *
 * @param enable 1: Airplane 모드 활성화 (무선 통신 차단), 0: Airplane 모드 비활성화
 */
void gsm_port_set_airplane_mode(uint8_t enable);

#endif

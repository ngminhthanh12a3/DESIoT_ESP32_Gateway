#ifndef INC_DESIOT_GATEWAY_H_
#define INC_DESIOT_GATEWAY_H_

// UART
#include <driver/uart.h>
#include <soc/uart_struct.h>
#include <soc/uart_reg.h>
#define DESIOT_UART_NUM UART_NUM_2
#define DESIOT_CIR_BUF_SIZE 1024u

void DESIoT_G_begin();
void DESIoT_G_loop();

#ifdef ESP32
static intr_handle_t handle_console;
static void IRAM_ATTR DESIoT_UART_INTR_HANDLE(void *arg);
#endif

#endif /* INC_DESIOT_GATEWAY_H_ */
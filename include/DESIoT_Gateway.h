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

// Circular buffer
typedef struct{
    uint16_t start;
    uint16_t end;
    uint8_t buffer[DESIOT_CIR_BUF_SIZE];
}DESIoT_CBUF_t;
enum DESIOT_CBUF_STATUS{
    DESIOT_CBUF_OK,
    DESIOT_CBUF_ERROR
};
uint8_t DESIoT_CBUF_getByte(DESIoT_CBUF_t *hCBuf, uint8_t *rx);
void DESIoT_CBUF_putByte(DESIoT_CBUF_t *hCBuf, uint8_t rx);
#ifdef ESP32
static intr_handle_t handle_console;
static void IRAM_ATTR DESIoT_UART_INTR_HANDLE(void *arg);
#endif

#endif /* INC_DESIOT_GATEWAY_H_ */
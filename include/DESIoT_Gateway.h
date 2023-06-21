#ifndef INC_DESIOT_GATEWAY_H_
#define INC_DESIOT_GATEWAY_H_

// UART
#include <driver/uart.h>
#include <soc/uart_struct.h>
#include <soc/uart_reg.h>
#define DESIOT_UART_NUM UART_NUM_2
#define DESIOT_CIR_BUF_SIZE 1024u

typedef struct
{
    uint8_t buf[DESIOT_CIR_BUF_SIZE];
    uint16_t start;
    uint16_t end;
} CBufer_handleTypeDef_t;

typedef enum
{
    CBUFFER_OK,
    CBUFFER_ERROR
} CBUFFER_StatusTypeDef;

void DESIoT_G_begin();
void DESIoT_G_loop();
void CBUFFER_putByte(CBufer_handleTypeDef_t *hcBuffer, uint8_t byte);
uint8_t CBUFFER_getByte(CBufer_handleTypeDef_t *hcBuffer, uint8_t *rx);

#ifdef ESP32
static intr_handle_t handle_console;
static void IRAM_ATTR DESIoT_UART_INTR_HANDLE(void *arg);
#endif

#endif /* INC_DESIOT_GATEWAY_H_ */
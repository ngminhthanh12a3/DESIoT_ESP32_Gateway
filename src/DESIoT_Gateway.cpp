#include <Arduino.h>
#include "DESIoT_Gateway.h"

void DESIoT_G_begin()
{
#ifdef ESP32
    // UART setup
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122};
    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(DESIOT_UART_NUM, &uart_config));
    // Set UART pins (using UART0 default pins ie no changes.)
    ESP_ERROR_CHECK(uart_set_pin(DESIOT_UART_NUM, 17, 16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    QueueHandle_t uart_queue;
    // Install UART driver using an event queue here
    ESP_ERROR_CHECK(uart_driver_install(DESIOT_UART_NUM, DESIOT_CIR_BUF_SIZE / 2,
                                        +DESIOT_CIR_BUF_SIZE / 2, 10, &uart_queue, 0));

    // clear the buffer
    uart_flush(DESIOT_UART_NUM);
    ESP_ERROR_CHECK(uart_isr_free(DESIOT_UART_NUM));
    ESP_ERROR_CHECK(uart_isr_register(DESIOT_UART_NUM, DESIoT_UART_INTR_HANDLE, NULL, ESP_INTR_FLAG_IRAM, &handle_console)); // register new UART subroutine
    ESP_ERROR_CHECK(uart_enable_rx_intr(DESIOT_UART_NUM));
#endif
}

void DESIoT_G_loop()
{
}

extern CBufer_handleTypeDef_t hcBuffer;
/*
 * Define UART interrupt subroutine to ackowledge interrupt
 */
static void IRAM_ATTR DESIoT_UART_INTR_HANDLE(void *arg)
{
    uint16_t rx_fifo_len, status;
    uint16_t i;
    status = UART2.int_st.val;             // read UART interrupt Status
    rx_fifo_len = UART2.status.rxfifo_cnt; // read number of bytes in UART buffer
    while (rx_fifo_len)
    {
        uint8_t uart_byte = UART2.fifo.rw_byte; // read all bytes
        CBUFFER_putByte(&hcBuffer, uart_byte);
        rx_fifo_len--;
    }
    // after reading bytes from buffer clear UART interrupt status
    uart_clear_intr_status(DESIOT_UART_NUM, UART_RXFIFO_FULL_INT_CLR | UART_RXFIFO_TOUT_INT_CLR);
}

void CBUFFER_putByte(CBufer_handleTypeDef_t *hcBuffer, uint8_t byte)
{
    hcBuffer->buf[hcBuffer->end++] = byte;
    hcBuffer->end %= DESIOT_CIR_BUF_SIZE;
}

uint8_t CBUFFER_getByte(CBufer_handleTypeDef_t *hcBuffer, uint8_t *rx)
{
    if (hcBuffer->end != hcBuffer->start)
    {
        *rx = hcBuffer->buf[hcBuffer->start++];
        hcBuffer->start %= DESIOT_CIR_BUF_SIZE;
        return CBUFFER_OK;
    }
    return CBUFFER_ERROR;
}
#ifndef INC_DESIOT_GATEWAY_H_
#define INC_DESIOT_GATEWAY_H_

// UART
#include <driver/uart.h>
#include <soc/uart_struct.h>
#include <soc/uart_reg.h>
#define DESIOT_UART_NUM UART_NUM_2
#define DESIOT_CIR_BUF_SIZE 1024u

// attributes
#define DESIOT_ATT_PACKED __attribute__((__packed__))
#define DESIOT_ATT_UNUSED __attribute__((__unused__))
#define DESIOT_ATT_WEAK __attribute__((weak))

// Lengths of packet types
#define DESIOT_H1_LEN 1u
#define DESIOT_H2_LEN 1u
#define DESIOT_HEAD_LEN (DESIOT_H1_LEN + DESIOT_H2_LEN)

#define DESIOT_T1_LEN 1u
#define DESIOT_T2_LEN 1u
#define DESIOT_TRAIL_LEN (DESIOT_T1_LEN + DESIOT_T2_LEN)

#define DESIOT_HEADERS_LEN (DESIOT_HEAD_LEN + DESIOT_TRAIL_LEN)

#define DESIOT_CRC_LEN 2u

#define DESIOT_CMD_LEN 0x1u
#define DESIOT_DATALEN_LEN 0x2u

#define DESIOT_HEAD_FRAME_LEN (DESIOT_HEAD_LEN + DESIOT_CMD_LEN + DESIOT_DATALEN_LEN)

void DESIoT_G_begin();
void DESIoT_G_loop();
void DESIoT_G_frameArbitrating();

// Circular buffer
typedef struct
{
    uint16_t start;
    uint16_t end;
    uint8_t buffer[DESIOT_CIR_BUF_SIZE];
} DESIoT_CBUF_t;
enum DESIOT_CBUF_STATUS
{
    DESIOT_CBUF_OK,
    DESIOT_CBUF_ERROR
};
uint8_t DESIoT_CBUF_getByte(DESIoT_CBUF_t *hCBuf, uint8_t *rx);
void DESIoT_CBUF_putByte(DESIoT_CBUF_t *hCBuf, uint8_t rx);

// Frame
typedef struct
{
    uint8_t cmd;
    union
    {
        uint8_t dataLenArr[2];
        uint16_t dataLen : 10;
    };
    uint8_t data[UINT16_MAX & 0x3FFu];
} DESIOT_ATT_PACKED DESIoT_dataPacket_t;

typedef struct
{
    uint8_t h1;
    uint8_t h2;
    DESIoT_dataPacket_t dataPacket;
    uint8_t t1;
    uint8_t t2;
    union
    {
        uint16_t crc;
        uint8_t crcArr[2];
    };
} DESIOT_ATT_PACKED DESIoT_Frame_t;

typedef struct
{
    uint8_t index;
    uint8_t status;
    DESIoT_Frame_t frame;
} DESIoT_Frame_Hander_t;

#define DESIOT_SET_FRAME_FAILED_STATUS(status) status--
#define DESIOT_SET_FRAME_SUCCESS_STATUS(status) status -= 2
enum DESIOT_FRAME_STATUSES
{
    DESIOT_FRAME_IDLE,
    DESIOT_FRAME_UART0_SUCCESS,
    DESIOT_FRAME_UART0_FAILED,
    DESIOT_FRAME_IN_UART0_PROGRESS
};

enum DESIOT_HEAD_FRAME_INDEXES
{
    DESIOT_H1_INDEX,
    DESIOT_H2_INDEX,
    DESIOT_CMD_INDEX,
    DESIOT_DATALEN_INDEX
};

// DESIoT default values
#define DESIOT_H1_DEFAULT 0x7u
#define DESIOT_H2_DEFAULT 0x17u
#define DESIOT_T1_DEFAULT 0x7u
#define DESIOT_T2_DEFAULT 0x17u

// CRC
#define DESIOT_CRC_GENERATOR 0x1305
void DESIoT_CalculateTable_CRC16();
uint16_t DESIoT_Compute_CRC16(uint8_t *bytes, const int32_t BYTES_LEN);

void DESIoT_FRAME_parsing(DESIoT_Frame_Hander_t *hFrame, uint8_t byte);
void DESIoT_frameFailedHandler();
void DESIoT_frameSuccessHandler();
#ifdef ESP32
static intr_handle_t handle_console;
static void IRAM_ATTR DESIoT_UART_INTR_HANDLE(void *arg);
#endif

#endif /* INC_DESIOT_GATEWAY_H_ */
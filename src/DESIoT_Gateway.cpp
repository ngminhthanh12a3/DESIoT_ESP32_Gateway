#include "DESIoT_Gateway.h"

#include "DESIoT_Gateway.h"

#ifdef ESP32
AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

unsigned long DESIoT_millis()
{
    return millis();
}
#endif

DESIoT_CBUF_t hUART2CBuffer = {.start = 0, .end = 0};
DESIoT_CBUF_t hMQTTCBuffer = {.start = 0, .end = 0};
DESIoT_Frame_Hander_t hFrame = {.index = 0};

void DESIoT_UART_begin()
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
    // Set UART pins (using UART2 default pins ie no changes.)
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
    hFrame.loopTick = DESIoT_millis();
    DESIoT_frameFailedHandler();
    DESIoT_frameSuccessHandler();
    DESIoT_frameTimeoutHandler();
    DESIoT_G_frameArbitrating();
}

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
        DESIoT_CBUF_putByte(&hUART2CBuffer, uart_byte);
        rx_fifo_len--;
    }
    // after reading bytes from buffer clear UART interrupt status
    uart_clear_intr_status(DESIOT_UART_NUM, UART_RXFIFO_FULL_INT_CLR | UART_RXFIFO_TOUT_INT_CLR);
}

uint8_t DESIoT_CBUF_getByte(DESIoT_CBUF_t *hCBuf, uint8_t *rx)
{
    if (hCBuf->end != hCBuf->start)
    {
        *rx = hCBuf->buffer[hCBuf->start++];
        return DESIOT_CBUF_OK;
    }

    return DESIOT_CBUF_ERROR;
}

void DESIoT_CBUF_putByte(DESIoT_CBUF_t *hCBuf, uint8_t rx)
{
    hCBuf->buffer[hCBuf->end++] = rx;
}

void DESIoT_setUpStartOfParsing(DESIoT_Frame_Hander_t *hFrame, DESIoT_CBUF_t *curCBuf)
{
    hFrame->millis = DESIoT_millis();
    hFrame->curCBuf = curCBuf;
    hFrame->curCBuf->startRestore = hFrame->curCBuf->start;
}

void DESIoT_FRAME_parsing(DESIoT_Frame_Hander_t *hFrame, uint8_t byte, DESIoT_CBUF_t *curCBuf)
{
    switch (hFrame->index)
    {
    case DESIOT_H1_INDEX:
        DESIoT_setUpStartOfParsing(hFrame, curCBuf);
        if (byte == DESIOT_H1_DEFAULT)
            hFrame->frame.h1 = byte;

        else
            DESIOT_SET_FRAME_FAILED_STATUS(hFrame->status);
        break;
    case DESIOT_H2_INDEX:
        if (byte == DESIOT_H2_DEFAULT)
            hFrame->frame.h2 = byte;
        else
            DESIOT_SET_FRAME_FAILED_STATUS(hFrame->status);
        break;
    case DESIOT_CMD_INDEX:
        hFrame->frame.dataPacket.cmd = byte;
        break;
    case DESIOT_DATALEN_INDEX:
        hFrame->frame.dataPacket.dataLenArr[0] = byte;
        break;
    case DESIOT_DATALEN_INDEX + 1:
        hFrame->frame.dataPacket.dataLenArr[1] = byte;
        break;
    default:
        if (hFrame->index == (DESIOT_HEAD_FRAME_LEN + hFrame->frame.dataPacket.dataLen)) // t1
        {
            if (byte == DESIOT_T1_DEFAULT)
                hFrame->frame.t1 = byte;
            else
                DESIOT_SET_FRAME_FAILED_STATUS(hFrame->status);
        }
        else if (hFrame->index == (DESIOT_HEAD_FRAME_LEN + hFrame->frame.dataPacket.dataLen + 1)) // t2
        {
            if (byte == DESIOT_T2_DEFAULT)
                hFrame->frame.t2 = byte;
            else
                DESIOT_SET_FRAME_FAILED_STATUS(hFrame->status);
        }
        else if (hFrame->index == (DESIOT_HEAD_FRAME_LEN + hFrame->frame.dataPacket.dataLen + 2)) // crc1
        {
            hFrame->frame.crcArr[0] = byte;
        }
        else if (hFrame->index == (DESIOT_HEAD_FRAME_LEN + hFrame->frame.dataPacket.dataLen + 3)) // crc2
        {
            hFrame->frame.crcArr[1] = byte;
            uint16_t crcCalculate = DESIoT_Compute_CRC16((uint8_t *)&hFrame->frame.dataPacket, DESIOT_CMD_LEN + DESIOT_DATALEN_LEN + hFrame->frame.dataPacket.dataLen);
            if (crcCalculate == hFrame->frame.crc)
            {
                DESIOT_SET_FRAME_SUCCESS_STATUS(hFrame->status);
                Serial.printf("\r\nFrame parsing successfully!");
            }
            else
            {
                Serial.printf("\r\nCRC failed");
                DESIOT_SET_FRAME_FAILED_STATUS(hFrame->status);
            }
        }
        else
        {
            hFrame->frame.dataPacket.data[hFrame->index - (DESIOT_HEAD_LEN + DESIOT_CMD_LEN + DESIOT_DATALEN_LEN)] = byte;
        }
        break;
    }
    hFrame->index++;
}
unsigned short crctable16[256] = {0x0000, 0x1305, 0x260A, 0x350F, 0x4C14, 0x5F11, 0x6A1E, 0x791B, 0x9828, 0x8B2D, 0xBE22, 0xAD27, 0xD43C, 0xC739, 0xF236, 0xE133, 0x2355, 0x3050, 0x055F, 0x165A, 0x6F41, 0x7C44, 0x494B, 0x5A4E, 0xBB7D, 0xA878, 0x9D77, 0x8E72, 0xF769, 0xE46C, 0xD163, 0xC266, 0x46AA, 0x55AF, 0x60A0, 0x73A5, 0x0ABE, 0x19BB, 0x2CB4, 0x3FB1, 0xDE82, 0xCD87, 0xF888, 0xEB8D, 0x9296, 0x8193, 0xB49C, 0xA799, 0x65FF, 0x76FA, 0x43F5, 0x50F0, 0x29EB, 0x3AEE, 0x0FE1, 0x1CE4, 0xFDD7, 0xEED2, 0xDBDD, 0xC8D8, 0xB1C3, 0xA2C6, 0x97C9, 0x84CC, 0x8D54, 0x9E51, 0xAB5E, 0xB85B, 0xC140, 0xD245, 0xE74A, 0xF44F, 0x157C, 0x0679, 0x3376, 0x2073, 0x5968, 0x4A6D, 0x7F62, 0x6C67, 0xAE01, 0xBD04, 0x880B, 0x9B0E, 0xE215, 0xF110, 0xC41F, 0xD71A, 0x3629, 0x252C, 0x1023, 0x0326, 0x7A3D, 0x6938, 0x5C37, 0x4F32, 0xCBFE, 0xD8FB, 0xEDF4, 0xFEF1, 0x87EA, 0x94EF, 0xA1E0, 0xB2E5, 0x53D6, 0x40D3, 0x75DC, 0x66D9, 0x1FC2, 0x0CC7, 0x39C8, 0x2ACD, 0xE8AB, 0xFBAE, 0xCEA1, 0xDDA4, 0xA4BF, 0xB7BA, 0x82B5, 0x91B0, 0x7083, 0x6386, 0x5689, 0x458C, 0x3C97, 0x2F92, 0x1A9D, 0x0998, 0x09AD, 0x1AA8, 0x2FA7, 0x3CA2, 0x45B9, 0x56BC, 0x63B3, 0x70B6, 0x9185, 0x8280, 0xB78F, 0xA48A, 0xDD91, 0xCE94, 0xFB9B, 0xE89E, 0x2AF8, 0x39FD, 0x0CF2, 0x1FF7, 0x66EC, 0x75E9, 0x40E6, 0x53E3, 0xB2D0, 0xA1D5, 0x94DA, 0x87DF, 0xFEC4, 0xEDC1, 0xD8CE, 0xCBCB, 0x4F07, 0x5C02, 0x690D, 0x7A08, 0x0313, 0x1016, 0x2519, 0x361C, 0xD72F, 0xC42A, 0xF125, 0xE220, 0x9B3B, 0x883E, 0xBD31, 0xAE34, 0x6C52, 0x7F57, 0x4A58, 0x595D, 0x2046, 0x3343, 0x064C, 0x1549, 0xF47A, 0xE77F, 0xD270, 0xC175, 0xB86E, 0xAB6B, 0x9E64, 0x8D61, 0x84F9, 0x97FC, 0xA2F3, 0xB1F6, 0xC8ED, 0xDBE8, 0xEEE7, 0xFDE2, 0x1CD1, 0x0FD4, 0x3ADB, 0x29DE, 0x50C5, 0x43C0, 0x76CF, 0x65CA, 0xA7AC, 0xB4A9, 0x81A6, 0x92A3, 0xEBB8, 0xF8BD, 0xCDB2, 0xDEB7, 0x3F84, 0x2C81, 0x198E, 0x0A8B, 0x7390, 0x6095, 0x559A, 0x469F, 0xC253, 0xD156, 0xE459, 0xF75C, 0x8E47, 0x9D42, 0xA84D, 0xBB48, 0x5A7B, 0x497E, 0x7C71, 0x6F74, 0x166F, 0x056A, 0x3065, 0x2360, 0xE106, 0xF203, 0xC70C, 0xD409, 0xAD12, 0xBE17, 0x8B18, 0x981D, 0x792E, 0x6A2B, 0x5F24, 0x4C21, 0x353A, 0x263F, 0x1330, 0x0035};

void DESIoT_CalculateTable_CRC16()
{
    for (int divident = 0; divident < 256; divident++) /* iterate over all possible input byte values 0 - 255 */
    {
        unsigned short curByte = (unsigned short)(divident << 8); /* move divident byte into MSB of 16Bit CRC */
        for (unsigned char bit = 0; bit < 8; bit++)
        {
            if ((curByte & 0x8000) != 0)
            {
                curByte <<= 1;
                curByte ^= DESIOT_CRC_GENERATOR;
            }
            else
                curByte <<= 1;
        }

        crctable16[divident] = curByte;
    }
}

uint16_t DESIoT_Compute_CRC16(uint8_t *bytes, const int32_t BYTES_LEN)
{
    uint16_t crc = 0;

    for (int32_t i = 0; i < BYTES_LEN; i++)
    {
        uint8_t b = bytes[i];

        /* XOR-in next input byte into MSB of crc, that's our new intermediate divident */
        uint8_t pos = (uint8_t)((crc >> 8) ^ b);
        /* Shift out the MSB used for division per lookuptable and XOR with the remainder */
        crc = (uint16_t)((crc << 8) ^ (uint16_t)(crctable16[pos]));
    }
    //	printf("\nHERE 3");

    return crc;
}

void DESIoT_G_frameArbitrating()
{
    // arbitrating for UART2
    if (hFrame.status == DESIOT_FRAME_IDLE || hFrame.status == DESIOT_FRAME_IN_UART2_PROGRESS)
    {
        uint8_t rx;
        if (DESIoT_CBUF_getByte(&hUART2CBuffer, &rx) == DESIOT_CBUF_OK)
        {
            hFrame.status = DESIOT_FRAME_IN_UART2_PROGRESS;
            DESIoT_FRAME_parsing(&hFrame, rx, &hUART2CBuffer);
        }
    }
    // arbitrating for MQTT
    if (hFrame.status == DESIOT_FRAME_IDLE || hFrame.status == DESIOT_FRAME_IN_MQTT_PROGRESS)
    {
        uint8_t rx;
        if (DESIoT_CBUF_getByte(&hMQTTCBuffer, &rx) == DESIOT_CBUF_OK)
        {
            hFrame.status = DESIOT_FRAME_IN_MQTT_PROGRESS;
            DESIoT_FRAME_parsing(&hFrame, rx, &hMQTTCBuffer);
        }
    }
}

void DESIoT_frameFailedHandler()
{
    uint8_t isFailed = 0;
    switch (hFrame.status)
    {
    case DESIOT_FRAME_UART2_FAILED:
    case DESIOT_FRAME_MQTT_FAILED:
        isFailed = 1;
        break;
    }

    if (isFailed)
    {
        DESIoT_restartFrameIndexes();
        DESIoT_restartCBufIndexes();
    }
}
void DESIoT_frameSuccessHandler()
{
    switch (hFrame.status)
    {
    case DESIOT_FRAME_UART2_SUCCESS:
        DESIoT_sendFrameToServer(DESIOT_SERIAL_CONNECTION_TYPE, DESIOT_UART2_ID);
        DESIoT_restartFrameIndexes();
        break;
    case DESIOT_FRAME_MQTT_SUCCESS:
    {
#ifdef DESIOT_ENCRYPTION_ENABLED

        uint8_t tagMatch = DESIoT_decryptData();
        if (tagMatch)
#endif
        {
            DESIoT_sendFrameToDevice();
        }
        DESIoT_restartFrameIndexes();
        break;
    }

    default:
        break;
    }
}

void DESIoT_restartFrameIndexes()
{
    hFrame.status = DESIOT_FRAME_IDLE;
    hFrame.index = 0;
}

void DESIoT_restartCBufIndexes()
{
    hFrame.curCBuf->start = hFrame.curCBuf->startRestore;

    // Flush to next headers
    uint16_t currentCBufEnd = hFrame.curCBuf->end;
    for (; hFrame.curCBuf->start != currentCBufEnd; hFrame.curCBuf->start++)
    {
        // check for H1 and H2 mathch
        if (hFrame.curCBuf->buffer[hFrame.curCBuf->start] == DESIOT_H1_DEFAULT && hFrame.curCBuf->buffer[hFrame.curCBuf->start + 1] == DESIOT_H2_DEFAULT)
            break;
    }
}

void DESIoT_frameTimeoutHandler()
{
    if (DESIOT_IS_FRAME_ON_PROCESS_STATUS(hFrame.status))
        if (DESIoT_millis() - hFrame.millis > DESIOT_TIMEOUT_DURATION)
        {
            DESIoT_restartFrameIndexes();
            DESIoT_restartCBufIndexes();
            Serial.printf("\r\nFrame timeout");
        }
}

void DESIoT_sendFrameToServer(uint8_t connection_type, uint8_t connection_id)
{
    char *payload = (char *)&hFrame.frame;

    // check data length
    if (hFrame.frame.dataPacket.dataLen + DESIOT_ADDITIONAL_GATEWAY_FRAME_SIZE <= sizeof(hFrame.frame.dataPacket.data))
    {
        // shift data of data packet of 14 bytes
        memmove(hFrame.frame.dataPacket.data + DESIOT_ADDITIONAL_GATEWAY_FRAME_SIZE, hFrame.frame.dataPacket.data, hFrame.frame.dataPacket.dataLen);
        hFrame.frame.dataPacket.dataLen += DESIOT_ADDITIONAL_GATEWAY_FRAME_SIZE;

        DESIoT_additionalGatewayData_t *additionalGatewayData = (DESIoT_additionalGatewayData_t *)hFrame.frame.dataPacket.data;

        memcpy(additionalGatewayData->gateway_id, hFrame.gateway_id, sizeof(hFrame.gateway_id));
        // additionalGatewayData->gateway_id =
        additionalGatewayData->connection_type = connection_type;
        additionalGatewayData->connection_id = connection_id;

#ifdef DESIOT_ENCRYPTION_ENABLED
        DESIoT_encryptData();
#endif

        // crc
        hFrame.frame.crc = DESIoT_Compute_CRC16((uint8_t *)&hFrame.frame.dataPacket, DESIOT_CMD_LEN + DESIOT_DATALEN_LEN + hFrame.frame.dataPacket.dataLen);
    }
    else
        return;

    size_t length = DESIOT_FIXED_COMPOMENTS_LENGTH + hFrame.frame.dataPacket.dataLen;

    // shift trail frame
    char *pTrailFrame = payload + DESIOT_HEAD_FRAME_LEN + hFrame.frame.dataPacket.dataLen;
    memcpy(pTrailFrame, &hFrame.frame.t1, DESIOT_TRAIL_FRAME_LEN);

    uint16_t packetID = mqttClient.publish(DESIOT_MQTT_PUBLISH_TOPIC, 2, false, payload, length);
    if (!packetID)
        Serial.printf("\r\nPublish failed");
    else
    {
        printf("\r\nPublish successfully with %d bytes of data", length);
    }
}

void DESIoT_sendFrameToDevice()
{
    char *src = (char *)&hFrame.frame;
    uint8_t connection_type = hFrame.frame.dataPacket.data[0], connection_id = hFrame.frame.dataPacket.data[1];

    // shift data.
    size_t shift_value = DESIOT_ADDITIONAL_GATEWAY_FRAME_SIZE - DESIOT_GATEWAYID_SIZE;
    hFrame.frame.dataPacket.dataLen -= shift_value;
    memmove(hFrame.frame.dataPacket.data, hFrame.frame.dataPacket.data + shift_value, hFrame.frame.dataPacket.dataLen);

    // crc
    hFrame.frame.crc = DESIoT_Compute_CRC16((uint8_t *)&hFrame.frame.dataPacket, DESIOT_CMD_LEN + DESIOT_DATALEN_LEN + hFrame.frame.dataPacket.dataLen);

    size_t length = DESIOT_FIXED_COMPOMENTS_LENGTH + hFrame.frame.dataPacket.dataLen;

    // shift trail frame
    char *pTrailFrame = src + DESIOT_HEAD_FRAME_LEN + hFrame.frame.dataPacket.dataLen;
    memcpy(pTrailFrame, &hFrame.frame.t1, DESIOT_TRAIL_FRAME_LEN);

    if (connection_type == DESIOT_SERIAL_CONNECTION_TYPE)
    {
        switch (connection_id)
        {
        case DESIOT_UART2_ID:
            uart_tx_chars(DESIOT_UART_NUM, src, length);
            break;

        default:
            break;
        }
    }
}
void connectToMqtt()
{
    if (!mqttClient.connected())
    {
        Serial.println("Connecting to MQTT...");
        mqttClient.connect();
    }
}

void WiFiEvent(WiFiEvent_t event)
{
    Serial.printf("[WiFi-event] event: %d\n", event);
    switch (event)
    {
    case SYSTEM_EVENT_STA_GOT_IP:
        Serial.println("WiFi connected");
        Serial.println("IP address: ");
        Serial.println(WiFi.localIP());
        connectToMqtt();
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        Serial.println("WiFi lost connection");
        xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
        xTimerStart(wifiReconnectTimer, 0);
        break;
    }
}

void onMqttConnect(bool sessionPresent)
{
    Serial.println("Connected to MQTT.");
    Serial.print("Session present: ");
    Serial.println(sessionPresent);
    uint16_t packetIdSub = mqttClient.subscribe(hFrame.mqttTopic, 2);
    Serial.print("Subscribing at QoS 2, packetId: ");
    Serial.println(packetIdSub);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason)
{
    Serial.println("Disconnected from MQTT.");

    if (WiFi.isConnected())
    {
        xTimerStart(mqttReconnectTimer, 0);
    }
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos)
{
    Serial.println("Subscribe acknowledged.");
    Serial.print("  packetId: ");
    Serial.println(packetId);
    Serial.print("  qos: ");
    Serial.println(qos);
}

void onMqttUnsubscribe(uint16_t packetId)
{
    Serial.println("Unsubscribe acknowledged.");
    Serial.print("  packetId: ");
    Serial.println(packetId);
}

void onMqttMessage(char *topic, char *payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total)
{

    if (strcmp(topic, hFrame.mqttTopic) == 0)
    {
        for (size_t i = 0; i < len; i++)
        {
            DESIoT_CBUF_putByte(&hMQTTCBuffer, payload[i]);
        }
    }
}

void onMqttPublish(uint16_t packetId)
{
    // Serial.println("Publish acknowledged.");
    // Serial.print("  packetId: ");
    // Serial.println(packetId);
}

#ifdef DESIOT_ENCRYPTION_ENABLED

uint8_t aad[12] = {ENV_AAD};
uint8_t key[32] = {ENV_KEY};
uint8_t nonce[] = {ENV_NONCE};
void DESIoT_encryptData()
{
    // shift data
    memmove(hFrame.frame.dataPacket.data + DESIOT_ENCRYPT_TAG_SIZE, hFrame.frame.dataPacket.data, hFrame.frame.dataPacket.dataLen);

    uint8_t *actual_plaintext = hFrame.frame.dataPacket.data + DESIOT_ENCRYPT_TAG_SIZE;
    uint8_t *ciphertext = actual_plaintext;
    size_t n = hFrame.frame.dataPacket.dataLen;

    hFrame.frame.dataPacket.dataLen += DESIOT_ENCRYPT_TAG_SIZE;
    chacha20poly1305_ctx ctx;

    rfc7539_init(&ctx, key, nonce);
    rfc7539_auth(&ctx, aad, sizeof(aad));
    chacha20poly1305_encrypt(&ctx, actual_plaintext, ciphertext, n);

    uint8_t *tag = hFrame.frame.dataPacket.data;
    rfc7539_finish(&ctx, sizeof(aad), n, tag);
}

uint8_t DESIoT_decryptData()
{
    uint8_t *ciphertext = hFrame.frame.dataPacket.data + DESIOT_ENCRYPT_TAG_SIZE, *plaintext = ciphertext;
    size_t n = hFrame.frame.dataPacket.dataLen - DESIOT_ENCRYPT_TAG_SIZE;
    chacha20poly1305_ctx ctx;
    rfc7539_init(&ctx, key, nonce);
    rfc7539_auth(&ctx, aad, sizeof(aad));
    chacha20poly1305_decrypt(&ctx, ciphertext, plaintext, n);

    uint8_t *tag = hFrame.frame.dataPacket.data;
    uint8_t computedTag[DESIOT_ENCRYPT_TAG_SIZE];
    rfc7539_finish(&ctx, sizeof(aad), n, computedTag);

    uint8_t tagMatch = !memcmp(computedTag, tag, DESIOT_ENCRYPT_TAG_SIZE);

    if (tagMatch) // shift data
    {
        memmove(hFrame.frame.dataPacket.data, plaintext, n);
        hFrame.frame.dataPacket.dataLen = n;
    }

    return tagMatch;
}
#endif
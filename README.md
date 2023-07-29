# Semantic versioning

## 1.13.0

`29-07-2023`

- Flush CBuf to the next headers.
  - Set this method when

## 1.12.0

`29-07-2023`

- Change default **TRAILERS** to make it completely different from the default **HEADERS**. (0x70, 0x71)
- Restore the start point of the CBuffer.
  - Set the **startRestore** and **curCBuf** in the start of parsing process.
  - Restore the **start** of **curCBuf** when frame faild or timeout.

## 1.11.0

`28-07-2023`

- Measure tick in **DESIoT_G_loop**
- Restart ESP when tick timeout.

## 1.10.0

`13-07-2023`

- ## Decrypt value when receiving MQTT frame success.

## 1.9.0

`13-07-2023`

- Implement ChaCha20-Poly1305 lightweight cryptography algorithm.
  - Add headers.
    - **chacha20poly1305.h**
    - **ecrypt-config.h**
    - **ecrypt-machine.h**
    - **ecrypt-portable.h**
    - **ecrypt-sync.h**
    - **ecrypt-types.h**
    - **poly1305-donna.h**
    - **poly1305-donna-32.h**
    - **rfc7539.h**
  - Add sources:
    - **chacha_merged.c**
    - **chacha20poly1305.c**
    - **poly1305-donna.c**
    - **rfc7539.c**
- Create encryption function.
  - Call function when defined DESIOT_ENCRYPTION_ENABLED

## 1.8.0

`12-07-2023`

- Check if the topic from the server when **onMqttMessage**.
  - Set mqtt topic in the begin.
  - Push payload to CBuffer when receiveing message.
  - Set statuses for MQTT atbitrating.
  - Config **DESIoT_frameFailedHandler** and **DESIoT_frameSuccessHandler**
- Send frame to device.
  - Get connection type and connection id from frame.
  - Check connection type.
  - Shift data. Eliminate two connection ids.
  - Calculate CRC.

## 1.7.0

- Using 25-byte string ID instead of 12-byte u8 ID.
  - Change **begin** method.
  - Remote **DESIoT_hexToU8Array** method.
    - Fix **DESIoT_additionalGatewayData_t**.

## 1.6.1

- Fix frame shifting when sending it to server.

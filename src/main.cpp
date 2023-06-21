#include <Arduino.h>
#include <DESIoT_Gateway.h>
// put function declarations here:
CBufer_handleTypeDef_t hcBuffer = {
    .start = 0u,
    .end = 0u};

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);
  DESIoT_G_begin();
}

void loop()
{
  // put your main code here, to run repeatedly:
  DESIoT_G_loop();
  uint8_t rx;
  if (CBUFFER_getByte(&hcBuffer, &rx) == CBUFFER_OK)
    Serial.printf("\r\n%02X", rx);
}

// put function definitions here:
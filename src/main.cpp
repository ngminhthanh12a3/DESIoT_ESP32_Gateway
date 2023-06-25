#define DESIOT_USER_WIFI_SSID "ThanhNguyen_test"
#define DESIOT_USER_WIFI_PASSWORD "nguyent1220"
#define DESIOT_USER_GATEWAY_ID "64953a3c976b713b0f840520"
// #include <Arduino.h>
#include "DESIoT_Gateway.h"

// put function declarations here:

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  DESIoT_G_begin();
  // DESIoT_getUserMacros();
}

void loop() {
  // put your main code here, to run repeatedly:
  DESIoT_G_loop();
}

// put function definitions here:
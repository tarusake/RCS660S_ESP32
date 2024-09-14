#include <Arduino.h>
#include <M5Unified.h>

#include "RCS660S_ESP32.h"

// M5 Core2
#define UART_TX_PIN 33
#define UART_RX_PIN 32

RCS660S rcs660s(Serial1);

void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);
  Serial.println("Hello World!!");
  Serial1.begin(115200, SERIAL_8N1, UART_TX_PIN, UART_RX_PIN);
  rcs660s.initDevice();
}

void loop() {
  if (rcs660s.polling()) {
    Serial.print("idm = ");
    for (int i = 0; i < 8; i++) {
      Serial.print(rcs660s.idm[i], HEX);
    }
    Serial.println();

    Serial.print("pmm = ");
    for (int i = 0; i < 8; i++) {
      Serial.print(rcs660s.pmm[i], HEX);
    }
    Serial.println();
  }
  delay(100);
}

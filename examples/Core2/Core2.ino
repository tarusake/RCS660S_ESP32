#include <M5Unified.h>
#include <RCS660S_ESP32.h>

// M5Stack Core2 Grove Port A: GPIO33 (RX), GPIO32 (TX)
const uint8_t uartRx = 33;
const uint8_t uartTx = 32;
const int pollingInterval = 300;

RCS660S nfc(Serial2);

void setup() {
  M5.begin();
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.println("RC-S660 UART Test (Core2)");
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, uartRx, uartTx);
  nfc.initDevice();
}

void loop() {
  // FeliCaカードのポーリング
  if (nfc.polling() == 1) {
    // カード検出時の処理
    M5.Lcd.print("Card Detect : IDm = ");

    // IDmの表示
    for (int i = 0; i < 8; i++) {
      if (nfc.idm[i] < 0x10) M5.Lcd.print('0');
      M5.Lcd.print(nfc.idm[i], HEX);
    }
    M5.Lcd.println();

    delay(1000);
  }

  delay(100);
}

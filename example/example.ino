#include <M5Unified.h>
#include <RCS660S.h>

const uint8_t txdp = 16;
const uint8_t rxdp = 17;
const int pollingInterval = 300;

RCS660S nfc(Serial2);

void setup() {
  M5.begin();
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.println("RC-S660 UART Test");
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, txdp, rxdp);
  nfc.initDevice();
}

void loop() {
  // FeliCaカードのポーリング
  if (nfc.polling() == 1) {
    // カード検出時の処理
    M5.Lcd.print("カード検出: IDm = ");

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
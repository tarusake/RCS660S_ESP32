#include <M5Unified.h>
#include <RCS660S_ESP32.h>

// M5Stack Core2 Grove Port A: GPIO33 (RX), GPIO32 (TX)
const uint8_t uartRx = 33;
const uint8_t uartTx = 32;

RCS660S nfc(Serial2);

// Suica balance service code (little-endian on wire)
#define SUICA_SERVICE_CODE 0x090F
// Block number to read (block 0 holds balance)
#define BLOCK_NUMBER 0x00

/**
 * @brief Read Suica balance using FeliCa Read Without Encryption.
 * @param balance Pointer to store balance in yen.
 * @return 0 on success, 1 on failure.
 */
int getSuicaBalance(int *balance) {
  int ret;
  uint8_t command[17];
  uint8_t response[256];
  uint8_t responseLen;

  // Build Read Without Encryption command
  command[0] = 0x06;                       // Command code
  memcpy(command + 1, nfc.idm, 8);         // IDm (card ID)
  command[9] = 0x01;                       // Service count
  command[10] = SUICA_SERVICE_CODE & 0xFF; // Service code (LE)
  command[11] = SUICA_SERVICE_CODE >> 8;   // Service code (LE)
  command[12] = 0x01;                      // Block count
  command[13] = 0x80;                      // Block list (2-byte format)
  command[14] = BLOCK_NUMBER;              // Block number

  ret = nfc.cardCommand(command, 15, response, &responseLen);

  // Validate response length
  if (!ret || (responseLen < 14)) {
    return 1;
  }

  // Balance is stored at response[22..23] (little-endian)
  *balance = (response[23] << 8) | response[22];

  return 0;
}

void setup() {
  M5.begin();
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.println("RC-S660 UART Test (Core2)");
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, uartRx, uartTx);
  // Initialize reader and start transparent session
  nfc.initDevice();
}

void loop() {
  // Poll for FeliCa cards (0x0003 = Suica/FeliCa system code)
  if (nfc.polling(0x0003) == 1) {
    // Card detected
    M5.Lcd.print("Card Detect : IDm = ");

    // Print IDm
    for (int i = 0; i < 8; i++) {
      if (nfc.idm[i] < 0x10) M5.Lcd.print('0');
      M5.Lcd.print(nfc.idm[i], HEX);
    }
    M5.Lcd.println();

    int balance = 0;
    if (getSuicaBalance(&balance) == 0) {
      // Display balance
      M5.Lcd.print("Suica Balance: ");
      M5.Lcd.print(balance);
      M5.Lcd.println(" yen");
    } else {
      // Read failed
      M5.Lcd.println("Suica Balance: read failed");
    }

    // Simple debounce to avoid repeated reads
    delay(1000);
  }

  // Polling interval
  delay(100);
}

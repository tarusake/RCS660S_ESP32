#include "RCS660S_ESP32.h"

#define RCS660S_DEFAULT_TIMEOUT 1000
#define RCS660_CCID_COMMAND_MAX 256
#define RCS660_CCID_RESPONSE_MAX 256

RCS660S::RCS660S(Stream &serial) {
  _serial = &serial;
  bSeq = 1;
  this->timeout = RCS660S_DEFAULT_TIMEOUT;
}

int RCS660S::polling(uint16_t systemCode) {
  uint8_t command[] = {0xFF, 0xC2, 0x00, 0x01, 0x0F, 0x5F, 0x46, 0x04, 0x60, 0xEA, 0x00, 0x00, 0x95, 0x06, 0x06, 0x00, 0xFF, 0xFF, 0x00, 0x00};
  uint8_t response[RCS660_CCID_RESPONSE_MAX];
  uint16_t res_len;

  command[16] = (uint8_t)((systemCode >> 8) & 0xff);
  command[17] = (uint8_t)((systemCode >> 0) & 0xff);
  apduCommand((const uint8_t *)command, sizeof(command), response, &res_len);

  if (res_len != 44) {
    return 0;
  }

  memcpy(this->idm, response + 26, 8);
  memcpy(this->pmm, response + 34, 8);

  return 1;
}

int RCS660S::initDevice() {
  uint8_t response[RCS660_CCID_RESPONSE_MAX];
  uint16_t res_len;

  Serial.println("-----");

  wakeup();
  abort();
  apduCommand((const uint8_t *)"\xFF\xC2\x00\x00\x02\x82\x00", (uint32_t)7, response, &res_len);
  apduCommand((const uint8_t *)"\xFF\xC2\x00\x00\x02\x81\x00", (uint32_t)7, response, &res_len);
  apduCommand((const uint8_t *)"\xFF\xC2\x00\x02\x04\x8F\x02\x03\x00", (uint32_t)9, response, &res_len);
  apduCommand((const uint8_t *)"\xFF\xC2\x00\x01\x04\x90\x02\x00\x1C", (uint32_t)9, response, &res_len);
  apduCommand((const uint8_t *)"\xFF\xC2\x00\x01\x03\x91\x01\x00", (uint32_t)8, response, &res_len);
  apduCommand((const uint8_t *)"\xFF\xC2\x00\x00\x06\xFF\x6E\x03\x05\x01\x89", (uint32_t)11, response, &res_len);
  apduCommand((const uint8_t *)"\xFF\xC2\x00\x00\x02\x84\x00\x00", (uint32_t)8, response, &res_len);

  return 0;
}

int RCS660S::apduCommand(const uint8_t *command, uint32_t len, uint8_t *response, uint16_t *res_len) {
  uint8_t buf[RCS660_CCID_COMMAND_MAX];

  buf[0] = 0x6B;
  buf[1] = (len >> 0) & 0xff;
  buf[2] = (len >> 8) & 0xff;
  buf[3] = (len >> 16) & 0xff;
  buf[4] = (len >> 24) & 0xff;
  buf[5] = 0x00;
  buf[6] = bSeq++;
  buf[7] = 0x00;
  buf[8] = 0x00;
  buf[9] = 0x00;
  memcpy(buf + 10, command, len);

  if (ccidCommand(buf, 10 + len, response, res_len)) {
    return 1;
  }

  printHex("ccid_res : ", response, *res_len);

  return 0;
}

void RCS660S::wakeup() {
  writeSerial((const uint8_t *)"\x01", 1);
  delay(20);
}

int RCS660S::abort() {
  uint8_t command[10] = {0x72, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  uint8_t response[18];
  uint16_t res_len;

  command[5] = bSeq++;

  ccidCommand((const uint8_t *)command, 10, response, &res_len);
  return 0;
}

int RCS660S::ccidCommand(const uint8_t *command, uint16_t len, uint8_t *response, uint16_t *res_len) {
  uint8_t buf[RCS660_CCID_COMMAND_MAX];
  uint8_t res_header_buf[6], footer_buf[2];

  buf[0] = 0x00;
  buf[1] = 0x00;
  buf[2] = 0xFF;
  buf[3] = (len >> 8) & 0xff;
  buf[4] = (len >> 0) & 0xff;
  buf[5] = calcDCS(buf + 3, 2);
  memcpy(buf + 6, command, len);
  buf[6 + len] = calcDCS(command, len);
  buf[6 + len + 1] = 0x00;

  writeSerial(buf, 6 + len + 2);

  receiveAck();
  readSerial(res_header_buf, 6);

  if (memcmp(res_header_buf, "\x00\x00\xff", 3)) {
    Serial.printf("Invalid Response\n");
    return 1;
  }

  *res_len = (res_header_buf[3] << 8) + res_header_buf[4];
  Serial.printf("res_len:%d\n", *res_len);

  readSerial(response, *res_len);
  readSerial(footer_buf, 2);

  return 0;
}

int RCS660S::receiveAck() {
  uint8_t buf[7];

  if (readSerial(buf, 7) != 7) {
    return 1;
  }

  if (memcmp(buf, "\x00\x00\xFF\x00\x00\xFF\x00", 7)) {
    return 1;
  }

  Serial.println("receiveAck:OK");
  return 0;
}

int RCS660S::writeSerial(const uint8_t *buf, uint16_t len) {
  printHex("write : ", buf, len);
  return _serial->write(buf, len);
}

int RCS660S::readSerial(uint8_t *buf, uint16_t len) {
  uint16_t cnt = 0;
  unsigned long t0 = millis();
  while (cnt < len) {
    unsigned long t1 = millis();

    if ((t1 - t0) >= this->timeout) {
      break;
    }

    if (_serial->available() > 0) {
      buf[cnt] = _serial->read();
      cnt++;
    }
  }

  printHex("read  : ", buf, len);
  return cnt;
}

uint8_t RCS660S::calcDCS(const uint8_t *data, uint16_t len) {
  uint8_t sum = 0;

  for (uint16_t i = 0; i < len; i++) {
    sum += data[i];
  }

  return (uint8_t) - (sum & 0xff);
}

void RCS660S::printHex(const char *header, const uint8_t *data, size_t len) {
  Serial.print(header);
  for (size_t i = 0; i < len; i++) {
    if (data[i] < 0x10) {
      Serial.print("0");
    }
    Serial.print(data[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}
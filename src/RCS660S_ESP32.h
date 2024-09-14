#ifndef RCS660S_ESP32_H_
#define RCS660S_ESP32_H_

#include <M5Unified.h>

class RCS660S {
public:
  RCS660S(Stream &serial = Serial1);

  int initDevice(void);
  void wakeup(void);
  int abort(void);
  int polling(uint16_t systemCode = 0xFFFF);
  int apduCommand(const uint8_t *command, uint32_t len, uint8_t *response, uint16_t *res_len);
  int ccidCommand(const uint8_t *command, uint16_t len, uint8_t *response, uint16_t *res_len);

private:
  Stream *_serial;
  uint8_t bSeq;

  int writeSerial(const uint8_t *buf, uint16_t len);
  int readSerial(uint8_t *buf, uint16_t len);
  int receiveAck();
  uint8_t calcDCS(const uint8_t *data, uint16_t len);
  void printHex(const char *header, const uint8_t *data, size_t len);

public:
  unsigned long timeout;
  uint8_t idm[8];
  uint8_t pmm[8];
};

#endif
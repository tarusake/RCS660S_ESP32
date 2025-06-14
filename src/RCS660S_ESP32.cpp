/**
 * RC-S660/S for Arduino
 * Implementation of RC-S660/S NFC reader interface
 * @author tarusake
 */
#include <Arduino.h>
#include "RCS660S_ESP32.h"

RCS660S::RCS660S(Stream &serial)
{
    _serial = &serial;
    this->timeout = 1000;         // Default timeout 1000ms
    this->bseq = 0x0;             // Initialize sequence number
    this->logLevel = LOG_ERROR;   // Debug mode off by default
}

/**
 * RCS620S compatible wrapper for card commands
 */
int RCS660S::cardCommand(
    const uint8_t *command,
    uint8_t command_len,
    uint8_t *response,
    uint8_t *response_len)
{
    uint8_t buf[RCS660S_BUFFER_SIZE];
    uint16_t buf_len;

    // Build command APDU
    buf[0] = RCS660S_CLA_DEFAULT;         // CLA
    buf[1] = RCS660S_INS_CARD_COMMAND;    // INS
    buf[2] = 0x00;                        // P1
    buf[3] = 0x01;                        // P2
    buf[4] = 0x1A;                        // Lc
    // Timer Data Object
    buf[5] = 0x5F;                        // Tag:Timer
    buf[6] = 0x46;                        // Tag:Timer
    buf[7] = 0x04;                        // Length
    buf[8] = 0x60;                        // Timer Value : 0xEA60 = 60000us = 60ms
    buf[9] = 0xEA;
    buf[10] = 0x01;
    buf[11] = 0x00;
    // Transceive Data Object
    buf[12] = 0x95;                       // Tag:Transceive
    buf[13] = (uint8_t)(command_len + 2); // Length
    buf[14] = (uint8_t)(command_len + 2); // Length(dummy)?
    memcpy(buf + 15, command, command_len);
    buf[15 + command_len] = 0x00;

    // Send command and receive response
    write_apdu(buf, 15 + command_len + 1);
    receive_ccid_response(buf, &buf_len);

    // Validate CCID Header
    if (buf[0] != 0x83 || buf[7] != 0x02)
    {
        Serial.println("Error: Invalid CCID response message header");
        return 0;
    }

    // Validate PCSC Response
    if (memcmp(buf + 10, "\xC0\x03\x00\x90\x00", 5))
    {
        Serial.println("Error: Invalid PCSC ManageSessionCommand Response");
        return 0;
    }

    *response_len = (uint8_t)(buf[23] - 1);
    memcpy(response, buf + 25, *response_len);

    return 1;
}

/**
 * Send CCID command to the device
 * @param command Command data to send
 * @param command_len Length of command data
 * @return 1 on success, 0 on failure
 */
int RCS660S::send_ccid_command(const uint8_t *command, uint16_t command_len)
{
    uint8_t dcs;
    uint8_t buf[RCS660S_SMALL_BUFFER_SIZE];

    // Calculate DCS (Data Check Sum)
    dcs = calcDCS(command, command_len);

    // Build CCID command frame
    buf[0] = 0x00;                                 // Preamble
    buf[1] = 0x00;                                 // Start code
    buf[2] = 0xFF;                                 // Start code
    buf[3] = (uint8_t)((command_len >> 8) & 0xFF); // Length (MSB)
    buf[4] = (uint8_t)(command_len & 0xFF);        // Length (LSB)
    buf[5] = (uint8_t) - (buf[3] + buf[4]);       // Length checksum
    memcpy(buf + 6, command, command_len);         // Command data
    buf[6 + command_len] = dcs;                    // Data checksum
    buf[6 + command_len + 1] = 0x00;              // Postamble

    // Send command frame
    writeSerial(buf, 6 + command_len + 2);
    return 1;
}

/**
 * Receive CCID response from the device
 * @param response Buffer to store response data
 * @param response_len Length of received response
 * @return 1 on success, 0 on failure
 */
int RCS660S::receive_ccid_response(uint8_t *response, uint16_t *response_len)
{
    int rc;
    uint8_t buf[RCS660S_SMALL_BUFFER_SIZE];
    uint16_t buf_len;
    uint16_t pd_len;
    uint16_t need_len;

    // Read response command header (6 bytes)
    buf_len = 0;
    rc = readSerial(buf, 6);
    if (!rc)
    {
        Serial.println("Error: UART read timeout");
        return 0;
    }

    // Validate header format
    if (memcmp(buf, "\x00\x00\xff", 3) != 0)
    {
        Serial.println("Error: Invalid response header format");
        return 0;
    }

    // Validate length checksum
    if (((buf[3] + buf[4] + buf[5]) & 0xff) != 0)
    {
        Serial.println("Error: Invalid response header checksum");
        return 0;
    }

    // Calculate and validate payload length
    pd_len = ((uint16_t)buf[3] << 8) + buf[4];
    if (pd_len > RCS660S_SMALL_BUFFER_SIZE - 8)  // Reserve space for header and checksum
    {
        Serial.println("Error: Response length exceeds buffer size");
        return 0;
    }

    // Read remaining data (payload + checksum)
    need_len = pd_len + 2;
    readSerial(buf + 6, need_len);

    // TODO: Add additional response validation checks

    // Copy payload data (excluding header)
    memcpy(response, buf + 6, pd_len);
    *response_len = pd_len;

    // Debug output
    if (this->logLevel >= LOG_DEBUG)
    {
        Serial.printf("CCID Response (%d bytes): ", pd_len);
        printHexArray(response, pd_len);
    }
    return 1;
}

/**
 * Receive acknowledgment from the device
 * ACK frame format: 00 00 FF 00 00 FF 00
 * @return 1 on success (ACK received), 0 on failure
 */
int RCS660S::receive_ack()
{
    int rc;
    uint8_t ack_buf[7];

    // Read ACK frame (7 bytes)
    rc = readSerial(ack_buf, 7);
    if (!rc)
    {
        Serial.println("Error: Failed to read ACK frame (timeout)");
        return 0;
    }

    // Validate ACK frame format
    if (memcmp(ack_buf, "\x00\x00\xff\x00\x00\xff\x00", 7) != 0)
    {
        Serial.println("Error: Invalid ACK frame received");
        return 0;
    }

    return 1;
}

/**
 * Send abort command to the device
 * @return 0 on success (Note: returns 0 to match RCS620S behavior)
 */
int RCS660S::abort_command()
{
    uint8_t abort[10]{
        RCS660S_MSG_TYPE_ABORT,     // Message type
        0x00, 0x00, 0x00, 0x00,     // Length (always 0)
        0x00,                       // Slot number
        0x00,                       // Sequence number (updated below)
        0x00, 0x00, 0x00           // Reserved
    };
    uint8_t response_buf[RCS660S_BUFFER_SIZE];
    uint16_t response_len;

    // Update sequence number
    abort[6] = ++bseq;

    // Send abort command and receive response
    send_ccid_command(abort, sizeof(abort));
    receive_ack();
    receive_ccid_response(response_buf, &response_len);
    return 0;
}

/**
 * Write APDU (Application Protocol Data Unit) to the device
 * @param data APDU data to write
 * @param data_len Length of APDU data
 * @return 0 on success (Note: returns 0 to match RCS620S behavior)
 */
int RCS660S::write_apdu(const uint8_t *data, uint32_t data_len)
{
    uint32_t buf_len;
    uint8_t buf[RCS660S_BUFFER_SIZE];

    // Build CCID message header
    buf[0] = RCS660S_MSG_TYPE_PC_TO_RDR_ESCAPE;  // Message type
    buf[1] = (uint8_t)(data_len & 0xFF);         // Length (LSB)
    buf[2] = (uint8_t)((data_len >> 8) & 0xFF);  // Length
    buf[3] = (uint8_t)((data_len >> 16) & 0xFF); // Length
    buf[4] = (uint8_t)((data_len >> 24) & 0xFF); // Length (MSB)
    buf[5] = 0x00;                               // Slot number
    buf[6] = ++bseq;                             // Sequence number
    buf[7] = 0x00;                               // Reserved
    buf[8] = 0x00;                               // Reserved
    buf[9] = 0x00;                               // Reserved

    // Copy APDU data
    memcpy(&buf[10], data, data_len);
    buf_len = 10 + data_len;

    // Debug output
    if (this->logLevel >= LOG_DEBUG)
    {
        Serial.printf("CCID Command (%d bytes): ", buf_len);
        printHexArray(buf, buf_len);
    }

    // Send command and wait for acknowledgment
    send_ccid_command(buf, buf_len);
    receive_ack();

    return 0;
}

int RCS660S::read_rapdu(uint8_t *data, uint32_t *data_len)
{
    uint8_t buf[1024];
    uint16_t buf_len;

    receive_ccid_response(buf, &buf_len);

    return 0;
}

int RCS660S::initDevice(void)
{
    uint8_t buf[1024];
    uint16_t buf_len;

    // Reset
    _serial->write(0x01);
    delay(20);

    // PC_to_RDR_Abort
    abort_command();

    // APDU : End Transparent Session
    write_apdu((const uint8_t *)"\xFF\xC2\x00\x00\x02\x82\x00", 7);
    receive_ccid_response(buf, &buf_len);

    // APDU : Start Transparent Session
    write_apdu((const uint8_t *)"\xFF\xC2\x00\x00\x02\x81\x00", 7);
    receive_ccid_response(buf, &buf_len);

    // APDU : Switch Protocol
    write_apdu((const uint8_t *)"\xFF\xC2\x00\x02\x04\x8F\x02\x03\x00", 9);
    receive_ccid_response(buf, &buf_len);

    // APDU : Transparent Exchange Transmission and Reception Flag
    write_apdu((const uint8_t *)"\xFF\xC2\x00\x01\x04\x90\x02\x00\x1C", 9);
    receive_ccid_response(buf, &buf_len);

    // APDU : Transparent Exchange Transmission Bit framing
    write_apdu((const uint8_t *)"\xFF\xC2\x00\x01\x03\x91\x01\x00", 8);
    receive_ccid_response(buf, &buf_len);

    // APDU : Manage Session Set Parameters
    write_apdu((const uint8_t *)"\xFF\xC2\x00\x00\x06\xFF\x6E\x03\x05\x01\x89", 11);
    receive_ccid_response(buf, &buf_len);

    // APDU Manage Session Turn On RF Field
    write_apdu((const uint8_t *)"\xFF\xC2\x00\x00\x02\x84\x00\x00", 8);
    receive_ccid_response(buf, &buf_len);

    return 1;
}

int RCS660S::polling(uint16_t systemCode)
{
    int ret;
    uint8_t command[] =
        {
            0xFF, // CLA
            0xC2, // INS
            0x00, // P1
            0x01, // P2
            0x0F, // Lc
            // Data Object
            0x5F, 0x46,             // Tag:Timer
            0x04,                   // Length
            0x60, 0xEA, 0x00, 0x00, // Timer Value
            // Data Object
            0x95,       // Tag:Transceive
            0x06, 0x06, // Length
            0x00,       // Command
            0xFF, 0xFF, // SystemCode
            0x00, 0x00};

    uint8_t buf[1024];
    uint16_t buf_len;

    command[16] = (uint8_t)((systemCode >> 8) & 0xff);
    command[17] = (uint8_t)((systemCode >> 0) & 0xff);
    write_apdu(command, sizeof(command));
    receive_ccid_response(buf, &buf_len);

    // no card
    if (buf_len != 44)
    {
        return 0;
    }

    memcpy(this->idm, buf + 26, 8);
    memcpy(this->pmm, buf + 34, 8);

    return 1;
}

uint8_t RCS660S::calcDCS(const uint8_t *data, uint16_t len)
{
    uint8_t sum = 0;
    uint16_t i;

    for (i = 0; i < len; i++)
    {
        sum += data[i];
    }

    return (uint8_t) - (sum & 0xff);
}

void RCS660S::writeSerial(
    const uint8_t *data,
    uint16_t len)
{
    _serial->write(data, len);

    if (this->logLevel >= LOG_DEBUG)
    {
        Serial.print("WriteSerial : ");
        printHexArray(data, len);
    }
}

int RCS660S::readSerial(
    uint8_t *data,
    uint16_t len)
{
    uint16_t nread = 0;
    unsigned long t0 = millis();

    while (nread < len)
    {
        if (checkTimeout(t0))
        {
            return 0;
        }

        if (_serial->available() > 0)
        {
            data[nread] = _serial->read();
            nread++;
        }
    }

    if (this->logLevel >= LOG_DEBUG)
    {
        Serial.print("ReadSerial  : ");
        printHexArray(data, len);
    }
    return 1;
}

void RCS660S::flushSerial(void)
{
    _serial->flush();
}

int RCS660S::checkTimeout(unsigned long t0)
{
    unsigned long t = millis();

    if ((t - t0) >= this->timeout)
    {
        return 1;
    }

    return 0;
}

void RCS660S::printHexArray(const uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; i++)
    {
        if (data[i] < 0x10)
        {
            // 1桁の場合は先頭に0をつける
            Serial.print("0");
        }
        Serial.print(data[i], HEX); // 16進数で出力
        if (i < len - 1)
        {
            Serial.print(" "); // 最後の要素でなければスペースを出力
        }
    }
    Serial.println(); // 改行を追加
}

/**
 * RC-S660/S for Arduino
 * A library for interfacing with RC-S660/S NFC reader
 * @author tarusake
 */

#ifndef RCS660S_H_
#define RCS660S_H_

// Buffer size constants
#define RCS660S_BUFFER_SIZE 1024
#define RCS660S_SMALL_BUFFER_SIZE 290

// Command constants
#define RCS660S_CLA_DEFAULT 0xFF
#define RCS660S_INS_CARD_COMMAND 0xC2
#define RCS660S_DEFAULT_SYSTEM_CODE 0xFFFF

// CCID message types
#define RCS660S_MSG_TYPE_PC_TO_RDR_ESCAPE 0x6B
#define RCS660S_MSG_TYPE_ABORT 0x72

class RCS660S {
public:
    /**
     * Constructor
     * @param serial Serial interface to use for communication
     */
    RCS660S(Stream &serial = Serial1);

    /**
     * Initialize the RC-S660/S device
     * @return 1 on success, 0 on failure
     */
    int initDevice(void);

    /**
     * Perform polling for NFC cards
     * @param systemCode System code to poll for
     * @return 1 if card found, 0 if no card
     */
    int polling(uint16_t systemCode = RCS660S_DEFAULT_SYSTEM_CODE);

    /**
     * Send command to card and receive response
     * @param command Command data to send
     * @param commandLen Length of command data
     * @param response Buffer to store response
     * @param responseLen Length of response
     * @return 1 on success, 0 on failure
     */
    int cardCommand(
        const uint8_t *command,
        uint8_t commandLen,
        uint8_t response[RCS660S_BUFFER_SIZE],
        uint16_t *responseLen);

    // Device information
    unsigned long timeout;  // Communication timeout in milliseconds
    uint8_t idm[8];        // Card ID information
    uint8_t pmm[8];        // Card manufacturer information

    // Log level enum
    enum LogLevel {
        LOG_ERROR = 0,
        LOG_WARN,
        LOG_INFO,
        LOG_DEBUG
    };

    // Current log level
    LogLevel logLevel;

private:
    Stream *_serial;       // Serial interface
    uint8_t bseq;         // Sequence number for CCID commands

    // Internal helper functions
    uint8_t calcDCS(const uint8_t *data, uint16_t len);
    void writeSerial(const uint8_t *data, uint16_t len);
    int readSerial(uint8_t *data, uint16_t len);
    void flushSerial(void);
    int checkTimeout(unsigned long t0);
    void printHexArray(const uint8_t *data, size_t len);

    // CCID protocol functions
    int send_ccid_command(const uint8_t *command, uint16_t command_len);
    int receive_ccid_response(uint8_t *response, uint16_t *response_len);
    int receive_ack(void);
    int abort_command(void);
    int write_apdu(const uint8_t *data, uint32_t data_len);
};

#endif

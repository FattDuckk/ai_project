#ifndef SERIALCOMM_H
#define SERIALCOMM_H

#include <Arduino.h>
#include "Define.h"
enum CommState
{
  IDLE,
  START,
  CMD,
  END,

  STATE_COUNT
};
class SerialComm
{

public:
    SerialComm();
    void begin();


    void handleCommand(const String &cmd);

    void sendOK();
    void getSerialCmd();
    void handleSerialCmd();
    void serialCmdInit();
    void handleSerialData(char data);

    void sendError(const String &error);
    uint32_t put(uint8_t value);
    uint32_t get(uint8_t *value);
    uint32_t isFull();
    uint32_t isEmpty();
    void reportResult(int reportCode, String result);
    void reportButtonEvent(unsigned char buttonId, unsigned char event);
    bool parseCommand(char *message);

    

private:
    

    String buffer;
    uint32_t head;
	uint32_t tail;
	uint8_t *data;
	uint32_t buffer_size;

    unsigned long lastSerialSendTime = 0;
    const unsigned long serialSendInterval = 10; // Minimum 10 ms between sends

    

    uint8_t bufData[RING_BUFFER_SIZE];
};


#endif

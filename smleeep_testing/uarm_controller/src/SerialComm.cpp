#include "SerialComm.h"

static CommState commState = IDLE;
static unsigned char cmdReceived[COM_LEN_MAX];
static unsigned char cmdIndex = 0;

SerialComm::SerialComm() {
}

void SerialComm::begin(long baud)
{
    Serial.begin(baud);
    while (!Serial)
    {
        ;
    }
    Serial.println("READY");
}



void SerialComm::getSerialCmd()
{
    int data = -1;

    while (Serial.available())
    {
        data = Serial.read();

        if (data != -1)
        {
            put((uint8_t)data);
        }
    }
}

void SerialComm::handleCommand(const String &cmd)
{
    double x, y, z;
    if (cmd.startsWith("MOVE"))
    {
        int count = sscanf(cmd.c_str(), "MOVE %lf %lf %lf", &x, &y, &z);
        if (count == 3)
        {
            sendOK();
        }
        else
        {
            sendError("Unreachable");
        }
    }
    else
    {
        sendError("Bad format");
    }
}

void SerialComm::handleSerialCmd()
{
    uint8_t data = 0;

    while (get(&data))
    {
        handleSerialData(data);
    }
}


void SerialComm::handleSerialData(char data)
{
    static unsigned char cmdCount = 0;

    switch (commState)
    {
    case IDLE:
        if (data == '#' || data == 'G' || data == 'M' || data == 'P')
        {
            commState = START;
            cmdIndex = 0;
            if (data != '#')
            {
                cmdCount = 1; // get cmd code
            }
            else
            {
                cmdCount = 0;
            }

            cmdReceived[cmdIndex++] = data;
        }
        break;

    case START:

        if (cmdIndex >= COM_LEN_MAX)
        {

            commState = IDLE;
        }
        else if (data == '#') // restart
        {
            cmdIndex = 0;
            cmdCount = 0;
            cmdReceived[cmdIndex++] = data;
        }
        else if (data == 'G' || data == 'M' || data == 'P')
        {
            if (cmdCount >= 1) // restart
            {
                cmdIndex = 0;
                cmdReceived[cmdIndex++] = data;
            }
            else
            {
                cmdCount++;
                cmdReceived[cmdIndex++] = data;
            }
        }
        else if (data == '\n')
        {

            cmdReceived[cmdIndex] = '\0';

            parseCommand((char *)cmdReceived);
            commState = IDLE;
        }
        else
        {

            cmdReceived[cmdIndex++] = data;
        }
        break;

    default:
        commState = IDLE;
        break;
    }
}

void SerialComm::serialCmdInit()
{
    data = bufData;
    buffer_size = RING_BUFFER_SIZE;
}

void SerialComm::sendOK()
{
    Serial.println("OK");
}

void SerialComm::sendError(const String &msg)
{
    Serial.print("ERROR: ");
    Serial.println(msg);
}

void SerialComm::reportButtonEvent(unsigned char buttonId, unsigned char event)
{
    char result[RESULT_BUFFER_SIZE];
    snprintf(result, sizeof(result), "B%d V%d", buttonId, event);
    reportResult(REPORT_BUTTON, result);
}

bool SerialComm::parseCommand(char *message)
{
    double value[6];
    int index = 0;
    bool hasSerialNum = false;
    Serial.print("DEBUG: message=");
    Serial.println(message);

    int len = strlen(message);

    char *pch;
    char cmdType;

    // skip white space
    while (len > 0)
    {
        if (isspace(message[len - 1]))
        {
            message[len - 1] = '\0';
        }
        else
        {
            break;
        }

        len--;
    }

    if (len <= 0)
        return false;

    if (message[0] == '#')
    {
        hasSerialNum = true;
    }

    pch = strtok(message, " ");
    while (pch != NULL && index < 6)
    {
        // debugPrint("pch=%s", pch);

        switch (index)
        {
        case 0:
            if (!hasSerialNum)
            {
                cmdType = *(pch);
            }
            value[index] = atof(pch + 1);
            break;

        case 1:
            if (hasSerialNum)
            {
                cmdType = *(pch);
            }
            // debugPrint("cmdType=%d", cmdType);
            value[index] = atof(pch + 1);
            break;

        default:
            value[index] = atof(pch + 1);
            break;
        }

        // debugPrint("value=%f", value[index]);

        pch = strtok(NULL, " ");

        index++;
    }

    int serialNum = 0;
    int cmdCode = 0;
    int parameterCount = 0;
    int valueStartIndex = 0;

    if (hasSerialNum)
    {
        serialNum = value[0];
        cmdCode = value[1];
        parameterCount = index - 2;
        valueStartIndex = 2;
    }
    else
    {
        serialNum = 0;
        cmdCode = value[0];
        parameterCount = index - 1;
        valueStartIndex = 1;
    }

    // switch (cmdType)
    // {
    // case 'G':
    //     HandleMoveCmd(cmdCode, serialNum, parameterCount, value + valueStartIndex);
    //     break;

    // case 'M':
    //     HandleSettingCmd(cmdCode, serialNum, parameterCount, value + valueStartIndex);
    //     break;

    // case 'P':
    //     HandleQueryCmd(cmdCode, serialNum, parameterCount, value + valueStartIndex);
    //     break;

    // }
}

int32_t SerialComm::put(uint8_t value)
{

    if (isFull())
    {
        return 0;
    }

    data[tail] = value;

    tail = (tail + 1) % buffer_size;

    return 1;
}

int32_t SerialComm::get(uint8_t *value)
{

    if (isEmpty())
        return 0;

    *value = data[head];

    head = (head + 1) % buffer_size;

    return 1;
}

int32_t SerialComm::isFull()
{
    return ((tail + 1) % buffer_size) == head ? 1 : 0;
}

int32_t SerialComm::isEmpty()
{
    return head == tail ? 1 : 0;
}

void SerialComm::reportResult(int reportCode, String result)
{

    Serial.print("@");
    Serial.print(reportCode);
    Serial.print(" ");
    Serial.println(result);
}

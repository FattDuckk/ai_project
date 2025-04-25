#ifndef ROBOTIC_ARM_H
#define ROBOTIC_ARM_H

#include <Arduino.h>
#include "Define.h"

struct Pos
{
    double x;
    double y;
    double z;
};

class RoboticArm
{

public:
    RoboticArm();

    void begin();

    void moveToAngles(double *angles);

    bool moveToXYZ(Pos position);

private:
    int pins[SERVO_COUNT] = {SERVO_ROT_PIN, SERVO_LEFT_PIN, SERVO_RIGHT_PIN, SERVO_HAND_ROT_PIN};
    double angles[SERVO_COUNT];
    double initialiseAngles[SERVO_COUNT] = {0};

    // Maps 0-180 degrees to 1000-2000us pulse
    void writeServoPulse(int pin, int angle);
};

#endif

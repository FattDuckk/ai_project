#include "RoboticArm.h"

RoboticArm::RoboticArm()
{
    RoboticArm::begin();
    RoboticArm::moveToAngles(initialiseAngles);
}

void RoboticArm::begin()
{
    for (int i = 0; i < SERVO_COUNT; i++)
    {
        Serial.print(pins[i]);
        pinMode(pins[i], OUTPUT);
        digitalWrite(pins[i], LOW);
    }
}

void RoboticArm::moveToAngles(double* angles)
{
    // You may want to call this in a timed loop for continuous pulses
    for (int i = 0; i < SERVO_COUNT; i++)
    {
        writeServoPulse(pins[i], angles[i]);
    }
}

bool RoboticArm::moveToXYZ(Pos position) {
    // Extract position coordinates
    double x = position.x;
    double y = position.y;
    double z = position.z;

    // Compute planar distance from base to target (projection on XY plane)
    float stretch = sqrt(x * x + y * y);
    float rotAngle = atan2(y, x) * 180.0 / PI; // Rotation angle in degrees

    // Compute the distance from the shoulder joint to the target (XZ plane)
    float h = sqrt(stretch * stretch + z * z);

    // Check if the target position is reachable
    if (h > (L1 + L2)) {
        Serial.println("Target position is unreachable.");
        return false;
    }

    // Calculate angles using inverse kinematics (law of cosines)
    float a1 = acos((L1 * L1 + h * h - L2 * L2) / (2 * L1 * h)); // Shoulder angle
    float a2 = acos((L1 * L1 + L2 * L2 - h * h) / (2 * L1 * L2)); // Elbow angle
    float angleShoulder = atan2(z, stretch) + a1; // Shoulder angle with offset
    float angleElbow = PI - a2; // Elbow angle

    // Convert angles to degrees and constrain them to servo limits
    int degRot = constrain(static_cast<int>(rotAngle), 0, 180);
    int degShoulder = constrain(static_cast<int>(angleShoulder * 180.0 / PI), 0, 180);
    int degElbow = constrain(static_cast<int>(angleElbow * 180.0 / PI), 0, 180);

    // Store the calculated angles
    angles[0] = degRot;
    angles[1] = degShoulder;
    angles[2] = degElbow;

    // Move the robotic arm to the calculated angles
    moveToAngles(angles);

    // Log the movement for debugging
    Serial.print("Moving to XYZ: ");
    Serial.print("X=");
    Serial.print(x);
    Serial.print(", Y=");
    Serial.print(y);
    Serial.print(", Z=");
    Serial.print(z);
    Serial.print(" | Angles: Rot=");
    Serial.print(degRot);
    Serial.print(", Shoulder=");
    Serial.print(degShoulder);
    Serial.print(", Elbow=");
    Serial.println(degElbow);

    return true;
}


void RoboticArm::writeServoPulse(int pin, int angle)
{
    angle = constrain(angle, 0, 180);
    int pulse = map(angle, 0, 180, 1000, 2000);

    // Send multiple pulses to ensure servo moves and holds position
    for (int i = 0; i < 20; i++)  // ~20 pulses over 400ms
    {
        digitalWrite(pin, HIGH);
        delayMicroseconds(pulse);
        digitalWrite(pin, LOW);
        delay(20 - pulse / 1000); // Wait for the rest of the 20ms period
    }
}




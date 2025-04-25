#include <Arduino.h>
#include "SerialComm.h"
#include "RoboticArm.h"

long baud = 115200; // Default baud rate

RoboticArm arm;
SerialComm comm;

void setup() {
    comm.begin(baud); // Adjust baud rate as needed
	Serial.begin(baud);
	Serial.println("hi");	// report ready
}

void loop() {
	comm.getSerialCmd();
	comm.handleSerialCmd();

}

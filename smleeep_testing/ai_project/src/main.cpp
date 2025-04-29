#include <Arduino.h>
// #include "SerialComm.h"
// #include "RoboticArm.h"
#include "Controller.h"


// RoboticArm arm;
// SerialComm comm;
Controller controller;
void setup()
{
	Serial.begin(115200);
	delay(1000); // Optional, sometimes helps
	Serial.println("hi"); // report ready
	// comm.begin(); // Adjust baud rate as needed
	// arm.begin(); // Initialize the robotic arm
	// comm.serialCmdInit();
	controller.begin(); // Initialize the controller
	pinMode(4, INPUT);
	pinMode(7, INPUT);
}

void loop()
{
	// comm.getSerialCmd();
	controller.readCommand();
	int state4 = digitalRead(4);
	int state7 = digitalRead(7);

	if (state4 == LOW)
	{
		Serial.println("Pin 4 is LOW");
	}

	if (state7 == LOW)
	{
		Serial.println("Pin 7 is LOW");
	}

	delay(250);
	digitalWrite(4, HIGH);
	digitalWrite(7, HIGH);
	
}

// void setup() {
// 	Serial.begin(115200);
// 	delay(1000);  // Optional, sometimes helps
//   }

//   void loop() {
// 	Serial.println("Hello from Arduino!");
// 	delay(1000); // send once per second
//   }

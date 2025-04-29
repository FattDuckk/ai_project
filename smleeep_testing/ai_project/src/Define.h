#ifndef _DEFINE_H_
#define _DEFINE_H_

#define METAL // Define robot type

#ifdef METAL
#define DEVICE_NAME "ROBOT"
#elif defined(ROBOT)
#define DEVICE_NAME "Robot"
#else
#define DEVICE_NAME "UNKNOWN"
#endif

#ifdef ROBOT

#define BUZZER 3 // HIGH = ON
#define BT_DETECT_PIN 3

#define LIMIT_SW 2 // LIMIT Switch Button

#define BTN_D4 7 // LOW = Pressed
#define BTN_D7 4 // LOW = Pressed

#define SYS_LED 6

#define PUMP_EN 5
#define GRIPPER 5
#define GRIPPER_FEEDBACK A7
#define PUMP_FEEDBACK A7

#define POWER_DETECT A6

#define SERVO_ROT_PIN 11
#define SERVO_LEFT_PIN 13
#define SERVO_RIGHT_PIN 12
#define SERVO_HAND_ROT_PIN 10

#define SERVO_ROT_ANALOG_PIN 2
#define SERVO_LEFT_ANALOG_PIN 0
#define SERVO_RIGHT_ANALOG_PIN 1
#define SERVO_HAND_ROT_ANALOG_PIN 3

#elif defined(METAL)

#define BUZZER 3   // HIGH = ON
#define LIMIT_SW 2 // LIMIT Switch Button

#define BTN_D4 4 // LOW = Pressed
#define BTN_D7 7 // LOW = Pressed

#define PUMP_EN 6  // HIGH = Valve OPEN
#define VALVE_EN 5 // HIGH = Pump ON
#define GRIPPER 9  // LOW = Catch
#define GRIPPER_FEEDBACK A6

#define SERVO_ROT_PIN 11
#define SERVO_LEFT_PIN 13
#define SERVO_RIGHT_PIN 12
#define SERVO_HAND_ROT_PIN 10

#define SERVO_ROT_ANALOG_PIN 2
#define SERVO_LEFT_ANALOG_PIN 0
#define SERVO_RIGHT_ANALOG_PIN 1
#define SERVO_HAND_ROT_ANALOG_PIN 3

#define SERVO_COUNT 4

#define MIN_PULSE_WIDTH 500      // the shortest pulse sent to a servo
#define MAX_PULSE_WIDTH 2500     // the longest pulse sent to a servo
#define DEFAULT_PULSE_WIDTH 1500 // default pulse width when servo is attached
#define REFRESH_INTERVAL 20000   // minumim time to refresh servos in microseconds

#define SERVOS_PER_TIMER 12 // the maximum number of servos controlled by one timer
#define SERVO_MIN 500       // minimum value in uS for this servo
#define SERVO_MAX 2500      // maximum value in uS for this servo

#define L1 148.25
#define L2 160.2

#define RESULT_BUFFER_SIZE  50
#define REPORT_POS        3 
#define REPORT_BUTTON     4

#define RING_BUFFER_SIZE    48
#define COM_LEN_MAX   48


#endif

#endif
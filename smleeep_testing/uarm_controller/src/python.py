import serial
import time

# Open the serial connection to the uArm control board
ser = serial.Serial('/dev/ttyUSB0', 9600)  # Replace with your serial port
time.sleep(2)  # Wait for the uArm to reset

def set_servo_angle(servo, angle):
    # Send the command to set the servo angle (e.g., 'S90,2' for Servo 2)
    command = f"S{angle},{servo}\n"
    ser.write(command.encode())

def main():
    try:
        while True:
            # Example: Sweep Servo 2 from 0 to 180 degrees and back
            for angle in range(0, 181, 10):  # Sweep from 0 to 180
                set_servo_angle(2, angle)  # Control Servo 2
                print(f"Setting Servo 2 to {angle} degrees")
                time.sleep(1)
            
            for angle in range(180, -1, -10):  # Sweep back from 180 to 0
                set_servo_angle(2, angle)  # Control Servo 2
                print(f"Setting Servo 2 to {angle} degrees")
                time.sleep(1)

    except KeyboardInterrupt:
        print("Exiting program.")
    finally:
        ser.close()

if __name__ == "__main__":
    main()

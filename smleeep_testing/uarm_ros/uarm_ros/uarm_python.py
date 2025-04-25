import serial
import time
import threading

class UArmController:
    def __init__(self, port="/dev/ttyUSB0", baud_rate=115200):
        """Initialize the serial connection."""
        self.port = port
        self.baud_rate = baud_rate
        self.ser = None
        self.analog_value = 0
        self.running = True  # Flag to stop thread safely

        try:
            self.ser = serial.Serial(self.port, self.baud_rate, timeout=1)
            print(f"Connected to {self.port} at {self.baud_rate} baud.")
        except Exception as e:
            print(f"Error opening serial port: {e}")
            self.ser = None

        time.sleep(2)  # Give the connection time to settle

    def send_gcode(self, x, y, z):
        """Send G-code commands to the uArm."""
        if self.ser:
            gcode_command = f"#1 G0 X{x} Y{y} Z{z} F100\n"
            self.ser.write(gcode_command.encode('utf-8'))
            print(f"Sent: {gcode_command.strip()}")

    def move_uarm_sequence(self, sequence):
        """Simulate a movement sequence for the uArm."""
        print("Sending movement sequence to uArm...")
        # Replace these with actual serial commands to the uArm
        if sequence == 1:
            self.send_gcode(0, 150, 150)
            time.sleep(0.5)
            self.send_gcode(100, 150, 150)
            time.sleep(0.5)
            self.send_gcode(-100, 200, 150)
            time.sleep(0.5)
        elif sequence == 2:
            self.send_gcode(-100, 200, 150)
            time.sleep(0.5)
            self.send_gcode(100, 150, 150)
            time.sleep(0.5)
            self.send_gcode(0, 150, 150)
            time.sleep(0.5)
        print("Movement sequence complete.")
    

    def run(self):
        """Listen for button presses from the Arduino."""
        if not self.ser:
            print("Serial connection not established.")
            return

        print("Listening for button presses from Arduino...")
        try:
            while True:
                if self.ser.in_waiting:  # Check if data is available
                    line = self.ser.readline().decode().strip()  # Read and decode the line
                    if "Pin 4 is LOW" in line:
                        print("Button 4 pressed.")
                        self.move_uarm_sequence(1)
                    elif "Pin 7 is LOW" in line:
                        print("Button 7 pressed.")
                        self.move_uarm_sequence(2)
                    else:
                        print(f"Unknown output: {line}")
                time.sleep(0.05)  # Small delay to avoid high CPU usage
        except KeyboardInterrupt:
            print("Exiting...")
            self.ser.close()

def main():
    # Create an instance of the UArmController
    controller = UArmController(port="/dev/ttyUSB0", baud_rate=115200)
    controller.run()

if __name__ == "__main__":
    main()

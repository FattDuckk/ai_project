import serial
import time

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

        time.sleep(1)  # Give the connection time to settle

    def send_gcode(self, x, y, z):
        """Send G-code commands to the uArm."""
        if self.ser:
            gcode_command = f"MOVE {x} {y} {z}\n"
            self.ser.write(gcode_command.encode('utf-8'))
            print(f"Sent: {gcode_command.encode('utf-8')}")
            print(self.ser.readline().decode().strip()) # Read and decode the line

    def move_uarm_sequence(self, sequence):
        """Simulate a movement sequence for the uArm."""
        print("Sending movement sequence to uArm...")
        # Replace these with actual serial commands to the uArm
        if sequence == 1:
            self.send_gcode(0, 150, 150)
            time.sleep(1)
            self.send_gcode(100, 150, 150)
            time.sleep(1)
            self.send_gcode(-100, 200, 150)
            time.sleep(1)
        elif sequence == 2:
            self.send_gcode(-100, 200, 150)
            time.sleep(1)
            self.send_gcode(100, 150, 150)
            time.sleep(1)
            self.send_gcode(0, 150, 150)
            time.sleep(1)
        print("Movement sequence complete.")
    

    def run(self):
        """Listen for button presses from the Arduino."""
        if not self.ser:
            print("Serial connection not established.")
            return

        print("Listening for button presses from Arduino...")
        try:
            self.send_gcode(0, 150, 150)
            while True:
                line = self.ser.readline().decode('utf-8', errors='replace').strip()  # Read and decode the line
                if "Pin 4 is LOW" in line:
                    print("Button 4 pressed.")
                    self.move_uarm_sequence(1)
                elif "Pin 7 is LOW" in line:
                    print("Button 7 pressed.")
                    self.move_uarm_sequence(1)
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

# import serial
# import time

# port = "/dev/ttyUSB0"

# ser = serial.Serial(port, 115200)  # Replace COM3 with your port

# def send_command(cmd):
#     ser.write(cmd.encode('utf-8'))
#     time.sleep(0.1)
#     response = ser.readline().decode().strip()
#     print("Arduino:", response)

# # Move to angles
# # send_command("MOVE_ANGLES 90 90 90 90")

# # Move to XYZ
# try:
#     while True:
#         line = ser.read()
#         try:
#             send_command("MOVE 0 0 0 \n")
#             decoded_line = line.decode('utf-8', errors='replace').strip()
#             print(decoded_line)
#         except UnicodeDecodeError:
#             print("Received non-UTF-8 data:", line)
        
#         if ser.in_waiting:  # Check if data is available
#             send_command("MOVE 0 0 0 \n")
#             time.sleep(0.05)
#             send_command("MOV1 \n")
#             time.sleep(0.05)
#         time.sleep(0.05)  # Small delay to avoid high CPU usage
# except KeyboardInterrupt:
#     print("Exiting...")
#     ser.close()

# import serial
# import time

# # Open serial connection
# ser = serial.Serial(port, 115200, timeout=1)  # COM port and baud must match Arduino

# time.sleep(1)  # <<-- IMPORTANT: wait for Arduino reset

# # Clear the input buffer
# ser.flush()

# while True:
#     if ser.in_waiting > 0:
#         line = ser.readline().decode('utf-8').rstrip()
#         print(f"Received: {line}")





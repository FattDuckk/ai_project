import time
import serial
from pyfirmata import Arduino, util

class UArmController:
    def __init__(self, arduino_port="/dev/ttyUSB0", uarm_port="/dev/ttyUSB0", baud_rate=115200):
        try:
            self.ser = serial.Serial(arduino_port, baudrate=baud_rate, timeout=1)
            print(f"Connected to {arduino_port} at {baud_rate} baud.")
        except Exception as e:
            print(f"Error opening serial port: {e}")
            self.ser = None  

        # Initialize pyFirmata for button input
        self.board = Arduino(arduino_port)
        self.button4 = self.board.get_pin('d:4:i')
        self.button7 = self.board.get_pin('d:7:i')

        # Enable internal pull-up resistors
        self.board.digital[4].mode = 2  # Set pin 4 to INPUT_PULLUP
        self.board.digital[7].mode = 2  # Set pin 7 to INPUT_PULLUP

        it = util.Iterator(self.board)
        it.start()
        time.sleep(1)

        # Initialize serial connection to uArm
        # self.uarm_serial = serial.Serial(uarm_port, baud_rate, timeout=1)
        # time.sleep(2)

        print("UArmController initialized")

    def send_gcode(self, x, y, z):
        gcode_command = f"#1 G0 X{x} Y{y} Z{z} F100\n"
        self.ser.write(gcode_command.encode('utf-8'))
        print(f"Sent: {gcode_command.strip()}")

    def move_uarm_sequence(self):
        print("Running movement sequence...")
        self.send_gcode(0, 50, 50)
        time.sleep(0.5)
        # self.send_gcode(100, 150, 150)
        # time.sleep(0.5)
        # You could add pump control here if needed
        # self.send_gcode(-100, 200, 150)
        # time.sleep(0.5)
        print("Sequence complete.\n")

    def run(self):
        print("Listening for button presses (press Ctrl+C to exit)...")
        try:
            self.ser.write(b"#1 G0 X0 Y0 Z0 F100\r\n")
            response = self.ser.readline().decode('utf-8').strip()
            print(f"Response: {response}")
            while True:
                print(self.button4.read(), self.button7.read())
                if self.button4.read():
                    print("Button on pin 4 pressed.")
                    self.move_uarm_sequence()
                    while self.button4.read():
                        time.sleep(0.01)

                if self.button7.read():
                    print("Button on pin 7 pressed.")
                    self.move_uarm_sequence()
                    while self.button7.read():
                        time.sleep(0.01)

                time.sleep(0.05)

        except KeyboardInterrupt:
            print("Exiting...")
            self.cleanup()

    def cleanup(self):
        self.board.exit()
        self.ser.close()


def main():
    controller = UArmController()
    controller.run()

if __name__ == "__main__":
    main()

# import serial
# import time
# from pyfirmata import Arduino, util

# # Setup
# board = Arduino('/dev/ttyUSB0')  # Replace with your port, e.g. COM3 on Windows
# # Configure serial connection
# # PORT = "/dev/ttyUSB0"  # Update with the correct COM port
# # BAUD_RATE = 115200

# # try:
# #     board = serial.Serial(PORT, BAUD_RATE, timeout=1)
# #     print(f"Connected to {PORT} at {BAUD_RATE} baud.")
# # except Exception as e:
# #     print(f"Error opening serial port: {e}")
# #     ser = None

# # Set pin modes
# button4 = board.get_pin('d:4:i')  # digital pin 4 as input
# button7 = board.get_pin('d:7:i')  # digital pin 7 as input
# # Turn on pin 13
# # board.write(b'H')

# # # Turn off pin 13
# # board.write(b'L')

# # Start iterator to avoid buffer overflow
# it = util.Iterator(board)
# it.start()
# time.sleep(1)  # Give some time to initialize


# def send_gcode(x, y, z):
#     """ Send G-code command to the uArm """
#     if board:
#         gcode_command = f"#1 G0 X{x} Y{y} Z{z} F100\n"
#         board.write(gcode_command.encode('utf-8'))
#         print(f"Sent: {gcode_command.strip()}")

# def move_uarm_sequence():
#     print("Moving uArm...")
#     # Replace these with actual serial commands to the uArm if connected separately
#     print("moveTo(0, 150, 150)")
#     send_gcode(0, 150, 150)
#     time.sleep(0.5)
#     print("moveTo(100, 150, 150)")
#     send_gcode(100, 150, 150)
#     time.sleep(0.5)
#     print("pumpOn()")
#     time.sleep(0.5)
#     print("moveTo(-100, 200, 150)")
#     send_gcode(-100, 200, 150)
#     time.sleep(0.5)
#     print("pumpOff()")
#     print("Sequence done.\n")

# try:
#     while True:
#         print("Ready, Press buttons connected to pin 4 or 7.")
#         while True:
#             if button4.read() == True:
#                 print("Button on pin 4 pressed.")
#                 move_uarm_sequence()
#                 while button4.read() == True:
#                     time.sleep(0.01)  # Wait for release to avoid repeats

#             if button7.read() == True:
#                 print("Button on pin 7 pressed.")
#                 move_uarm_sequence()
#                 while button7.read() == True:
#                     time.sleep(0.01)  # Wait for release to avoid repeats

#             time.sleep(0.05)

# except KeyboardInterrupt:
#     print("Exiting...")
#     board.exit()












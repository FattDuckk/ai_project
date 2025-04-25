import serial
import tkinter as tk

# Configure serial connection
PORT = "/dev/ttyUSB0"  # Update with the correct COM port
BAUD_RATE = 115200

try:
    ser = serial.Serial(PORT, BAUD_RATE, timeout=1)
    print(f"Connected to {PORT} at {BAUD_RATE} baud.")
except Exception as e:
    print(f"Error opening serial port: {e}")
    ser = None

# Default X, Y, Z positions
pos_x, pos_y, pos_z = 100, 100, 100  # Set initial default positions

def send_gcode():
    """ Send G-code command to the uArm """
    if ser:
        gcode_command = f"#1 G0 X{pos_x} Y{pos_y} Z{pos_z} F100\n"
        ser.write(gcode_command.encode('utf-8'))
        print(f"Sent: {gcode_command.strip()}")

def update_x(value):
    """ Update X position and send command """
    global pos_x
    pos_x = int(value)
    send_gcode()

def update_y(value):
    """ Update Y position and send command """
    global pos_y
    pos_y = int(value)
    send_gcode()

def update_z(value):
    """ Update Z position and send command """
    global pos_z
    pos_z = int(value)
    send_gcode()

# GUI Setup
root = tk.Tk()
root.title("uArm Position Controller")

# X Slider
tk.Label(root, text="X Position").pack()
slider_x = tk.Scale(root, from_=0, to=200, orient=tk.HORIZONTAL, command=update_x)
slider_x.set(pos_x)
slider_x.pack()

# Y Slider
tk.Label(root, text="Y Position").pack()
slider_y = tk.Scale(root, from_=0, to=200, orient=tk.HORIZONTAL, command=update_y)
slider_y.set(pos_y)
slider_y.pack()

# Z Slider
tk.Label(root, text="Z Position").pack()
slider_z = tk.Scale(root, from_=0, to=200, orient=tk.HORIZONTAL, command=update_z)
slider_z.set(pos_z)
slider_z.pack()

# Run GUI
root.mainloop()

# Close serial port when GUI is closed
if ser:
    ser.close()
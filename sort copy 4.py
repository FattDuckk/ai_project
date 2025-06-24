import pygame
import time
# Initialise Pygame's joystick module
pygame.init()
pygame.joystick.init()

# Connect to the first available joystick
joystick = pygame.joystick.Joystick(0)
joystick.init()

print(f"🎮 Connected to: {joystick.get_name()}")
time.sleep(1)  # Allow time for joystick to initialize

# Mapping: You can update this later to suit your project
axis_names = ["LX", "LY", "RX", "RY", "L2", "R2"]
button_names = ["Square", "Cross", "Circle", "Triangle", "L1", "R1", "L2", "R2", "Share", "Options", "L3", "R3", "PS", "Touchpad"]

try:
    while True:
        pygame.event.pump()

        # Read axes
        for i in range(joystick.get_numaxes()):
            axis = joystick.get_axis(i)
            if abs(axis) > 0.1:  # Deadzone
                print(f"Axis {i}: {axis:.2f}")

        # Read buttons
        for i in range(joystick.get_numbuttons()):
            button = joystick.get_button(i)
            if button:
                print(f"Button {i} ({button_names[i] if i < len(button_names) else 'Unknown'}): Pressed")

        pygame.time.wait(100)  # Delay to prevent spam
except KeyboardInterrupt:
    print("👋 Exiting")
    pygame.quit()

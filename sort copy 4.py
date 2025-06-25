import pygame
import time

# Initialize Pygame and joystick module
pygame.init()
pygame.joystick.init()

# Connect to the first available joystick
if pygame.joystick.get_count() == 0:
    print("❌ No joystick detected. Please connect your PS5 controller.")
    exit()

joystick = pygame.joystick.Joystick(0)
joystick.init()

print(f"🎮 Connected to: {joystick.get_name()}")
time.sleep(1)

# Optional label maps
axis_names = {
    0: "Left Stick X",
    1: "Left Stick Y",
    # 2: "Left Stick Y111",
    3: "Right Stick X",
    4: "Right Stick Y",
    # 5: "R2 Trigger"
}

button_names = {
    0: "X",
    1: "Circle",
    2: "Triangle",
    3: "Square",
    4: "L1",
    5: "R1",
    6: "L2",
    7: "R2",
    8: "Share",
    9: "Options",
    10: "PS",
    11: "L3",
    12: "R3",
    13: "Touchpad"
}

dpad_map = {
    (0, 1): "Up",
    (0, -1): "Down",
    (-1, 0): "Left",
    (1, 0): "Right",
    (1, 1): "Up-Right",
    (-1, 1): "Up-Left",
    (1, -1): "Down-Right",
    (-1, -1): "Down-Left",
}

try:
    print("📡 Listening for input... (Ctrl+C to quit)\n")
    while True:
        pygame.event.pump()

        # Read axis (sticks and triggers)
        for i in range(joystick.get_numaxes()):
            val = joystick.get_axis(i)
            if abs(val) > 0.1:  # Deadzone
                axis_label = axis_names.get(i, f"Axis {i}")
                print(f"{axis_label}: {val:.2f}")

        # Read buttons
        for i in range(joystick.get_numbuttons()):
            if joystick.get_button(i):
                btn_label = button_names.get(i, f"Button {i}")
                print(f"🔘 {btn_label} Pressed")

        # Read D-Pad
        for i in range(joystick.get_numhats()):
            hat = joystick.get_hat(i)
            if hat != (0, 0):
                direction = dpad_map.get(hat, f"{hat}")
                print(f"🕹️ D-Pad: {direction}")

        pygame.time.wait(100)

except KeyboardInterrupt:
    print("\n👋 Exiting...")
    pygame.quit()

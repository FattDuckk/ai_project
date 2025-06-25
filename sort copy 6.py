import pybullet as p
import time
import cv2
import numpy as np

from env import UArmEnv
from ultralytics import YOLO
from stable_baselines3 import PPO
import pygame

# Init pygame for controller
pygame.init()
pygame.joystick.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()

# Initialise environment
env = UArmEnv(render=True)
p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
env.reset()

# Load PPO model
ppo_model = PPO.load("./ppo_checkpoints/ppo_checkpoint_1660000")

# Track state
mode = "Idle"
camera_enabled = True

# Show OpenCV HUD with current mode + camera
def show_hud(current_mode, cam_on, controller_mode=None):
    hud = np.zeros((240, 450, 3), dtype=np.uint8)

    cv2.putText(hud, "CONTROLS", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    cv2.putText(hud, "[S] Start Sorting", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
    cv2.putText(hud, "[R] Reset Environment", (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 255, 200), 1)
    cv2.putText(hud, "[M] Manual Mode", (10, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 230, 180), 1)
    cv2.putText(hud, "[B] Back (Exit Manual)", (10, 220), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 255), 1)
    cv2.putText(hud, "[C] Toggle Camera", (10, 160), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (180, 230, 255), 1)
    cv2.putText(hud, "[Q] Quit", (10, 190), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 200, 200), 1)

    mode_text = f"{current_mode}"
    if current_mode == "Manual" and controller_mode:
        mode_text += f" ({controller_mode.title()})"
    cv2.putText(hud, f"MODE: {mode_text}", (180, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 100), 2)

    cam_status = "ON" if cam_on else "OFF"
    cam_color = (100, 255, 100) if cam_on else (100, 100, 255)
    cv2.putText(hud, f"CAMERA: {cam_status}", (180, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, cam_color, 2)

    cv2.imshow("Legend", hud)
    cv2.waitKey(1)

# Manual mode using built-in sliders
import pygame

# Initialize pygame for PS5 controller and keyboard
pygame.init()
pygame.joystick.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()
print(f"🎮 Controller detected: {joystick.get_name()}")

def manual_mode():
    print("🛠️ Entered Manual Mode (press [Z] to toggle input | [B] to exit)")
    step_count = 0
    print_interval = 10
    control_mode = "slider"  # or "controller"
    joint_angles = [0.0, 0.8, -1.57, 0.77, 0]  # default positions

    def deadzone(val, threshold=0.1):
        return val if abs(val) > threshold else 0.0

    gripper_state = False
    prev_x_pressed = False

    while True:
        pygame.event.pump()
        show_hud("Manual", camera_enabled, control_mode)


        keys = p.getKeyboardEvents()
        if ord('b') in keys and keys[ord('b')] & p.KEY_WAS_TRIGGERED:
            print("⬅️ Exiting Manual Mode")
            return
        elif ord('z') in keys and keys[ord('z')] & p.KEY_WAS_TRIGGERED:
            control_mode = "controller" if control_mode == "slider" else "slider"
            print(f"🔁 Switched to {control_mode} mode")

        if control_mode == "controller":
            ly = deadzone(joystick.get_axis(1))
            ry = deadzone(joystick.get_axis(4))
            hat_x, _ = joystick.get_hat(0)

            joint_angles[0] += hat_x * 0.05
            joint_angles[1] += ly * 0.05
            joint_angles[2] += ry * 0.05

            joint_angles[0] = np.clip(joint_angles[0], -np.pi/2, np.pi/2)
            joint_angles[1] = np.clip(joint_angles[1], -np.pi*0.4, np.pi)
            joint_angles[2] = np.clip(joint_angles[2], -np.pi/2, np.pi*0.12)

            # Toggle gripper with X button (button 0)
            x_pressed = joystick.get_button(0)
            if x_pressed and not prev_x_pressed:
                env.toggle_gripper_func()
                gripper_state = not gripper_state
                print("🗜️ Gripper", "Closed" if gripper_state else "Opened")
            prev_x_pressed = x_pressed

            action = [joint_angles[0], joint_angles[1], joint_angles[2], 0]
            obs, reward, done, info = env.step(action)

        else:
            obs, reward, done, info = env.step()
            time.sleep(0.01)
            if camera_enabled:
                env.get_camera_image()
            continue

        if step_count % print_interval == 0:
            ee_position = obs[3:6]
            print(f"[{control_mode}] Joints: {np.round(joint_angles, 3)} | EE: {np.round(ee_position, 3)}")

        if camera_enabled:
            env.get_camera_image()

        step_count += 1
        time.sleep(0.01)



# Move using PPO
def move_to(x, y, z):
    env.set_goal([x, y, z])
    obs = env._get_obs()
    done = False
    while not done:
        action, _ = ppo_model.predict(obs)
        obs, reward, done, info = env.step(action)
        if camera_enabled:
            env.get_camera_image()
        time.sleep(0.05)
    return action

# Sorting routine
def run_sorting():
    topdown_action = [0.0, 0.8, -1.57, 0]
    for _ in range(10):
        env.step(topdown_action)

    env.get_camera_image()
    env.capture_image_label_and_gt("./yolo/test_dataset", image_index=0)

    image_path = "./yolo/test_dataset/images/image_0000.png"
    yolo_model = YOLO("./yolo/runs/detect/train3/weights/best.pt")
    results = yolo_model(image_path)

    X_MIN, X_MAX = -0.170, 0.210
    Y_MIN, Y_MAX = -0.490, -0.120

    predicted_boxes = []
    for box in results[0].boxes:
        cls = int(box.cls[0])
        x_center_n, y_center_n = map(float, box.xywhn[0][:2])
        x_world = X_MIN + (1.0 - x_center_n) * (X_MAX - X_MIN)
        y_world = Y_MIN + y_center_n * (Y_MAX - Y_MIN)
        predicted_boxes.append((cls, x_world, y_world))

    goal_positions = {
        0: env.goal_pos_red,
        1: env.goal_pos_green,
        2: env.goal_pos_blue
    }

    for cls, x, y in predicted_boxes:
        print(f"Handling Class {cls} at ({x:.5f}, {y:.5f})")
        move_to(x, y, 0.025)
        env.toggle_gripper_func()
        time.sleep(0.07)
        move_to(x, y, 0.06)
        move_to(*goal_positions[cls])
        env.toggle_gripper_func()
        time.sleep(0.07)

    print("✅ All boxes sorted!")

# Main loop
print("🎮 Controls: [S] Start | [R] Reset | [M] Manual | [C] Camera | [B] Exit Manual | [Q] Quit")

try:
    while True:
        show_hud(mode, camera_enabled)
        keys = p.getKeyboardEvents()

        if ord('s') in keys and keys[ord('s')] & p.KEY_WAS_TRIGGERED:
            print("▶️ 's' pressed — start sorting")
            mode = "Sorting"
            show_hud(mode, camera_enabled)
            run_sorting()
            mode = "Idle"

        if ord('r') in keys and keys[ord('r')] & p.KEY_WAS_TRIGGERED:
            print("🔁 'r' pressed — reset")
            env.reset()
            mode = "Idle"

        if ord('m') in keys and keys[ord('m')] & p.KEY_WAS_TRIGGERED:
            print("🛠️ 'm' pressed — manual mode")
            mode = "Manual"
            show_hud(mode, camera_enabled)
            manual_mode()
            mode = "Idle"

        if ord('c') in keys and keys[ord('c')] & p.KEY_WAS_TRIGGERED:
            camera_enabled = not camera_enabled
            print(f"📷 Camera toggled {'ON' if camera_enabled else 'OFF'}")

        if ord('q') in keys and keys[ord('q')] & p.KEY_WAS_TRIGGERED:
            print("❌ 'q' pressed — quit")
            break

        time.sleep(0.05)

except KeyboardInterrupt:
    print("👋 Interrupted. Closing.")
finally:
    env.close()
    cv2.destroyAllWindows()

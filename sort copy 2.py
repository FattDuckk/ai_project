from env import UArmEnv
from ultralytics import YOLO
from stable_baselines3 import PPO
import pybullet as p
import time

# Initialise environment
env = UArmEnv(render=True)
p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
env.reset()

# Create fake panel labels as sliders
p.addUserDebugParameter("======== CONTROLS ========", 1, 1, 1)
p.addUserDebugParameter("[S] Start Sorting", 1, 1, 1)
p.addUserDebugParameter("[R] Reset Environment", 1, 1, 1)
p.addUserDebugParameter("[Q] Quit", 1, 1, 1)



# Load PPO model
ppo_model = PPO.load("./ppo_checkpoints/ppo_checkpoint_1660000")

def move_to(x, y, z):
    env.set_goal([x, y, z])
    obs = env._get_obs()
    done = False
    while not done:
        action, _ = ppo_model.predict(obs)
        obs, reward, done, info = env.step(action)
        # env.get_camera_image()
        time.sleep(0.05)
    return action

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
        time.sleep(0.02)
        move_to(x, y, 0.06)
        move_to(*goal_positions[cls])
        env.toggle_gripper_func()
        time.sleep(0.02)

    print("✅ All boxes sorted!")

print("🎮 Controls: [S] Start | [R] Reset | [Q] Quit")

try:
    while True:
        keys = p.getKeyboardEvents()

        if ord('s') in keys and keys[ord('s')] & p.KEY_WAS_TRIGGERED:
            print("▶️ 's' pressed — start sorting")
            run_sorting()

        if ord('r') in keys and keys[ord('r')] & p.KEY_WAS_TRIGGERED:
            print("🔁 'r' pressed — reset")
            env.reset()

        if ord('q') in keys and keys[ord('q')] & p.KEY_WAS_TRIGGERED:
            print("❌ 'q' pressed — quit")
            break

        time.sleep(0.05)

except KeyboardInterrupt:
    print("👋 Interrupted. Closing.")
finally:
    env.close()

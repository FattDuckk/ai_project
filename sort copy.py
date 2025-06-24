from env import UArmEnv
from ultralytics import YOLO
from stable_baselines3 import PPO
import time

# Initialise environment
env = UArmEnv(render=True)
env.reset()


# Step 1: Move to top-down camera view
topdown_action = [0.0, 0.8, -1.57, 0]
for _ in range(10):
    env.step(topdown_action)

# Step 2: Capture image
env.get_camera_image()
env.capture_image_label_and_gt("./yolo/test_dataset", image_index=0)
image_path = "./yolo/test_dataset/images/image_0000.png"

# Step 3: Run YOLO prediction
yolo_model = YOLO("./yolo/runs/detect/train3/weights/best.pt")  # use your best model
results = yolo_model(image_path)

# Coordinate bounds for reconversion
X_MIN, X_MAX = -0.170, 0.210
Y_MIN, Y_MAX = -0.490, -0.120

# Convert YOLO predictions to world coordinates
predicted_boxes = []
for box in results[0].boxes:
    cls = int(box.cls[0])
    x_center_n, y_center_n = map(float, box.xywhn[0][:2])

    x_world = X_MIN + (1.0 - x_center_n) * (X_MAX - X_MIN)
    y_world = Y_MIN + y_center_n * (Y_MAX - Y_MIN)

    predicted_boxes.append((cls, x_world, y_world))

# Define goal positions by class
goal_positions = {
    0: env.goal_pos_red,
    1: env.goal_pos_green,
    2: env.goal_pos_blue
}

# Step 4: Iterate through all predictions

ppo_model = PPO.load("./ppo_checkpoints/ppo_checkpoint_1660000")

def move_to(x, y, z):
    env.set_goal([x, y, z])  # hover height
    obs = env._get_obs()

    done = False
    while not done:
        action, _ = ppo_model.predict(obs)
        obs, reward, done, info = env.step(action)
        # print("Action taken:", action)
        rgb, depth = env.get_camera_image()
        time.sleep(0.05) 

    return action

for cls, x, y in predicted_boxes:
    print(f"Handling Class {cls} at ({x:.5f}, {y:.5f})")

    # Step 4.1: Move above box
    action_box = move_to(x, y, 0.025)  # hover height
    print("Picking up", action_box)
    env.toggle_gripper_func()
    time.sleep(0.02) 

    action_lift = move_to(x, y, 0.06)  # lift height
    print("Lifting Up", action_lift)

    action_goal = move_to(*goal_positions[cls])
    print("Goalies", action_goal)
    env.toggle_gripper_func()
    time.sleep(0.02) 
    

print("✅ All boxes sorted!")
env.close()

import gym
import pybullet as p
import pybullet_data
import numpy as np
from gym import spaces
from robot import UArm
import time
from utilities import Camera
import random

class UArmEnv(gym.Env):
    def __init__(self, render=False):
        self.render = render
        self.physics_client = p.connect(p.GUI if self.render else p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        
        self.max_steps = 500
        self.current_step = 0
        self.global_step = 0
        self.success_count = 0
        self.timeout_count = 0

        # Load the robot and plane
        self.robot = UArm()
        self.robot.load('./urdf/UArm.urdf')
        self.planeID = p.loadURDF("plane.urdf")

        # Initialize the camera
        cam_pos, cam_forward, cam_up = self.robot.get_camera_pose("camera_link")
        self.camera = Camera(cam_pos, cam_pos + cam_forward, cam_up)
        

        # Set action and observation space
   

        self.action_space = spaces.Box(low=-1, high=1, shape=(4,), dtype=np.float32)
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(9,), dtype=np.float32)


        # Add sliders for real-time control
        self.sliders = {
            'joint1': p.addUserDebugParameter("Joint 1", -np.pi/2, np.pi/2, 0),
            'joint2': p.addUserDebugParameter("Joint 2", -np.pi*0.4, np.pi, 0.8),
            'joint3': p.addUserDebugParameter("Joint 3", -np.pi/2, np.pi*0.12, -1.57),
            # 'joint4': p.addUserDebugParameter("Joint 4", -np.pi/2, np.pi/2, 0),
            # 'joint5': p.addUserDebugParameter("Joint 5", -np.pi*2, np.pi*2, 0),
        }

        self.picked_up = False
        self.attachment = None
        self.held_box = None
        self.pickup_threshold = 0.03  # 3 cm radius snap zone
        self.last_r_pressed = False
        self.r_toggled = False
        self.toggle_gripper = False  # for toggling gripper state

        self.prev_gripper_state = None
        self.holding_box = False
        self.just_dropped = False


        

    def reset(self):
        p.resetSimulation()
        
        p.setGravity(0, 0, -9.81)
        self.planeID = p.loadURDF("plane.urdf")
        self.robot.load('./urdf/UArm.urdf')
        self.robot.reset_arm()
        self.current_step = 0  

        # Spawn the goal zone:
        # The hand moves between 0.2 to 0.4 from its base, so keep it within 0.3
        box_height = 0.25

        # red_angle = -1.3
        # red_goal_pos = [(- 0.3 * np.sin(red_angle)), (- 0.3 * np.cos(red_angle)), 0.001]
        # self.goal_id_red = p.loadURDF("urdf/goal_zone_red.urdf", basePosition=red_goal_pos)
        # self.goal_pos_red = np.array([red_goal_pos[0], red_goal_pos[1], 0.03])  # for the arm to reach, added box height

        # green_angle = 0.60 #rad, 22.5 degrees, 1/4 of a quater circle
        # green_goal_pos = [(- 0.3 * np.sin(green_angle)), (- 0.3 * np.cos(green_angle)), 0.001]

        # self.goal_id_green = p.loadURDF("urdf/goal_zone_green.urdf", basePosition=green_goal_pos)
        # self.goal_pos_green = np.array([green_goal_pos[0],green_goal_pos[1], 0.03])  # for the arm to reach, added box height

        # blue_angle = 1.3
        # blue_goal_pos = [(- 0.3 * np.sin(blue_angle)), (- 0.3 * np.cos(blue_angle)), 0.001]
        # self.goal_id_blue = p.loadURDF("urdf/goal_zone_blue.urdf", basePosition=blue_goal_pos)
        # self.goal_pos_blue = np.array([blue_goal_pos[0], blue_goal_pos[1], 0.03])

        green_angle = 0.60 #rad, 22.5 degrees, 1/4 of a quater circle
        green_goal_pos = [(- 0.3 * np.sin(green_angle)), (- 0.3 * np.cos(green_angle)), 0.001]
        self.goal_id_green = p.loadURDF("urdf/goal_zone_green.urdf", basePosition=green_goal_pos)
        self.goal_pos_green = np.array([green_goal_pos[0],green_goal_pos[1], 0.03])  # for the arm to reach, added box height

        # red_angle = -1.3
        red_goal_pos = [(green_goal_pos[0]), (green_goal_pos[1]+0.07), 0.001]
        self.goal_id_red = p.loadURDF("urdf/goal_zone_red.urdf", basePosition=red_goal_pos)
        self.goal_pos_red = np.array([red_goal_pos[0], red_goal_pos[1], 0.03])  # for the arm to reach, added box height

        # blue_angle = 1.3
        blue_goal_pos = [(green_goal_pos[0]), (green_goal_pos[1]-0.07), 0.001]
        self.goal_id_blue = p.loadURDF("urdf/goal_zone_blue.urdf", basePosition=blue_goal_pos)
        self.goal_pos_blue = np.array([blue_goal_pos[0], blue_goal_pos[1], 0.03])

        # self.robot.visualize_end_effector()
        self.robot.visualize_camera_orientation("camera_link")
        self.robot.visualize_tool_axes("tool")

        self.boxes = []
        self.box_instances = []  # [(box_id, class_id)]

        box_types = [
            (0, "box_small.urdf"),
            (1, "box_medium.urdf"),
            (2, "box_large.urdf"),
        ]

        box_specs = []
        for _ in range(9):  # 3x3 grid
            box_specs.append(random.choice(box_types))

        start_x = 0.04
        start_y = -0.25
        spacing_x = 0.07
        spacing_y = 0.07

        for idx, (class_id, box_file) in enumerate(box_specs):
            row = idx // 3
            col = idx % 3
            x = start_x + col * spacing_x
            y = start_y - row * spacing_y
            pos = [x, y, 0.025]
            box_id = p.loadURDF(f"boxes/{box_file}", basePosition=pos)
            self.boxes.append(box_id)
            self.box_instances.append((box_id, class_id))

        # ran_new_or_goal = np.random.uniform(0, 1) > 0.1
        # if ran_new_or_goal:
        #     ef_rad = np.random.uniform(-1.5, 1.5)
        #     ef_dist = np.random.uniform(0.2, 0.4)  # distance from base
        #     x = -ef_dist * np.sin(ef_rad)
        #     y = -ef_dist * np.cos(ef_rad)
        #     z = np.random.uniform(0.025, 0.075)  # fixed height for the end-effector
        #     self.goal_pos = np.array([x, y, z])
        # else:
        #     # Randomly choose one of the predefined goal positions
        #     goal_choice = np.random.choice(['red', 'green', 'blue'])
        #     if goal_choice == 'red':
        #         self.goal_pos = self.goal_pos_red
        #     elif goal_choice == 'green':
        #         self.goal_pos = self.goal_pos_green
        #     else:
        #         self.goal_pos = self.goal_pos_blue
        # self.goal_pos = self.goal_pos_red

        self.spawn_positions_xy = [
            (0.040, -0.390),
            (0.040, -0.320),
            (0.040, -0.250),
            (0.110, -0.250),
            (0.110, -0.390),
            (0.110, -0.320),
            (0.180, -0.250),
            (0.180, -0.320),
            (0.180, -0.390),
        ]
        self.spawn_z_values = [0.025, 0.075]

        self.goal_positions = [
            self.goal_pos_red,
            self.goal_pos_green,
            self.goal_pos_blue
        ]

        if np.random.uniform() < 0.7:
            # 📦 Choose from spawn positions
            x, y = random.choice(self.spawn_positions_xy)
            x += np.random.uniform(-0.005, 0.005)
            y += np.random.uniform(-0.005, 0.005)
            z = random.choice(self.spawn_z_values)
            self.goal_pos = np.array([x, y, z])
        else:
            # 🎯 Choose from goal zones (z is always 0.03)
            self.goal_pos = random.choice(self.goal_positions)


        print(f"Goal position set to: {self.goal_pos}")
        

        return self._get_obs()

    def step(self, action=None):
        # Read joint sliders
        reward = 0.0
        done = False


        if action is not None:
            # PPO mode
            joint1 = np.clip(action[0], -np.pi/2, np.pi/2) 
            joint2 = np.clip(action[1], -np.pi*0.4, np.pi) 
            joint3 = np.clip(action[2], -np.pi/2, np.pi*0.12) 
            release_toggle = False
            self.r_toggled = not release_toggle if self.picked_up else release_toggle
        else:
            # Manual debug mode with sliders
            joint1 = p.readUserDebugParameter(self.sliders['joint1'])
            joint2 = p.readUserDebugParameter(self.sliders['joint2'])
            joint3 = p.readUserDebugParameter(self.sliders['joint3'])

        # Auto-compensate wrist tilt
        joint4 = -joint2 - joint3

        # # Dynamically set Joint 5 to cancel rotation
        joint5 = -joint1

        # Apply joint angles
        joint_angles = [joint1, joint2, joint3, joint4, joint5]
        self.robot.move_ee(joint_angles)

        # Get gripper position
        ee_pos = self.robot.get_joint_obs()['ee_pos']

        # If not already holding something, try to pick one up
        # Gripper closed: attempt pickup if not holding anything
        if self.r_toggled and not self.picked_up:
            for box_id in self.boxes:
                box_pos, _ = p.getBasePositionAndOrientation(box_id)
                dist = np.linalg.norm(np.array(ee_pos) - np.array(box_pos))
                if dist < self.pickup_threshold:
                    self.attachment = p.createConstraint(
                        parentBodyUniqueId=self.robot.id,
                        parentLinkIndex=self.robot.eef_id,
                        childBodyUniqueId=box_id,
                        childLinkIndex=-1,
                        jointType=p.JOINT_FIXED,
                        jointAxis=[0, 0, 0],
                        parentFramePosition=[0, 0, 0],
                        childFramePosition=[0, 0, 0.020-0.007]  # ← adjust as needed
                    )
                    self.picked_up = True
                    
                    self.held_box = box_id
                    print("📦 Box snapped to gripper!")
                    break

        # Gripper open: release if currently holding
        if not self.r_toggled and self.picked_up:
            p.removeConstraint(self.attachment)
            self.attachment = None
            self.picked_up = False
            self.held_box = None
            
            print("👐 Box released.")

        # Manual release with 'R'
        keys = p.getKeyboardEvents()
        r_pressed = ord('r') in keys and keys[ord('r')] & p.KEY_WAS_TRIGGERED

        # If the R key has just been pressed (toggle)
        if r_pressed and not self.last_r_pressed:
            self.r_toggled = not self.r_toggled  # Flip gripper state
            self.toggle_gripper = not self.toggle_gripper  # Toggle gripper state
            print(f"[R] Toggled gripper: {'Closed' if self.r_toggled else 'Open'}")

        # Save state for next frame
        self.last_r_pressed = r_pressed

        # Update camera visuals
        self.robot.update_camera_orientation("camera_link")
        self.robot.update_tool_axes("tool")


        for _ in range(10):  # Try 30–120 steps per frame
            p.stepSimulation()
            if self.render:
                time.sleep(1/240)

        # Despawn boxes that reach goal zones
        # Despawn boxes only when they reach their correct goal zone
        if self.toggle_gripper == False:  # Only check for despawn when not holding a box
            for box_id, class_id in self.box_instances[:]:
                if class_id == 0 and self._check_box_in_goal_zone(box_id, self.goal_pos_red):
                    p.removeBody(box_id)
                    self.box_instances.remove((box_id, class_id))
                    self.boxes.remove(box_id)
                    # print(f"🔴 Red box {box_id} despawned.")
                elif class_id == 1 and self._check_box_in_goal_zone(box_id, self.goal_pos_green):
                    p.removeBody(box_id)
                    self.box_instances.remove((box_id, class_id))
                    self.boxes.remove(box_id)
                    # print(f"🟢 Green box {box_id} despawned.")
                elif class_id == 2 and self._check_box_in_goal_zone(box_id, self.goal_pos_blue):
                    p.removeBody(box_id)
                    self.box_instances.remove((box_id, class_id))
                    self.boxes.remove(box_id)
                    # print(f"🔵 Blue box {box_id} despawned.")


        # Get obs and reward
        obs = self._get_obs()
        ee_pos = self.robot.get_joint_obs()['ee_pos']
        dist_to_goal = np.linalg.norm(np.array(ee_pos) - self.goal_pos)
        reward = -dist_to_goal

        # done = dist_to_goal < 0.05
        self.current_step += 1
        self.global_step += 1
        if dist_to_goal < 0.04 :
            print(f"🎉 Goal reached! EE: {np.round(ee_pos, 3)} | Goal: {np.round(self.goal_pos, 3)} | Reward: {round(reward, 4)}")
            # Reset the environment if goal is reached
            done = True
            reward += 2.0  # Bonus for reaching the goal
            self.success_count += 1

        if self.current_step >= self.max_steps:
            done = True
            # print(f"⏱️ Timeout at Step {self.current_step} | EE: {np.round(ee_pos, 3)} | Goal: {np.round(self.goal_pos, 3)}")
            self.timeout_count += 1
        if self.global_step % 20000 == 0:
            print(f"📊 Step {self.global_step} | ✅ Successes: {self.success_count} | ⏱️ Timeouts: {self.timeout_count}")

        # print("Reward:", reward)
        # current_gripper_state = action[3]
        # self.just_dropped = self.holding_box and current_gripper_state < 0
        # self.holding_box = current_gripper_state > 0

        # # ee_pos = self.get_end_effector_pos()["ee_pos"]
        # goal_pos = self.goal_pos  # Or any logic for selecting target

        # # Check if we just dropped onto correct zone
        # placed_successfully = False
        # if self.just_dropped:
        #     for box_id, class_id in self.box_instances[:]:
        #         target_zone = [self.goal_pos_red, self.goal_pos_green, self.goal_pos_blue][class_id]
        #         if self._check_box_in_goal_zone(box_id, target_zone):
        #             placed_successfully = True
        #             break

        # reward = self.compute_reward(ee_pos, goal_pos, current_gripper_state, self.holding_box, self.just_dropped, placed_successfully)



        return obs, reward, done, {}


    # def _get_obs(self):
    #     joint_obs = self.robot.get_joint_obs()
    #     positions = joint_obs['positions']
    #     velocities = joint_obs['velocities']
    #     ee_pos = joint_obs['ee_pos']
    #     return np.concatenate([positions, velocities, ee_pos])
    def _get_obs(self):
        joint_obs = self.robot.get_joint_obs()
        joint_pos = joint_obs['positions'][:3]       # only joints 1-3
        ee_pos = joint_obs['ee_pos']
        return np.concatenate([joint_pos, ee_pos, self.goal_pos])  # shape: (9,)


    def render(self, mode='human'):
        pass  # PyBullet handles rendering automatically

    def close(self):
        p.disconnect(self.physics_client)


    def get_camera_image(self):
        cam_pos, cam_forward, cam_up = self.robot.get_camera_pose("camera_link")
        if cam_pos is None:
            return None, None

        cam_target = cam_pos + cam_forward * 1.0  # 1 meter ahead
        self.camera.update_view_matrix(cam_pos, cam_target, cam_up)
        rgb, depth = self.camera.capture_image()
        return rgb, depth
    
    def get_box_positions(self):
        """
        Returns list of dicts: [{'class_id': 0, 'world_pos': [x, y, z], 'size': [sx, sy, sz]}, ...]
        """
        info = []
        for box_id, box_meta in self.spawned_boxes:  # assuming you track them
            pos, _ = p.getBasePositionAndOrientation(box_id)
            size = box_meta['size']  # or hardcoded
            info.append({'class_id': 0, 'world_pos': pos, 'size': size})
        return info

    def capture_single_image(self, save_path="test_image.png"):
        rgb, _ = self.get_camera_image()
        if rgb is not None:
            import cv2
            rgb = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            cv2.imwrite(save_path, rgb)
            print(f"✅ Saved image to {save_path}")
        else:
            print("❌ Camera image not captured.")
        rgb, _ = self.get_camera_image()

        cam_pos, cam_forward, cam_up = self.robot.get_camera_pose("camera_link")
        cam_target = cam_pos + cam_forward

        print(f"Camera Position: {cam_pos}")
        print(f"Camera Forward: {cam_forward}")
        print(f"Camera Target:  {cam_target}")



    def capture_image_and_dummy_label(self, base_dir, image_index):
        import os
        import cv2

        # --- Save image ---
        rgb, _ = self.get_camera_image()
        if rgb is None:
            print("❌ Failed to capture image.")
            return

        image_dir = os.path.join(base_dir, "images")
        label_dir = os.path.join(base_dir, "labels")
        os.makedirs(image_dir, exist_ok=True)
        os.makedirs(label_dir, exist_ok=True)

        image_path = os.path.join(image_dir, f"image_{image_index:04}.png")
        rgb_bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        cv2.imwrite(image_path, rgb_bgr)

        # --- YOLO Label Generation ---
        width, height = self.camera.width, self.camera.height

        # Define camera view bounds manually (these should match what the camera sees)
        X_MIN, X_MAX = -0.170, 0.210
        Y_MIN, Y_MAX = -0.490, -0.120

        box_width_map = {
            0: 0.035,  # small
            1: 0.04,  # medium
            2: 0.05   # large
        }

        box_length_map = {
            0: 0.04,  # small
            1: 0.04,  # medium
            2: 0.04   # large
        }

        label_path = os.path.join(label_dir, f"image_{image_index:04}.txt")
        with open(label_path, "w") as f:
            for box_id, class_id in self.box_instances:
                pos, _ = p.getBasePositionAndOrientation(box_id)
                x_world, y_world, _ = pos

                # Skip boxes outside view
                if not (X_MIN <= x_world <= X_MAX and Y_MIN <= y_world <= Y_MAX):
                    continue

                # Convert world to image-normalised coordinates
                x_center = 1.0 - (x_world - X_MIN) / (X_MAX - X_MIN)  # because +X is to the left
                y_center = (y_world - Y_MIN) / (Y_MAX - Y_MIN)  # because +Y is upward



                # # Use normalised size estimate
                # size = box_size_map[class_id]
                # box_w = size / (X_MAX - X_MIN)
                # box_h = size / (Y_MAX - Y_MIN)

                size_length = box_length_map[class_id]
                box_h = size_length / (X_MAX - X_MIN)

                size_width = box_width_map[class_id]
                box_w = size_width / (Y_MAX - Y_MIN)

                f.write(f"{class_id} {x_center:.6f} {y_center:.6f} {box_w:.6f} {box_h:.6f}\n")

        print(f"✅ Saved image + YOLO label to: {image_path}, {label_path}")

    def estimate_camera_bounds(self, grid_res=0.01):
        in_view = []

        view_matrix = np.array(self.camera.view_matrix).reshape(4, 4, order='F')
        proj_matrix = np.array(self.camera.projection_matrix).reshape(4, 4, order='F')

        # World grid (flat XY plane at z=0.025)
        X = np.arange(-0.5, 0.5, grid_res)
        Y = np.arange(-0.5, 0.5, grid_res)
        Z = 0.025

        for x in X:
            for y in Y:
                world = np.array([x, y, Z, 1.0])
                clip = proj_matrix @ (view_matrix @ world)
                if clip[3] == 0:
                    continue
                ndc = clip[:3] / clip[3]
                x_ndc, y_ndc = ndc[0], ndc[1]
                if -1 <= x_ndc <= 1 and -1 <= y_ndc <= 1:
                    in_view.append((x, y))

        if not in_view:
            print("❌ No points detected in view!")
            return

        x_vals, y_vals = zip(*in_view)
        print("✅ Camera visible bounds:")
        print(f"X_MIN = {min(x_vals):.3f}, X_MAX = {max(x_vals):.3f}")
        print(f"Y_MIN = {min(y_vals):.3f}, Y_MAX = {max(y_vals):.3f}")


    def capture_image_label_and_gt(self, base_dir, image_index):
        import os
        import cv2

        # Get image from top-down camera
        rgb, _ = self.get_camera_image()
        if rgb is None:
            print("❌ Failed to capture image.")
            return

        # Create folders
        image_dir = os.path.join(base_dir, "images")
        label_dir = os.path.join(base_dir, "labels")
        gt_dir = os.path.join(base_dir, "gt_locations")
        os.makedirs(image_dir, exist_ok=True)
        os.makedirs(label_dir, exist_ok=True)
        os.makedirs(gt_dir, exist_ok=True)

        # Save image
        image_path = os.path.join(image_dir, f"image_{image_index:04}.png")
        rgb_bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        cv2.imwrite(image_path, rgb_bgr)

        # Camera view bounds
        X_MIN, X_MAX = -0.170, 0.210
        Y_MIN, Y_MAX = -0.490, -0.120

        box_width_map = {
            0: 0.035,  # small
            1: 0.04,  # medium
            2: 0.05   # large
        }

        box_length_map = {
            0: 0.04,  # small
            1: 0.04,  # medium
            2: 0.04   # large
        }

        # Save YOLO label and GT world coordinates
        label_path = os.path.join(label_dir, f"image_{image_index:04}.txt")
        gt_path = os.path.join(gt_dir, f"image_{image_index:04}.txt")

        with open(label_path, "w") as f_label, open(gt_path, "w") as f_gt:
            for box_id, class_id in self.box_instances:
                pos, _ = p.getBasePositionAndOrientation(box_id)
                x_world, y_world, _ = pos

                # Normalise to YOLO format
                x_center = 1.0 - (x_world - X_MIN) / (X_MAX - X_MIN)  # because +X is left
                y_center = (y_world - Y_MIN) / (Y_MAX - Y_MIN)  # because -Y is up

                # box_size = box_size_map[class_id]
                # box_w = box_size / (X_MAX - X_MIN)
                # box_h = box_size / (Y_MAX - Y_MIN)

                size_length = box_length_map[class_id]
                box_h = size_length / (X_MAX - X_MIN)

                size_width = box_width_map[class_id]
                box_w = size_width / (Y_MAX - Y_MIN)

                # Write label
                f_label.write(f"{class_id} {x_center:.6f} {y_center:.6f} {box_w:.6f} {box_h:.6f}\n")

                # Write ground truth world position
                f_gt.write(f"{class_id} {x_world:.6f} {y_world:.6f}\n")

        print(f"✅ Saved image, label, and ground truth for image_{image_index:04}")


    def _check_box_in_goal_zone(self, box_id, goal_pos, xy_threshold=0.06, z_threshold=0.015):
        box_pos, _ = p.getBasePositionAndOrientation(box_id)
        xy_dist = np.linalg.norm(np.array(box_pos[:2]) - np.array(goal_pos[:2]))
        z_diff = abs(box_pos[2] - goal_pos[2])
        return (xy_dist < xy_threshold) and (z_diff < z_threshold) 
    

    def compute_reward(self, ee_pos, goal_pos, gripper_state, holding_box, just_dropped, placed_successfully):
        # Distance to goal
        dist_to_goal = np.linalg.norm(np.array(ee_pos[:2]) - np.array(goal_pos[:2]))
        reward = -dist_to_goal  # Encourage getting closer

        # Bonus for being very close
        if dist_to_goal < 0.02:
            reward += 0.5

        # Bonus for gripping a box
        if holding_box:
            reward += 0.3

        # Success: box dropped in correct goal zone
        if just_dropped and placed_successfully:
            print(f"🎉 Box placed successfully in goal zone! EE: {np.round(ee_pos, 3)} | Goal: {np.round(goal_pos, 3)}")
            reward += 2.0

        # Failure: dropped elsewhere
        elif just_dropped:
            reward -= 0.5

        return reward






    # def capture_image_and_dummy_label(self, base_dir, image_index):
    #     import os
    #     import cv2

    #     rgb, _ = self.get_camera_image()
    #     if rgb is None:
    #         print("❌ Failed to capture image.")
    #         return

    #     # Create directories
    #     image_dir = os.path.join(base_dir, "images")
    #     label_dir = os.path.join(base_dir, "labels")
    #     os.makedirs(image_dir, exist_ok=True)
    #     os.makedirs(label_dir, exist_ok=True)

    #     # Save image
    #     image_path = os.path.join(image_dir, f"image_{image_index:04}.png")
    #     rgb_bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    #     cv2.imwrite(image_path, rgb_bgr)

    #     # Save dummy YOLO label
    #     label_path = os.path.join(label_dir, f"image_{image_index:04}.txt")
    #     with open(label_path, "w") as f:
    #         for box_id, class_id in self.box_instances:
    #             pos, _ = p.getBasePositionAndOrientation(box_id)
    #             screen_pos = self.project_to_image(pos)

    #             if screen_pos:  # If inside view
    #                 cx, cy = screen_pos
    #                 f.write(f"{class_id} {cx:.6f} {cy:.6f} 0.08 0.08\n")  # assume box size ~8%


    #     print(f"✅ Saved image and dummy label to folders (index {image_index:04})")
    # def capture_image_and_dummy_label(self, base_dir, image_index):
    #     import os
    #     import cv2

    #     rgb, _ = self.get_camera_image()
    #     if rgb is None:
    #         print("❌ Failed to capture image.")
    #         return

    #     # Create directories
    #     image_dir = os.path.join(base_dir, "images")
    #     label_dir = os.path.join(base_dir, "labels")
    #     os.makedirs(image_dir, exist_ok=True)
    #     os.makedirs(label_dir, exist_ok=True)

    #     # Save image
    #     image_path = os.path.join(image_dir, f"image_{image_index:04}.png")
    #     rgb_bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    #     cv2.imwrite(image_path, rgb_bgr)

    #     # Save dummy YOLO label
    #     label_path = os.path.join(label_dir, f"image_{image_index:04}.txt")
    #     with open(label_path, "w") as f:
    #         for box_id, class_id in self.box_instances:
    #             pos, _ = p.getBasePositionAndOrientation(box_id)
    #             screen_pos = self.project_to_image(pos)

    #             if screen_pos:  # If inside view
    #                 cx, cy = screen_pos
    #                 f.write(f"{class_id} {cx:.6f} {cy:.6f} 0.08 0.08\n")  # assume box size ~8%


        # print(f"✅ Saved image and dummy label to folders (index {image_index:04})")
    # def capture_image_and_label(self, base_dir, image_index):
    #     import os, cv2

    #     rgb, _ = self.get_camera_image()
    #     if rgb is None:
    #         print("❌ Failed to capture image.")
    #         return

    #     image_dir = os.path.join(base_dir, "images")
    #     label_dir = os.path.join(base_dir, "labels")
    #     os.makedirs(image_dir, exist_ok=True)
    #     os.makedirs(label_dir, exist_ok=True)

    #     image_path = os.path.join(image_dir, f"image_{image_index:04}.png")
    #     rgb_bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    #     cv2.imwrite(image_path, rgb_bgr)

    #     box_size_map = {0: 0.04, 1: 0.06, 2: 0.08}

    #     label_path = os.path.join(label_dir, f"image_{image_index:04}.txt")
    #     with open(label_path, "w") as f:
    #         for box_id, class_id in self.box_instances:
    #             pos, _ = p.getBasePositionAndOrientation(box_id)
    #             screen_pos = self.project_to_image(pos)
    #             if screen_pos:
    #                 cx, cy = screen_pos
    #                 cx /= self.image_width
    #                 cy /= self.image_height
    #                 box_size = box_size_map.get(class_id, 0.05)
    #                 norm_size = box_size / self.image_width
    #                 f.write(f"{class_id} {cx:.6f} {cy:.6f} {norm_size:.6f} {norm_size:.6f}\n")

    #     print(f"✅ Saved image and label to folders (index {image_index:04})")




def capture_global_view():
    camera_target = [0, 0, 0]           # center of the robot
    camera_pos = [0.5, -0.5, 0.5]        # adjust for your best view
    camera_up = [0, 0, 1]

    view_matrix = p.computeViewMatrix(
        cameraEyePosition=camera_pos,
        cameraTargetPosition=camera_target,
        cameraUpVector=camera_up
    )

    projection_matrix = p.computeProjectionMatrixFOV(
        fov=60, aspect=1.0, nearVal=0.01, farVal=10.0
    )

    width, height, rgbPixels, _, _ = p.getCameraImage(
        width=320, height=320,
        viewMatrix=view_matrix,
        projectionMatrix=projection_matrix
    )

    return np.reshape(rgbPixels, (height, width, 4))[:, :, :3]


    

# Uncomment to run standalone
# if __name__ == '__main__':
#     env = UArmEnv(render=True)
#     obs = env.reset()
#     while True:
#         obs, reward, done, info = env.step()
#         time.sleep(0.01)
#     env.close()

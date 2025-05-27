import gym
import pybullet as p
import pybullet_data
import numpy as np
from gym import spaces
from robot import UArm
import time
from utilities import Camera

class UArmEnv(gym.Env):
    def __init__(self, render=False):
        self.render = render
        self.physics_client = p.connect(p.GUI if self.render else p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        
        self.max_steps = 2000
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

        red_angle = -1.3
        red_goal_pos = [(- 0.3 * np.sin(red_angle)), (- 0.3 * np.cos(red_angle)), 0.001]
        self.goal_id_red = p.loadURDF("urdf/goal_zone_red.urdf", basePosition=red_goal_pos)
        self.goal_pos_red = np.array([red_goal_pos[0], red_goal_pos[1], red_goal_pos[2] + box_height])  # for the arm to reach, added box height

        green_angle = 0.60 #rad, 22.5 degrees, 1/4 of a quater circle
        green_goal_pos = [(- 0.3 * np.sin(green_angle)), (- 0.3 * np.cos(green_angle)), 0.001]

        self.goal_id_green = p.loadURDF("urdf/goal_zone_green.urdf", basePosition=green_goal_pos)
        self.goal_pos_green = np.array([green_goal_pos[0],green_goal_pos[1], green_goal_pos[2]+box_height])  # for the arm to reach, added box height

        blue_angle = 1.3
        blue_goal_pos = [(- 0.3 * np.sin(blue_angle)), (- 0.3 * np.cos(blue_angle)), 0.001]
        self.goal_id_blue = p.loadURDF("urdf/goal_zone_blue.urdf", basePosition=blue_goal_pos)
        self.goal_pos_blue = np.array([blue_goal_pos[0], blue_goal_pos[1], blue_goal_pos[2] + box_height])


        # self.robot.visualize_end_effector()
        self.robot.visualize_camera_orientation("camera_link")
        self.robot.visualize_tool_axes("tool")

        # # Spawns boxes
        # self.boxes = []
        # # Boxes specifications
        # box_specs = [
        #     ("box_small.urdf", 4),
        #     ("box_medium.urdf", 3),
        #     ("box_large.urdf", 3)

        # ]

        # start_x = 0.05
        # start_y = -0.25     # top row
        # spacing_x = 0.04  # distance between boxes
        # spacing_y = 0.07  # distance between rows

        # for row_idx, (box_file, count) in enumerate(box_specs):
        #     for i in range(count):
        #         x = start_x + i * spacing_x
        #         y = start_y - row_idx * spacing_y
        #         pos = [x, y, 0.025]
        #         box = p.loadURDF(f"boxes/{box_file}", basePosition=pos)
        #         self.boxes.append(box)
        #     spacing_x += 0.01

        # Spawns one box for training
        green_box_pos = [0.15, -0.25, 0.025]  # start closer to robot
        box_file = "box_medium.urdf"     # name your green box file clearly
        box_id = p.loadURDF(f"./boxes/{box_file}", basePosition=green_box_pos)
        self.boxes = [box_id]
        self.target_box = box_id

        ef_rad = np.random.uniform(-1.5, 1.5)
        ef_dist = np.random.uniform(0.2, 0.4)  # distance from base
        x = -ef_dist * np.sin(ef_rad)
        y = -ef_dist * np.cos(ef_rad)
        z = 0.025  # fixed height for the end-effector
        self.goal_pos = np.array([x, y, z])

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
            release_toggle = action[3] > 0.5
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
                        childFramePosition=[0, 0, 0.020]  # ← adjust as needed
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


        # Get obs and reward
        obs = self._get_obs()
        ee_pos = self.robot.get_joint_obs()['ee_pos']
        dist_to_goal = np.linalg.norm(np.array(ee_pos) - self.goal_pos)
        reward = -dist_to_goal

        # done = dist_to_goal < 0.05
        self.current_step += 1
        self.global_step += 1
        if dist_to_goal < 0.1 :
            # print(f"🎉 Goal reached! EE: {np.round(ee_pos, 3)} | Goal: {np.round(self.goal_pos, 3)} | Reward: {round(reward, 4)}")
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

def capture_global_view():
    camera_target = [0, 0, 0]           # center of the robot
    camera_pos = [0.5, 0.5, 0.5]        # adjust for your best view
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

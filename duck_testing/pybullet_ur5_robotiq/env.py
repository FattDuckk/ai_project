import time
import math
import random

import numpy as np
import pybullet as p
import pybullet_data

from utilities import Models, Camera
from collections import namedtuple
from attrdict import AttrDict
from tqdm import tqdm


class FailToReachTargetError(RuntimeError):
    pass


class ClutteredPushGrasp:

    SIMULATION_STEP_DELAY = 1 / 240.

    def __init__(self, robot, models: Models, camera=None, vis=False) -> None:


        self.robot = robot
        self.vis = vis
        if self.vis:
            self.p_bar = tqdm(ncols=0, disable=False)
        self.camera = camera

        # define environment
        self.physicsClient = p.connect(p.GUI if self.vis else p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -10)
        self.planeID = p.loadURDF("plane.urdf")

        self.robot.load()
        self.robot.step_simulation = self.step_simulation

        # custom sliders to tune parameters (name of the parameter,range,initial value)
        self.xin = p.addUserDebugParameter("x", -0.224, 0.224, 0)
        self.yin = p.addUserDebugParameter("y", -0.224, 0.224, 0)
        self.zin = p.addUserDebugParameter("z", 0, 1., 0.5)
        self.rollId = p.addUserDebugParameter("roll", -3.14, 3.14, 0)
        self.pitchId = p.addUserDebugParameter("pitch", -3.14, 3.14, np.pi/2)
        self.yawId = p.addUserDebugParameter("yaw", -np.pi/2, np.pi/2, np.pi/2)
        self.gripper_opening_length_control = p.addUserDebugParameter("gripper_opening_length", 0, 0.085, 0.04)

        # self.boxID = p.loadURDF("./urdf/skew-box-button.urdf",
        #                         [0.0, 0.0, 0.0],
        #                         # p.getQuaternionFromEuler([0, 1.5706453, 0]),
        #                         p.getQuaternionFromEuler([0, 0, 0]),
        #                         useFixedBase=True,
        #                         flags=p.URDF_MERGE_FIXED_LINKS | p.URDF_USE_SELF_COLLISION)

        # For calculating the reward
        self.box_opened = False
        self.btn_pressed = False
        self.box_closed = False

        self.boxes = []  # for tracking box IDs

        import random
        import math

        # Box spawn settings
        box_types = [
            "./boxes/box_small.urdf",
            "./boxes/box_medium.urdf",
            "./boxes/box_large.urdf",
        ]

        num_boxes = 12
        y_range = (0.1, -0.3)
        x_range = (-0.3, 0.3)
        min_distance = 0.08  # Minimum distance between box centers (adjust if needed)

        existing_positions = []

        for _ in range(num_boxes):
            placed = False
            while not placed:
                urdf_file = random.choice(box_types)
                x = random.uniform(x_range[0], x_range[1])
                y = random.uniform(y_range[0], y_range[1])
                z = 0.05

                new_pos = (x, y)

                # Check against all existing boxes
                too_close = False
                for pos in existing_positions:
                    dist = math.sqrt((pos[0] - x)**2 + (pos[1] - y)**2)
                    if dist < min_distance:
                        too_close = True
                        break

                if not too_close:
                    box_id = p.loadURDF(urdf_file, basePosition=[x, y, z])
                    self.boxes.append(box_id)
                    existing_positions.append(new_pos)
                    placed = True

        # if self.vis:
        #     p.resetDebugVisualizerCamera(
        #         cameraDistance=1.2,      # how far back the camera is
        #         cameraYaw=90,            # rotate left/right
        #         cameraPitch=-89,         # -89 = looking almost straight down
        #         cameraTargetPosition=[0, -1.5, 0.05]   # where the camera looks (center of your boxes)
        #     )

        # if self.vis:
        #     p.resetDebugVisualizerCamera(
        #         cameraDistance=0.5,      # how far back the camera is
        #         cameraYaw=90,            # rotate left/right
        #         cameraPitch=-89,         # -89 = looking almost straight down
        #         cameraTargetPosition=[0, -1.5, 0.05]   # where the camera looks (center of your boxes)
        #     )

        # if self.vis:
        #     for d in np.linspace(100.0, 0.5, num=300):  # Start at 3.0, end at 1.2
        #         p.resetDebugVisualizerCamera(
        #             cameraDistance=d,
        #             cameraYaw=90,
        #             cameraPitch=-89,
        #             cameraTargetPosition=[0, -1.5, 0.05]
        #         )
        #         time.sleep(1/60)  # 60 FPS smooth zoom-in

    def get_observation(self):
        obs = dict()

        try:
            # Get gripper pose
            ee_pos, ee_orn = p.getLinkState(self.robot.id, self.robot.eef_link_index)[:2]
            print(f"[DEBUG] ee_pos: {ee_pos}, ee_orn: {ee_orn}")

            ee_rot_matrix = p.getMatrixFromQuaternion(ee_orn)
            ee_rot_matrix = np.array(ee_rot_matrix).reshape(3, 3)


            cam_forward = ee_rot_matrix @ np.array([1, 0, 0])
            cam_up = ee_rot_matrix @ np.array([0, 1, 0])
            cam_target = ee_pos + 0.1 * cam_forward

            # view_matrix = p.computeViewMatrix(ee_pos, cam_target, cam_up)

            cam_forward = ee_rot_matrix @ np.array([1, 0, 0])     # Forward in X direction
            cam_forward = cam_forward / np.linalg.norm(cam_forward)

            cam_up = ee_rot_matrix @ np.array([0, 1, 0])           # Y-axis up

            # Move the camera origin FORWARD
            cam_pos = ee_pos + 0.1 * cam_forward   # 🔧 Try 0.03 to move in front of gripper base
            cam_target = cam_pos + 0.2 * cam_forward

            view_matrix = p.computeViewMatrix(cam_pos, cam_target, cam_up)

            projection_matrix = p.computeProjectionMatrixFOV(
                fov=60, aspect=1.0, nearVal=0.01, farVal=2.0
            )

            width, height, rgba_img, depth_img, seg_img = p.getCameraImage(
                width=320,
                height=320,
                viewMatrix=view_matrix,
                projectionMatrix=projection_matrix,
                renderer=p.ER_BULLET_HARDWARE_OPENGL
            )

            obs["rgb"] = np.reshape(rgba_img, (height, width, 4))
            obs["depth"] = depth_img
            obs["seg"] = seg_img

        except Exception as e:
            print(f"[ERROR] Camera capture failed: {e}")

        obs.update(self.robot.get_joint_obs())
        return obs






    def step_simulation(self):
        """
        Hook p.stepSimulation()
        """
        p.stepSimulation()
        if self.vis:
            time.sleep(self.SIMULATION_STEP_DELAY)
            self.p_bar.update(1)

    def read_debug_parameter(self):
        # read the value of task parameter
        x = p.readUserDebugParameter(self.xin)
        y = p.readUserDebugParameter(self.yin)
        z = p.readUserDebugParameter(self.zin)
        roll = p.readUserDebugParameter(self.rollId)
        pitch = p.readUserDebugParameter(self.pitchId)
        yaw = p.readUserDebugParameter(self.yawId)
        gripper_opening_length = p.readUserDebugParameter(self.gripper_opening_length_control)

        return x, y, z, roll, pitch, yaw, gripper_opening_length

    def step(self, action, control_method='joint'):
        """
        action: (x, y, z, roll, pitch, yaw, gripper_opening_length) for End Effector Position Control
                (a1, a2, a3, a4, a5, a6, a7, gripper_opening_length) for Joint Position Control
        control_method:  'end' for end effector position control
                         'joint' for joint position control
        """
        assert control_method in ('joint', 'end')
        self.robot.move_ee(action[:-1], control_method)
        self.robot.move_gripper(action[-1])
        for _ in range(120):  # Wait for a few steps
            self.step_simulation()

        # reward = self.update_reward()
        reward = 0
        done = True if reward == 1 else False
        info = dict(box_opened=self.box_opened, btn_pressed=self.btn_pressed, box_closed=self.box_closed)
        return self.get_observation(), reward, done, info

    # def update_reward(self):
    #     reward = 0
    #     if not self.box_opened:
    #         if p.getJointState(self.boxID, 1)[0] > 1.9:
    #             self.box_opened = True
    #             print('Box opened!')
    #     elif not self.btn_pressed:
    #         if p.getJointState(self.boxID, 0)[0] < - 0.02:
    #             self.btn_pressed = True
    #             print('Btn pressed!')
    #     else:
    #         if p.getJointState(self.boxID, 1)[0] < 0.1:
    #             print('Box closed!')
    #             self.box_closed = True
    #             reward = 1
    #     return reward

    # def get_observation(self):
    #     obs = dict()
    #     if isinstance(self.camera, Camera):
    #         rgb, depth, seg = self.camera.shot()
    #         obs.update(dict(rgb=rgb, depth=depth, seg=seg))
    #     else:
    #         assert self.camera is None
    #     obs.update(self.robot.get_joint_obs())

    #     return obs

    # def reset_box(self):
    #     p.setJointMotorControl2(self.boxID, 0, p.POSITION_CONTROL, force=1)
    #     p.setJointMotorControl2(self.boxID, 1, p.VELOCITY_CONTROL, force=0)

    def reset(self):
        self.robot.reset()
        # self.reset_box()
        return self.get_observation()

    def close(self):
        p.disconnect(self.physicsClient)

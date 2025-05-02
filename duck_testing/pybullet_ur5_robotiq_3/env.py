
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

class DummyRobot:
    def load(self): pass
    def reset(self): pass
    def move_ee(self, pose, method): pass
    def move_gripper(self, opening_length): pass
    def get_joint_obs(self): return {}

class ClutteredPushGrasp:

    SIMULATION_STEP_DELAY = 1 / 240.

    def __init__(self, robot, models: Models, camera=None, vis=False) -> None:
        self.robot = robot or DummyRobot()
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

        self.xin = p.addUserDebugParameter("x", -0.224, 0.224, 0)
        self.yin = p.addUserDebugParameter("y", -0.224, 0.224, 0)
        self.zin = p.addUserDebugParameter("z", 0, 1., 0.5)
        self.rollId = p.addUserDebugParameter("roll", -3.14, 3.14, 0)
        self.pitchId = p.addUserDebugParameter("pitch", -3.14, 3.14, np.pi/2)
        self.yawId = p.addUserDebugParameter("yaw", -np.pi/2, np.pi/2, np.pi/2)
        self.gripper_opening_length_control = p.addUserDebugParameter("gripper_opening_length", 0, 0.085, 0.04)

        self.box_opened = False
        self.btn_pressed = False
        self.box_closed = False
        self.boxes = []

        box_types = [
            "./boxes/box_small.urdf",
            "./boxes/box_medium.urdf",
            "./boxes/box_large.urdf",
        ]

        num_boxes = 0
        y_range = (0.1, -0.3)
        x_range = (-0.3, 0.3)
        min_distance = 0.08

        existing_positions = []

        for _ in range(num_boxes):
            placed = False
            while not placed:
                urdf_file = random.choice(box_types)
                x = random.uniform(x_range[0], x_range[1])
                y = random.uniform(y_range[0], y_range[1])
                z = 0.05

                new_pos = (x, y)
                too_close = any(math.sqrt((pos[0] - x)**2 + (pos[1] - y)**2) < min_distance for pos in existing_positions)

                if not too_close:
                    box_id = p.loadURDF(urdf_file, basePosition=[x, y, z])
                    self.boxes.append(box_id)
                    existing_positions.append(new_pos)
                    placed = True

    def drop_box(self, urdf_file=None, x=0.0, y=0.0):
        if urdf_file is None:
            urdf_file = "./boxes/box_large.urdf"
        z = 0.5
        box_id = p.loadURDF(urdf_file, basePosition=[x, y, z])
        self.boxes.append(box_id)
        return box_id

    def step_simulation(self):
        p.stepSimulation()
        if self.vis:
            time.sleep(self.SIMULATION_STEP_DELAY)
            self.p_bar.update(1)

    def read_debug_parameter(self):
        return (
            p.readUserDebugParameter(self.xin),
            p.readUserDebugParameter(self.yin),
            p.readUserDebugParameter(self.zin),
            p.readUserDebugParameter(self.rollId),
            p.readUserDebugParameter(self.pitchId),
            p.readUserDebugParameter(self.yawId),
            p.readUserDebugParameter(self.gripper_opening_length_control)
        )

    def step(self, action, control_method='joint'):
        assert control_method in ('joint', 'end')
        self.robot.move_ee(action[:-1], control_method)
        self.robot.move_gripper(action[-1])
        for _ in range(120):
            self.step_simulation()

        reward = 0
        done = reward == 1
        info = dict(box_opened=self.box_opened, btn_pressed=self.btn_pressed, box_closed=self.box_closed)
        return self.get_observation(), reward, done, info

    def get_observation(self):
        obs = dict()
        if isinstance(self.camera, Camera):
            rgb, depth, seg = self.camera.shot()
            obs.update(dict(rgb=rgb, depth=depth, seg=seg))
        obs.update(self.robot.get_joint_obs())
        return obs

    def reset(self):
        self.robot.reset()
        return self.get_observation()

    def close(self):
        p.disconnect(self.physicsClient)

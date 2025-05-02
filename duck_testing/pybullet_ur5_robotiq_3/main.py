import os
import math
import random
import time

import pybullet as p
import numpy as np
from tqdm import tqdm

from env import ClutteredPushGrasp, DummyRobot  # DummyRobot is now inside env_patched.py
from robot import UR5Robotiq85
from utilities import YCBModels, Camera

def user_control_demo():
    ycb_models = YCBModels(
        os.path.join('./data/ycb', '**', 'textured-decmp.obj'),
    )
    camera = Camera((1, 1, 1),
                    (0, 0, 0),
                    (0, 0, 1),
                    0.1, 5, (320, 320), 40)
    robot = UR5Robotiq85((0, 0.5, 0), (0, 0, 0))
    env = ClutteredPushGrasp(robot, ycb_models, camera, vis=True)

    env.reset()
    while True:
        obs, reward, done, info = env.step(env.read_debug_parameter(), 'end')

def tetris_box_drop():
    env = ClutteredPushGrasp(robot=DummyRobot(), models=None, camera=None, vis=True)
    box_types = [
        "./boxes/box_small.urdf",
        "./boxes/box_medium.urdf",
        "./boxes/box_large.urdf",
    ]

    try:
        for i in range(40):
            urdf = random.choice(box_types)
            x = random.uniform(-0.1, 0.1)
            y = 0.0
            env.drop_box(urdf, x, y)
            for _ in range(240):  # 1 second of sim
                env.step_simulation()
    finally:
        env.close()

if __name__ == '__main__':
    # user_control_demo()
    tetris_box_drop()

import os

import numpy as np
import pybullet as p

from tqdm import tqdm
from env import ClutteredPushGrasp
from robot import Panda, UR5Robotiq85, UR5Robotiq140
from utilities import YCBModels, Camera
import time
import math


def user_control_demo():
    ycb_models = YCBModels(
        os.path.join('./data/ycb', '**', 'textured-decmp.obj'),
    )
    camera = Camera((1, 1, 1),
                    (0, 0, 0),
                    (0, 0, 1),
                    0.1, 5, (320, 320), 40)
    camera = None
    # robot = Panda((0, 0.5, 0), (0, 0, math.pi))
    robot = UR5Robotiq85((0, 0.5, 0), (0, 0, 0))
    env = ClutteredPushGrasp(robot, ycb_models, camera, vis=True)

    env.reset()
    # env.SIMULATION_STEP_DELAY = 0
    while True:
        obs, reward, done, info = env.step(env.read_debug_parameter(), 'end')
        # print(obs, reward, done, info)
        show_gripper_view(env)

import cv2

def show_gripper_view(env):
    obs = env.get_observation()
    rgb_img = obs["rgb"]

    # Convert from RGBA to BGR for OpenCV
    bgr_img = cv2.cvtColor(rgb_img.astype(np.uint8), cv2.COLOR_RGBA2BGR)
    cv2.imshow("Gripper Camera View", bgr_img)
    cv2.waitKey(1)  # Show for 1 ms



if __name__ == '__main__':
    user_control_demo()
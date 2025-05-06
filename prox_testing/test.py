import roboticstoolbox as rtb
import swift
import time
import numpy as np

robot = rtb.models.UArm()
print(robot)

q_start = np.zeros(robot.n)

robot.q = q_start
env = swift.Swift()
env.launch()
env.add(robot)
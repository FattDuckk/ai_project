

import numpy as np
import matplotlib.pyplot as plt
from roboticstoolbox import models
from spatialmath import SE3
from roboticstoolbox.tools.trajectory import lspb, trapezoidal
import roboticstoolbox as rtb
import swift
import time

# Load the robot model
robot = rtb.models.RoboticArm()

# Print robot info
print(robot)

# Initialize Swift


# Define initial and goal joint configurations
q_start = np.zeros(robot.n)  # [0, 0, 0, 0, 0, 0]
q_goal = [np.pi/2, -np.pi/4, np.pi/6, np.pi/12, 0]  # Adjusted to 5 elements

# Set initial configuration
robot.q = q_start
robot.plot(q_start, block=False)

# Generate a trajectory between start and goal
steps = 100  # number of points in the trajectory
traj = trapezoidal(0, 1, steps)  # Time-scaling from 0 to 1

# Solve inverse kinematics
sol = robot.ikine_LM(robot.fkine(q_goal), q0=q_start)

print(robot.fkine(sol.q))

q_traj = rtb.jtraj(q_start, sol.q, steps)
env = swift.Swift()
env.launch()
env.add(robot)


# Move along the trajectory
try:
    for q in q_traj.q:
        robot.q = q
        env.step(0.05)  # advance simulation by 50 ms
    print("Reached target pose. Waiting...")

    while True:
        time.sleep(0.05)

except KeyboardInterrupt:
    print("\nShutting down simulator...")
    env.close()

# Keep the simulator open
# # DHRobot
# link1 = RevoluteDH(a=0, alpha=-np.pi/2) 
# link2 = RevoluteDH(a=0.18, alpha=0)         
# link3 = RevoluteDH(a=0.14, alpha=np.pi/2)     
# link4 = RevoluteDH(a=0.035, alpha=np.pi/2, d=-0.06)         

# robot = DHRobot([link1, link2, link3, link4], name='4DOF_Robot')
# robot.base = SE3(0, 0, 0) 

# q = [np.pi/4, -np.pi/4, np.pi/6, np.pi/12]  
# T = robot.fkine(q)  

# robot.tool = SE3.Rz(np.pi/2) * SE3.Ry(-np.pi/2)

# robot.plot(q, block=True)

# q_start = np.array([np.pi/4, -np.pi/4, np.pi/6, np.pi/12])
# q_end   = np.array([-np.pi/4, np.pi/3, -np.pi/6, -np.pi/3])


# steps = 100
# s_obj = lspb(0, 1, steps)
# s = s_obj.s 
# q_traj = np.outer(1 - s, q_start) + np.outer(s, q_end) 

# # Get velocity and acceleration
# q_velocity = np.vstack((np.zeros((1, len(q))), np.diff(q_traj, axis=0)))
# q_acceleration = np.vstack((np.zeros((1, len(q))), np.diff(q_velocity, axis=0)))


# fig, axes = plt.subplots(3, 1, figsize=(10, 12))

# # Position
# for i in range(len(q)):
#     axes[0].plot(q_traj[:, i], label=f'Joint {i+1}')
# axes[0].set_title("Joint Position (rad)")
# axes[0].set_xlabel("Steps")
# axes[0].set_ylabel("Angle")
# axes[0].legend()
# axes[0].grid()

# # Velocity
# for i in range(len(q)):
#     axes[1].plot(q_velocity[:, i], label=f'Joint {i+1}')
# axes[1].set_title("Joint Velocity (rad/step)")
# axes[1].set_xlabel("Steps")
# axes[1].set_ylabel("Velocity")
# axes[1].legend()
# axes[1].grid()

# # Acceleration
# for i in range(len(q)):
#     axes[2].plot(q_acceleration[:, i], label=f'Joint {i+1}')
# axes[2].set_title("Joint Acceleration (rad/step²)")
# axes[2].set_xlabel("Steps")
# axes[2].set_ylabel("Acceleration")
# axes[2].legend()
# axes[2].grid()

# plt.tight_layout()
# plt.show()
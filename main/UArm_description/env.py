import gym
import pybullet as p
import pybullet_data
import numpy as np
from gym import spaces
from robot import UArm
import time

class UArmEnv(gym.Env):
    def __init__(self, render=False):
        self.render = render
        self.physics_client = p.connect(p.GUI if self.render else p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)

        # Load the robot
        self.robot = UArm()
        self.robot.load('./urdf/UArm.urdf')
        self.planeID = p.loadURDF("plane.urdf")
        


        # Set action and observation space
        self.action_space = spaces.Box(low=-np.pi, high=np.pi, shape=(self.robot.arm_num_dofs,), dtype=np.float32)
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(self.robot.arm_num_dofs * 2 + 3,), dtype=np.float32)

        # Add sliders for real-time control
        self.sliders = {
            'joint1': p.addUserDebugParameter("Joint 1", -np.pi/2, np.pi/2, 0),
            'joint2': p.addUserDebugParameter("Joint 2", -np.pi*0.4, np.pi, 0),
            'joint3': p.addUserDebugParameter("Joint 3", -np.pi/2, np.pi*0.12, 0),
            'joint4': p.addUserDebugParameter("Joint 4", -np.pi/2, np.pi/2, 0),
        }

    def reset(self):
        p.resetSimulation()
        p.setGravity(0, 0, -9.81)
        self.planeID = p.loadURDF("plane.urdf")
        self.robot.load('./urdf/UArm.urdf')
        self.robot.reset_arm()
        return self._get_obs()

    def step(self, action=None):
        # Apply joint position control from sliders
        joint_angles = [p.readUserDebugParameter(self.sliders[f'joint{i+1}']) for i in range(self.robot.arm_num_dofs)]
        self.robot.move_ee(joint_angles)

        # Step simulation
        p.stepSimulation()

        # Get observations
        obs = self._get_obs()
        reward = -np.linalg.norm(obs[-3:])  # Distance to origin as a simple reward
        done = False  # No terminal state yet
        return obs, reward, done, {}

    def _get_obs(self):
        joint_obs = self.robot.get_joint_obs()
        positions = joint_obs['positions']
        velocities = joint_obs['velocities']
        ee_pos = joint_obs['ee_pos']
        return np.concatenate([positions, velocities, ee_pos])

    def render(self, mode='human'):
        pass  # PyBullet handles rendering automatically

    def close(self):
        p.disconnect(self.physics_client)

    

# Uncomment to run standalone
# if __name__ == '__main__':
#     env = UArmEnv(render=True)
#     obs = env.reset()
#     while True:
#         obs, reward, done, info = env.step()
#         time.sleep(0.01)
#     env.close()

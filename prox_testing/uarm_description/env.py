import gym
import pybullet as p
import pybullet_data
import numpy as np
from gym import spaces
from robot import UArm  # Uses the updated UArm class from your new robot.py
import time
from utilities import Camera

def setup_additional_constraints(robot_id):
    """
    Establish extra fixed constraints to "glue" together subassemblies.
    
    For example, suppose:
      - add_4 (intended to be mounted on link_1) is at index 8,
      - add_2 (on the branch from link_3) is at index 6,
      - add_3 (from base_rot) is at index 7,
      - add_1 (mounted on link_1) is at index 5, and
      - add_5 (attached to link_2's branch) is at index 9.
      
    These indices are placeholders, so verify them in your simulation.
    """
    # Constraint 1: Fix add_4 to link_1.
    cid1 = p.createConstraint(
        parentBodyUniqueId=robot_id,
        parentLinkIndex=1,   # link_1 – verify with p.getJointInfo
        childBodyUniqueId=robot_id,
        childLinkIndex=8,    # add_4 – verify with p.getJointInfo
        jointType=p.JOINT_FIXED,
        jointAxis=[0, 0, 0],
        parentFramePosition=[0, 0, 0],  # Adjust as needed.
        childFramePosition=[0, 0, 0]    # Adjust as needed.
    )
    print("Constraint created between link_1 and add_4. ID:", cid1)

    # Constraint 2: Fix add_2 (from link_3 branch) to add_4.
    cid2 = p.createConstraint(
        parentBodyUniqueId=robot_id,
        parentLinkIndex=8,   # add_4
        childBodyUniqueId=robot_id,
        childLinkIndex=6,    # add_2 
        jointType=p.JOINT_FIXED,
        jointAxis=[0, 0, 0],
        parentFramePosition=[0, 0, 0],  # Adjust accordingly.
        childFramePosition=[0, 0, 0]    # Adjust accordingly.
    )
    print("Constraint created between add_4 and add_2. ID:", cid2)

    # Constraint 3: Fix add_3 (mounted on base_rot) to link_1.
    cid3 = p.createConstraint(
        parentBodyUniqueId=robot_id,
        parentLinkIndex=1,   # link_1
        childBodyUniqueId=robot_id,
        childLinkIndex=7,    # add_3
        jointType=p.JOINT_FIXED,
        jointAxis=[0, 0, 0],
        parentFramePosition=[0, 0, 0],  # Adjust accordingly.
        childFramePosition=[0, 0, 0]    # Adjust accordingly.
    )
    print("Constraint created between link_1 and add_3. ID:", cid3)

    # Constraint 4: Fix add_5 (from link_2 branch) to add_1.
    cid4 = p.createConstraint(
        parentBodyUniqueId=robot_id,
        parentLinkIndex=5,   # add_1 (mounted on link_1)
        childBodyUniqueId=robot_id,
        childLinkIndex=9,    # add_5
        jointType=p.JOINT_FIXED,
        jointAxis=[0, 0, 0],
        parentFramePosition=[0, 0, 0],  # Adjust accordingly.
        childFramePosition=[0, 0, 0]    # Adjust accordingly.
    )
    print("Constraint created between add_1 and add_5. ID:", cid4)


class UArmEnv(gym.Env):
    def __init__(self, render=False):
        self.render_mode = render
        # Connect to PyBullet (GUI if render=True, otherwise DIRECT)
        self.physics_client = p.connect(p.GUI if self.render_mode else p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)

        # Initialize the robot using the updated UArm class
        self.robot = UArm()
        # Use the new URDF file (firefighter.urdf)
        self.robot.load('./urdf/firefighter.urdf')
        self.planeID = p.loadURDF("plane.urdf")
        self.robot.disable_self_collision()

        

        # Add additional constraints to enforce subassembly cohesion.
        setup_additional_constraints(self.robot.id)

        # Initialize the camera from the "camera_link" of the robot.
        cam_pos, cam_forward, cam_up = self.robot.get_camera_pose("camera_link")
        self.camera = Camera(cam_pos, cam_pos + cam_forward, cam_up)

        # Dynamically create debug sliders for each controllable joint.
        self.sliders = {}
        for i in range(self.robot.arm_num_dofs):
            self.sliders[f'joint{i+1}'] = p.addUserDebugParameter(
                f"Joint {i+1}", -np.pi, np.pi, 0
            )

        # Define action and observation spaces.
        # Here, observation contains positions, velocities, and the end-effector position.
        self.action_space = spaces.Box(
            low=-np.pi,
            high=np.pi,
            shape=(self.robot.arm_num_dofs,),
            dtype=np.float32
        )
        self.observation_space = spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(self.robot.arm_num_dofs * 2 + 3,),
            dtype=np.float32
        )

    def reset(self):
        # Reset the simulation and reload the plane and robot.
        p.resetSimulation()
        p.setGravity(0, 0, -9.81)
        self.planeID = p.loadURDF("plane.urdf")
        self.robot.load('./urdf/firefighter.urdf')
        self.robot.disable_self_collision()
        self.robot.reset_arm()
        self.robot.visualize_camera_orientation("camera_link")
        # Reapply additional constraints after simulation reset.
        setup_additional_constraints(self.robot.id)
        return self._get_obs()

    def step(self, action=None):
        # Read joint angles from debug sliders.
        joint_angles = [p.readUserDebugParameter(self.sliders[f'joint{i+1}'])
                        for i in range(self.robot.arm_num_dofs)]

        # Command the robot using the new joint angles.
        self.robot.move_ee(joint_angles)
        # Update the camera orientation visualization.
        self.robot.update_camera_orientation("camera_link")
        p.stepSimulation()

        obs = self._get_obs()
        # Simple reward: negative distance from origin of the end-effector.
        reward = -np.linalg.norm(obs[-3:])
        done = False
        return obs, reward, done, {}

    def _get_obs(self):
        # Get joint positions, velocities, and end-effector position.
        joint_obs = self.robot.get_joint_obs()
        positions = joint_obs['positions']
        velocities = joint_obs['velocities']
        ee_pos = joint_obs['ee_pos']
        return np.concatenate([positions, velocities, ee_pos])

    def render(self, mode='human'):
        pass  # Rendering is handled by the PyBullet GUI

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


# Uncomment the following block for standalone testing:
# if __name__ == '__main__':
#     env = UArmEnv(render=True)
#     obs = env.reset()
#     while True:
#         obs, reward, done, info = env.step()
#         time.sleep(0.01)
#     env.close()


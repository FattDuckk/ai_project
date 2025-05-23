import pybullet as p
import numpy as np
from collections import namedtuple
import os

class UArm:
    def __init__(self, base_pos=(0, 0, 0), base_ori=(0, 0, 0)):
        self.base_pos = base_pos
        self.base_ori = p.getQuaternionFromEuler(base_ori)
        self.id = None
        self.joint_info = None
        self.eef_id = None
        # We'll determine the number of controllable joints after loading the URDF.
        self.arm_num_dofs = 0  
        self.arm_rest_poses = []  # To be initialized after loading joints.
        self.gripper_range = [0, np.deg2rad(180)]  # Assuming 0 to 180 degrees for gripper

    def get_tool_link_index(self, tool_name="tool_link"):
        """
        Look for the link with the specified name.
        For the new robot, the end-effector (tool) is named "tool_link" (default).
        """
        if self.id is None:
            print("[ERROR] Robot not loaded. Cannot find tool link.")
            return None

        for i in range(p.getNumJoints(self.id)):
            info = p.getJointInfo(self.id, i)
            link_name = info[12].decode("utf-8")
            if link_name == tool_name:
                return i

        print(f"[ERROR] Tool link '{tool_name}' not found. Defaulting to last link.")
        return p.getNumJoints(self.id) - 1

    def load(self, urdf_path='./urdf/firefighter.urdf', use_fixed_base=True):
        """
        Load the new robot URDF. After loading, parse joint info,
        determine the number of controllable joints, and reset them.
        """
        abs_path = os.path.abspath(urdf_path)
        if not os.path.isfile(abs_path):
            print(f"[ERROR] URDF file not found: {abs_path}")
            return

        print(f"[INFO] Trying to load URDF from: {abs_path}")
        self.id = p.loadURDF(abs_path, self.base_pos, self.base_ori, useFixedBase=use_fixed_base)
        if self.id is None:
            print(f"[ERROR] Failed to load URDF: {abs_path}")
            return

        # Set the end-effector (tool link) using the default name "tool_link"
        self.eef_id = self.get_tool_link_index("tool_link")
        print(f"[INFO] End-effector set to link index {self.eef_id}")

        # Parse joint information and update controllable joints.
        self._parse_joint_info()
        self.arm_num_dofs = len(self.controllable_joints)
        # Set default (rest) poses (here all zeros, so the URDF's <origin> defines the starting pose).
        self.arm_rest_poses = [0.0] * self.arm_num_dofs

        self.reset_arm()

    def _parse_joint_info(self):
        """
        Parse all joint information from PyBullet and build a list of controllable joints.
        """
        num_joints = p.getNumJoints(self.id)
        JointInfo = namedtuple('jointInfo', ['id', 'name', 'type', 'lowerLimit', 'upperLimit', 'maxForce', 'maxVelocity', 'controllable'])
        self.joint_info = []
        self.controllable_joints = []

        for i in range(num_joints):
            info = p.getJointInfo(self.id, i)
            jointID = info[0]
            jointName = info[1].decode("utf-8")
            jointType = info[2]
            lowerLimit = info[8]
            upperLimit = info[9]
            maxForce = info[10]
            maxVelocity = info[11]
            controllable = (jointType != p.JOINT_FIXED)

            if controllable:
                self.controllable_joints.append(jointID)
                # Zero out any residual velocities/forces for a clean start.
                p.setJointMotorControl2(self.id, jointID, p.VELOCITY_CONTROL, targetVelocity=0, force=0)

            self.joint_info.append(JointInfo(jointID, jointName, jointType, lowerLimit, upperLimit, maxForce, maxVelocity, controllable))
        # Uncomment to view the number of detected controllable joints:
        # print(f"[INFO] Detected {len(self.controllable_joints)} controllable joints.")

    def reset_arm(self):
        """
        Reset all controllable joints to their default (rest) positions.
        Using the URDF defaults (i.e. 0 for each joint) so that
        the <origin> tags determine the starting configuration.
        """
        if self.id is None:
            print("[ERROR] Robot not loaded. Cannot reset arm.")
            return

        for rest_pose, joint_id in zip(self.arm_rest_poses, self.controllable_joints):
            p.resetJointState(self.id, joint_id, rest_pose)
        # print("[INFO] Robot arm reset to default position.")

    def move_ee(self, joint_angles):
        """
        Command the robot joints using POSITION_CONTROL.
        The number of joint_angles provided must equal the number of controllable joints.
        """
        if self.id is None:
            print("[ERROR] Robot not loaded. Cannot move end-effector.")
            return

        if len(joint_angles) != self.arm_num_dofs:
            print(f"[ERROR] Expected {self.arm_num_dofs} joint angles but got {len(joint_angles)}")
            return

        for i, joint_id in enumerate(self.controllable_joints):
            target_angle = joint_angles[i]
            p.setJointMotorControl2(self.id, joint_id, p.POSITION_CONTROL, targetPosition=target_angle)

    def get_joint_obs(self):
        """
        Return a dictionary containing current joint positions, velocities, and the end-effector position.
        """
        if self.id is None:
            print("[ERROR] Robot not loaded. Cannot get joint observations.")
            return dict(positions=[], velocities=[], ee_pos=[0, 0, 0])

        positions = []
        velocities = []
        for joint_id in self.controllable_joints:
            pos, vel, _, _ = p.getJointState(self.id, joint_id)
            positions.append(pos)
            velocities.append(vel)

        ee_pos = p.getLinkState(self.id, self.eef_id)[0]
        return dict(positions=positions, velocities=velocities, ee_pos=ee_pos)

    def open_gripper(self):
        self.move_gripper(self.gripper_range[1])

    def close_gripper(self):
        self.move_gripper(self.gripper_range[0])

    def move_gripper(self, angle):
        """
        Command the gripper joint using POSITION_CONTROL.
        """
        if self.id is None:
            print("[ERROR] Robot not loaded. Cannot move gripper.")
            return
        angle = np.clip(angle, *self.gripper_range)
        p.setJointMotorControl2(self.id, self.eef_id, p.POSITION_CONTROL, targetPosition=angle)

    def reset(self):
        self.reset_arm()

    def visualize_end_effector(self):
        """
        Visualize the current end-effector position by placing a small red sphere marker.
        """
        if self.id is None:
            print("[ERROR] Robot not loaded. Cannot visualize end-effector.")
            return

        ee_pos = p.getLinkState(self.id, self.eef_id)[0]
        self.ee_marker = p.createVisualShape(shapeType=p.GEOM_SPHERE, radius=0.01, rgbaColor=[1, 0, 0, 1])
        self.ee_marker_id = p.createMultiBody(baseVisualShapeIndex=self.ee_marker, basePosition=ee_pos)
        # print(f"[INFO] End-effector marker created at {ee_pos}")

    def update_end_effector_marker(self):
        """
        Update the position of the end-effector marker.
        """
        if self.id is None or not hasattr(self, 'ee_marker_id'):
            return

        ee_pos = p.getLinkState(self.id, self.eef_id)[0]
        p.resetBasePositionAndOrientation(self.ee_marker_id, ee_pos, [0, 0, 0, 1])

    def get_camera_pose(self, camera_link="camera_link"):
        """
        Get the position and orientation of the specified camera link.
        """
        if self.id is None:
            print("[ERROR] Robot not loaded. Cannot get camera pose.")
            return (None, None, None)

        cam_index = self.get_tool_link_index(camera_link)
        cam_state = p.getLinkState(self.id, cam_index, computeForwardKinematics=True)
        cam_pos = np.array(cam_state[0])
        cam_orn = np.array(cam_state[1])
        
        # Convert quaternion to a rotation matrix.
        rot_matrix = np.array(p.getMatrixFromQuaternion(cam_orn)).reshape(3, 3)
        cam_forward = rot_matrix @ np.array([1, 0, 0])
        cam_up = rot_matrix @ np.array([0, 0, 1])
        return cam_pos, cam_forward, cam_up

    def visualize_camera_orientation(self, camera_link="camera_link", length=0.05):
        """
        Draw debug lines for the camera's forward, up, and side vectors.
        """
        if self.id is None:
            return

        cam_pos, cam_forward, cam_up = self.get_camera_pose(camera_link)

        # Draw the forward (red) vector.
        forward_end = cam_pos + length * cam_forward
        self.cam_forward_id = p.addUserDebugLine(cam_pos, forward_end, [1, 0, 0], lineWidth=2)
        # Draw the up (green) vector.
        up_end = cam_pos + length * cam_up
        self.cam_up_id = p.addUserDebugLine(cam_pos, up_end, [0, 1, 0], lineWidth=2)
        # Draw the side (blue) vector.
        right = np.cross(cam_up, cam_forward)
        right_end = cam_pos + length * right
        self.cam_right_id = p.addUserDebugLine(cam_pos, right_end, [0, 0, 1], lineWidth=2)
        # print(f"[INFO] Camera orientation indicator added at {cam_pos}")

    def update_camera_orientation(self, camera_link="camera_link", length=0.05):
        """
        Update the camera debug lines based on the current camera pose.
        """
        if self.id is None:
            print("[ERROR] Robot not loaded. Cannot update camera orientation.")
            return
        
        cam_pos, cam_forward, cam_up = self.get_camera_pose(camera_link)
        forward_end = cam_pos + length * cam_forward
        p.addUserDebugLine(cam_pos, forward_end, [1, 0, 0], lineWidth=2, replaceItemUniqueId=self.cam_forward_id)
        up_end = cam_pos + length * cam_up
        p.addUserDebugLine(cam_pos, up_end, [0, 1, 0], lineWidth=2, replaceItemUniqueId=self.cam_up_id)
        right = np.cross(cam_up, cam_forward)
        right_end = cam_pos + length * right
        p.addUserDebugLine(cam_pos, right_end, [0, 0, 1], lineWidth=2, replaceItemUniqueId=self.cam_right_id)

    # --- New Debugging Function ---
    def disable_self_collision(self):
        """
        Disable self-collision for all links of the robot for debugging purposes.
        This sets both the collision filter group and mask to 0 for each link.
        """
        # Disable collision for the base (body -1 in PyBullet)
        p.setCollisionFilterGroupMask(self.id, -1, 0, 0)
        num_joints = p.getNumJoints(self.id)
        for i in range(num_joints):
            p.setCollisionFilterGroupMask(self.id, i, 0, 0)
        print("[DEBUG] Self-collision disabled for all links.")

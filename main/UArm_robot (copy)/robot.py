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
        self.arm_num_dofs = 4  # 4 DOF for UArm
        self.arm_rest_poses = [0, 0, 0, 0]  # Default rest position
        self.gripper_range = [0, np.deg2rad(180)]  # Assuming 0 to 180 degrees for gripper


    def get_tool_link_index(self, tool_name="tool"):
        if self.id is None:
            print("[ERROR] Robot not loaded. Cannot find tool link.")
            return None
        
        for i in range(p.getNumJoints(self.id)):
            joint_info = p.getJointInfo(self.id, i)
            link_name = joint_info[12].decode("utf-8")
            if link_name == tool_name:
                print(f"[INFO] Found tool link '{tool_name}' at index {i}")
                return i
        
        print(f"[ERROR] Tool link '{tool_name}' not found. Defaulting to last link.")
        return p.getNumJoints(self.id) - 1

    def load(self, urdf_path='./urdf/UArm.urdf', use_fixed_base=True):
        abs_path = os.path.abspath(urdf_path)
        if not os.path.isfile(abs_path):
            print(f"[ERROR] URDF file not found: {abs_path}")
            return

        print(f"[INFO] Trying to load URDF from: {abs_path}")
        self.id = p.loadURDF(abs_path, self.base_pos, self.base_ori, useFixedBase=use_fixed_base)
        if self.id is None:
            print(f"[ERROR] Failed to load URDF: {abs_path}")
            return

        # Automatically detect the end-effector
        # self.eef_id = p.getNumJoints(self.id) - 1
        self.eef_id = self.get_tool_link_index("tool")


        print(f"[INFO] End-effector set to link index {self.eef_id}")
        self._parse_joint_info()
        self.reset_arm()


    def _parse_joint_info(self):
        num_joints = p.getNumJoints(self.id)
        joint_info = namedtuple('jointInfo', ['id', 'name', 'type', 'lowerLimit', 'upperLimit', 'maxForce', 'maxVelocity', 'controllable'])
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
                # Explicitly zero out velocity and force
                p.setJointMotorControl2(self.id, jointID, p.VELOCITY_CONTROL, targetVelocity=0, force=0)
            self.joint_info.append(joint_info(jointID, jointName, jointType, lowerLimit, upperLimit, maxForce, maxVelocity, controllable))

        print(f"[INFO] Detected {len(self.controllable_joints)} controllable joints.")


    def reset_arm(self):
        if self.id is None:
            print("[ERROR] Robot not loaded. Cannot reset arm.")
            return

        for rest_pose, joint_id in zip(self.arm_rest_poses, self.controllable_joints):
            p.resetJointState(self.id, joint_id, rest_pose)
        print("[INFO] uArm reset to default position.")

    def move_ee(self, joint_angles):
        
        if self.id is None:
            print("[ERROR] Robot not loaded. Cannot move end-effector.")
            return
        assert len(joint_angles) == self.arm_num_dofs
        for i, joint_id in enumerate(self.controllable_joints):
            target_angle = joint_angles[i]
            p.setJointMotorControl2(self.id, joint_id, p.POSITION_CONTROL, targetPosition=target_angle)

    def get_joint_obs(self):
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
        if self.id is None:
            print("[ERROR] Robot not loaded. Cannot move gripper.")
            return
        angle = np.clip(angle, *self.gripper_range)
        p.setJointMotorControl2(self.id, self.eef_id, p.POSITION_CONTROL, targetPosition=angle)

    def reset(self):
        self.reset_arm()

    def visualize_end_effector(self):
        if self.id is None:
            print("[ERROR] Robot not loaded. Cannot visualize end-effector.")
            return

        # Get the current end-effector position
        ee_pos = p.getLinkState(self.id, self.eef_id)[0]

        # Draw a small sphere to mark the end-effector position
        self.ee_marker = p.createVisualShape(
            shapeType=p.GEOM_SPHERE,
            radius=0.01,  # Small sphere
            rgbaColor=[1, 0, 0, 1]  # Red color
        )

        self.ee_marker_id = p.createMultiBody(
            baseVisualShapeIndex=self.ee_marker,
            basePosition=ee_pos
        )

        print(f"[INFO] End-effector marker created at {ee_pos}")

    def update_end_effector_marker(self):
        if self.id is None or self.ee_marker_id is None:
            return

        # Update the marker to the current end-effector position
        ee_pos = p.getLinkState(self.id, self.eef_id)[0]
        p.resetBasePositionAndOrientation(self.ee_marker_id, ee_pos, [0, 0, 0, 1])



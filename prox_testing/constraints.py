import time
import numpy as np
import pybullet as p
import pybullet_data
from env import UArmEnv

def setup_additional_constraints(env):
    """
    Establish extra fixed constraints to "glue" together subassemblies that
    in your URDF are attached via branches that might separate.

    In this example (placeholder indices):
      - We enforce that add_4 remains fixed to link_1 (its intended parent),
        even though its URDF joint connects it from link_2.
      - We fix add_2 to add_4 so that the extra part on link_3's branch is held in place.
      - We fix add_3 (which is connected to base_rot) to link_1 so that the overall
        assembly remains coherent.
      - We fix add_5 to add_1, ensuring that the mismatch between link_2 and link_1 is resolved.
    
    You must adjust the indices and frame offsets according to your actual robot.
    """
    # Example indices (replace with your actual ones):
    #   base_rot: index 0
    #   link_1:   index 1
    #   link_2:   index 2
    #   link_3:   index 3
    #   link_end: index 4
    #   add_1:    index 5
    #   add_2:    index 6
    #   add_3:    index 7
    #   add_4:    index 8
    #   add_5:    index 9

    # Constraint 1: Fix add_4 to link_1.
    # (If add_4 is meant to be mounted to link_1, then regardless of its URDF joint,
    #  we maintain its relative position.)
    constraint_id1 = p.createConstraint(
        parentBodyUniqueId=env.robot_id,
        parentLinkIndex=1,   # link_1
        childBodyUniqueId=env.robot_id,
        childLinkIndex=8,    # add_4
        jointType=p.JOINT_FIXED,
        jointAxis=[0, 0, 0],
        # These offsets (parentFramePosition and childFramePosition) are crucial!
        # They should be the transforms (as defined in your URDF) from link_1 to add_4.
        parentFramePosition=[0.0, 0.0, 0.0],   # Replace with your values.
        childFramePosition=[0.0, 0.0, 0.0]     # Replace with your values.
    )

    # Constraint 2: Fix add_2 to add_4.
    # This ensures that the extra component from link_3's branch (add_2) stays attached to the add_4 subassembly.
    constraint_id2 = p.createConstraint(
        env.robot_id, 8,   # add_4 as reference
        env.robot_id, 6,   # add_2
        p.JOINT_FIXED,
        jointAxis=[0, 0, 0],
        parentFramePosition=[0.0, 0.0, 0.0],  # Adjust these offsets.
        childFramePosition=[0.0, 0.0, 0.0]
    )

    # Constraint 3: Fix add_3 (mounted on base_rot) to link_1.
    # This ensures that even though add_3’s URDF joint originates from base_rot,
    # it will follow link_1’s transformation as part of the subassembly.
    constraint_id3 = p.createConstraint(
        env.robot_id, 1,  # link_1 as the reference
        env.robot_id, 7,  # add_3
        p.JOINT_FIXED,
        jointAxis=[0, 0, 0],
        parentFramePosition=[0.0, 0.0, 0.0],  # Replace with correct offsets.
        childFramePosition=[0.0, 0.0, 0.0]
    )

    # Constraint 4: Fix add_5 to add_1.
    # This binds the extra component add_5 (which is attached to link_2 in the URDF)
    # to add_1 (mounted on link_1), ensuring both move together.
    constraint_id4 = p.createConstraint(
        env.robot_id, 5,  # add_1 (from link_1)
        env.robot_id, 9,  # add_5
        p.JOINT_FIXED,
        jointAxis=[0, 0, 0],
        parentFramePosition=[0.0, 0.0, 0.0],  # Set appropriate offset.
        childFramePosition=[0.0, 0.0, 0.0]
    )

    print("Additional subassembly constraints established:")
    print("Constraint 1 (link_1 <-> add_4):", constraint_id1)
    print("Constraint 2 (add_4 <-> add_2):", constraint_id2)
    print("Constraint 3 (link_1 <-> add_3):", constraint_id3)
    print("Constraint 4 (add_1 <-> add_5):", constraint_id4)

def sandbox_mode(env):
    """
    Run the simulation while enforcing additional fixed constraints to keep
    the subassemblies together.
    """
    step_count = 0
    print_interval = 10

    # Set up additional constraints for subassembly issues:
    setup_additional_constraints(env)

    while True:
        # Step the simulation
        obs, reward, done, info = env.step()

        if step_count % print_interval == 0:
            joint_positions = obs[:env.robot.arm_num_dofs]
            ee_position = obs[-3:]
            print(f"Joint Positions: {np.round(joint_positions, 3)} | End-Effector Position: {np.round(ee_position, 3)}")
        step_count += 1

        # Optionally, capture and process camera images
        rgb, depth = env.get_camera_image()
        time.sleep(0.01)

if __name__ == '__main__':
    # Connect to PyBullet and set up data paths if needed.
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    env = UArmEnv(render=True)
    obs = env.reset()
    print(f"Initial observation: {obs}")

    sandbox_mode(env)


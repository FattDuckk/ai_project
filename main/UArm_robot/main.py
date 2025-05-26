import time
import numpy as np
from env import UArmEnv
import pybullet as p

def sandbox_mode():
    """
    Run the simulation in sandbox mode, allowing for real-time control of the robot.
    """
    step_count = 0
    print_interval = 10
    
    while True:
        # Step the simulation without external actions
        obs, reward, done, info = env.step()
        if step_count % print_interval == 0:
                # Print joint positions and end-effector position
            joint_positions = obs[:env.robot.arm_num_dofs]
            ee_position = obs[-3:]
            print(f"Joint Positions: {np.round(joint_positions, 4)} | End-Effector Position: {np.round(ee_position, 4)}")
        step_count += 1

        # Capture and display the camera image
        rgb, depth = env.get_camera_image()
        if rgb is not None:
            # print("Camera Image Captured")
            pass

        # time.sleep(0.01)        
        
        # time.sleep(0.01)  # Slow down for readability


if __name__ == '__main__':
    # Initialize the environment
    env = UArmEnv(render=True)
    obs = env.reset()
    print(f"Initial observation: {obs}")
    

    # Sandbox mode with sliders
    sandbox_mode()



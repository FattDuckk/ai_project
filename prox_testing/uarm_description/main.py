import time
import numpy as np
from env import UArmEnv
import pybullet as p

def sandbox_mode(env):
    """
    Run the simulation in sandbox mode with the updated robot.
    The environment now dynamically creates a slider for every controllable joint.
    """
    step_count = 0
    print_interval = 10
    
    while True:
        # Step the simulation using environment.step(), which reads slider values
        obs, reward, done, info = env.step()
        
        # Print joint positions and end-effector position periodically.
        if step_count % print_interval == 0:
            joint_positions = obs[:env.robot.arm_num_dofs]
            ee_position = obs[-3:]
            print(f"Joint Positions: {np.round(joint_positions, 3)} | End-Effector Position: {np.round(ee_position, 3)}")
        
        step_count += 1

        # Optionally retrieve and process camera images.
        rgb, depth = env.get_camera_image()
        if rgb is not None:
            # For now, we just pass.
            pass

        time.sleep(0.01)       

if __name__ == '__main__':
    # Initialize the updated environment.
    env = UArmEnv(render=True)
    obs = env.reset()
    print(f"Initial observation: {obs}")
    
    # Run sandbox mode to allow real-time joint control via sliders.
    sandbox_mode(env)


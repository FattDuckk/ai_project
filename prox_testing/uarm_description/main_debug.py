import time
import numpy as np
import pybullet as p
import pybullet_data
from env import UArmEnv

# Initialize the environment (using your existing UArmEnv from env.py)
env = UArmEnv(render=True)
obs = env.reset()

# Disable self-collision for debugging.
env.robot.disable_self_collision()  
print("Self-collision disabled for debugging.")

# --- Begin Debug Setup for Constraint Offsets ---
child_x_slider  = p.addUserDebugParameter("Child Pivot X", -0.1, 0.1, 0)
child_y_slider  = p.addUserDebugParameter("Child Pivot Y", -0.1, 0.1, 0)
child_z_slider  = p.addUserDebugParameter("Child Pivot Z", -0.1, 0.1, 0)
child_orient_slider = p.addUserDebugParameter("Child Yaw (deg)", -180, 180, 0)
child_roll = 0
child_pitch = 0
constraint_id = 1  # Replace this with the correct constraint ID from your setup_additional_constraints() calls.
print("Debug mode: Using constraint_id:", constraint_id)
# --- End Debug Setup ---

while True:
    # Read slider values.
    c_x = p.readUserDebugParameter(child_x_slider)
    c_y = p.readUserDebugParameter(child_y_slider)
    c_z = p.readUserDebugParameter(child_z_slider)
    
    yaw_deg = p.readUserDebugParameter(child_orient_slider)
    yaw = np.deg2rad(yaw_deg)
    child_orient = p.getQuaternionFromEuler([child_roll, child_pitch, yaw])
    
    p.changeConstraint(
        userConstraintUniqueId=constraint_id,
        jointChildPivot=[c_x, c_y, c_z],
        jointChildFrameOrientation=child_orient,
        maxForce=500
    )
    
    p.stepSimulation()
    time.sleep(0.01)


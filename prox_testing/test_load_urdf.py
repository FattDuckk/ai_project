import pybullet as p
import pybullet_data
import time
import os

# --- Configuration ---
# Construct the path to the URDF file relative to this script
script_dir = os.path.dirname(os.path.abspath(__file__))
# Assumes 'uarm_description' is in the same directory as the script
urdf_relative_path = os.path.join("uarm_description", "urdf", "urarm.urdf")
urdf_file_path = os.path.join(script_dir, urdf_relative_path)

print(f"Attempting to load URDF file: {urdf_file_path}")

# Check if the URDF file actually exists
if not os.path.exists(urdf_file_path):
    print(f"ERROR: URDF file not found at the specified path!")
    print("Please ensure the file exists and the path is correct.")
    exit()

# --- PyBullet Setup ---
# Start PyBullet in GUI mode
physicsClient = p.connect(p.GUI)

# Add PyBullet's standard data path for things like the ground plane
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Set gravity (optional, but good practice)
p.setGravity(0, 0, -9.81)

# Load a ground plane
planeId = p.loadURDF("plane.urdf")

# --- Load the Robot ---
# Define the starting position and orientation for the robot's base
start_pos = [0, 0, 0.01] # Slightly above the ground
start_orientation = p.getQuaternionFromEuler([0, 0, 0]) # No initial rotation

print("Loading URDF...")
try:
    # Load the URDF file
    # useFixedBase=1 makes the robot's base static (won't fall over)
    robot_id = p.loadURDF(urdf_file_path,
                          start_pos,
                          start_orientation,
                          useFixedBase=1)

    print(f"SUCCESS: URDF loaded successfully!")
    print(f"Robot ID: {robot_id}")
    print("Check the PyBullet window to see the robot.")

except Exception as e:
    # Catch potential errors during loading (e.g., file not found, mesh issues)
    print("----------------------------------------")
    print(f"ERROR: Failed to load URDF!")
    print(f"PyBullet Error: {e}")
    print("----------------------------------------")
    print("Troubleshooting tips:")
    print(f"1. Verify the URDF path is correct: {urdf_file_path}")
    print("2. Check the URDF for XML errors.")
    print("3. Ensure all mesh (.stl, .dae) files referenced in the URDF exist in the correct") # Added .dae
    print("   'uarm_description/meshes/' directory relative to the URDF file.")
    print("4. PyBullet's DAE support might be limited; if errors persist, consider converting DAE to STL/OBJ.") # Added DAE warning
    print("5. Check the PyBullet console output for more detailed error messages.")
    p.disconnect()
    exit() # Exit if loading failed

# --- Keep the Simulation Running for Visual Inspection ---
print("\nSimulation running. Press Ctrl+C in the terminal (or close the window) to exit.")

try:
    # Keep the script alive so you can see the window.
    # We don't need p.stepSimulation() if we're just looking at the static load.
    while True:
        time.sleep(0.1) # Prevent high CPU usage
except KeyboardInterrupt:
    print("\nExiting...")

# --- Cleanup ---
p.disconnect()
print("Simulation disconnected.")
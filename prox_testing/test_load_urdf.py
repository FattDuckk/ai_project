import pybullet as p
import pybullet_data
import time
import os

# --- Configuration ---
# Get the directory where the script is located
script_dir = os.path.dirname(os.path.abspath(__file__))

# Define the known root directory of your project/package
# This is the directory that contains the 'uarm_description' folder
project_root_dir = "/home/matt/ai_project/prox_testing" # <--- Set this to your actual project root

# Define the relative path to the URDF file from the project root
urdf_relative_path = os.path.join("uarm_description", "urdf", "firefighter.urdf")
urdf_file_path = os.path.join(project_root_dir, urdf_relative_path)

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

# --- IMPORTANT FIX FOR PACKAGE:// PATHS ---
# Add the directory that serves as the root for your 'package://' paths.
# Your URDF uses 'package://uarm_description/...'.
# PyBullet needs to know where the 'uarm_description' directory is located.
# Based on your file structure, the 'uarm_description' directory is inside
# your project root. Therefore, the search path should be the project root directory.
package_search_path = project_root_dir # <--- Set search path to the directory *containing* uarm_description
p.setAdditionalSearchPath(package_search_path)
print(f"Added package search path: {package_search_path}")

p.setAdditionalSearchPath(pybullet_data.getDataPath())  # Set gravity (optional, but good practice) 
p.setGravity(0, 0, -9.81)  # Load a ground plane 
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
    print(f"2. Verify the package search path is correct: {package_search_path}")
    print("3. Check the URDF for XML errors.")
    print("4. Ensure all mesh (.stl) files referenced in the URDF exist within the")
    print("   correct subdirectories relative to the package search path.")
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

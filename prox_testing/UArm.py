#!/usr/bin/env python

import numpy as np
# Make sure roboticstoolbox is installed: pip install roboticstoolbox-python
try:
    from roboticstoolbox.robot import Robot
except ImportError:
    print(
        "\nError: roboticstoolbox-python is not installed."
        "Please install it using: pip install roboticstoolbox-python\n"
        )
    exit()

from math import pi
import os # Import os for path handling robustness (optional but good)


class UArm(Robot):
    """
    Class that imports a UArm URDF model using the corrected path.

    ``UArm()`` is a class which imports a UArm robot definition
    from a URDF file. The model describes its kinematic and graphical
    characteristics.

    .. runblock:: pycon

        >>> import roboticstoolbox as rtb
        >>> from UArm import UArm # Assuming you save this script as UArm.py
        >>> robot = UArm()
        >>> print(robot)

    Defined joint configurations are:

    - qz, zero joint angle configuration.
    - up, an attempt at a robot pointing upwards configuration
      (may need adjustment based on the specific URDF kinematics).

    .. codeauthor:: Tassos Natsakis (modified)
    """

    def __init__(self):
	
        # --- Define the path to the URDF file ---
        # Use the lowercase package name 'uarm_description'
        # Use the filename of the URDF that loads correctly (e.g., UArm_fixed.urdf)
        # ADJUST FILENAME if your working URDF has a different name
        urdf_filename = "uarm_description/urdf/UArm_fixed.urdf"

        # --- Optional: Make path more robust ---
        # This finds the script's directory and joins the relative path
        # Useful if you run the script from different locations,
        # but requires the uarm_description folder to be relative to the script
        # script_dir = os.path.dirname(os.path.abspath(__file__))
        # urdf_load_path = os.path.join(script_dir, urdf_filename)
        # --- Or just use the relative path directly if running from parent dir ---
        urdf_load_path = urdf_filename
        # ---

        print(f"Attempting to load URDF: {urdf_load_path}")
	
        try:
            # Use the corrected path to load the URDF
            links, name, urdf_string, urdf_filepath = self.URDF_read(
                urdf_load_path
            )
        except FileNotFoundError:
             print(f"\nError: Cannot find the URDF file at '{urdf_load_path}'.")
             print("Please ensure:")
             print("  1. You are running this script from the 'ai_project/prox_testing/' directory.")
             print("  2. The directory 'uarm_description/urdf/' exists relative to the script.")
             print(f"  3. The URDF file '{os.path.basename(urdf_load_path)}' exists inside that directory.")
             exit()
        except Exception as e:
            print(f"\nAn unexpected error occurred during URDF loading: {e}")
            print("Check the URDF file for syntax errors or issues.")
            exit()


        super().__init__(
            links,
            name=name,
            # name="UArm", # Optionally override the name from the URDF
            urdf_string=urdf_string,
            urdf_filepath=urdf_filepath,
        )

        self.manufacturer = "Unknown/Custom" # Set appropriately

        # Define joint configurations based on the loaded URDF
        # The number of joints should match the number of non-fixed joints
        num_joints = len(self.links) - self.nlinks # Crude way, better to count non-fixed

        # Check if the loaded robot has the expected number of joints (e.g., 4 for UArm_fixed.urdf)
        if num_joints == 4:
            # zero angles configuration
            self.addconfiguration("qz", np.zeros(4)) # Use zeros(num_joints)

            # Reference pose attempt: robot pointing upwards
            # NOTE: This configuration [0, 0, pi/2, 0] corresponds to the joint axes
            # in UArm_fixed.urdf. joint2 (shoulder) = 0, joint3 (elbow/roll?) = pi/2.
            # This might not look physically "up" depending on joint3's X-axis rotation.
            # A more typical 'up' might involve non-zero shoulder/elbow pitch.
            self.addconfiguration("up", np.array([0.0000, 0.0000, pi/2, 0.0000]))
        else:
            print(f"Warning: Loaded URDF has {num_joints} non-fixed joints. Default configurations might be incorrect.")
            self.addconfiguration("qz", np.zeros(num_joints))



if __name__ == "__main__":  # pragma nocover

    robot = UArm()
    print("\nRobot loaded successfully:")
    print(robot)

    print("\nDefined configurations:")
    print(f"qz: {robot.qz}")
    if hasattr(robot, 'up'): # Check if 'up' was added
      print(f"up: {robot.up}")

    # Example of accessing links (requires roboticstoolbox)
    print(f"\nNumber of links: {len(robot.links)}")
    # print(f"Link names: {[link.name for link in robot.links]}")
    # print(f"Joint names: {robot.joint_names}") # If available in RTB version

    # Example plotting (requires roboticstoolbox and matplotlib)
    # try:
    #   print("\nPlotting robot in qz configuration...")
    #   robot.plot(robot.qz)
    # except NameError:
    #    print("Plotting requires matplotlib installed.")
    # except Exception as e:
    #    print(f"Error during plotting: {e}")



# import arm
# import spatialmath as sm
# import swift as Swift
# robot= arm.hand('RoboticArm', "qz", 0.01, 0.05)   
# # Fetch can be swapped with other robot arms include Panda, KinoveGen3, UR5, UR3
# # if 'qz' brings up an error then try 'qr' or you can define a custom ready pose using sm
# # qz, and qr are usually predefined poses by the robot models.

# # below I used the spatial maths librarty to define a target pose
# robot.run(sm.SE3.Trans(0.75, 0.8, 0.9) * sm.SE3.RPY(0.2, 0.6, 0.5))
# # arm.q = arm.qz  # Set to zero configuration

# # Initialize Swift simulator
# env = Swift()
# env.launch(realtime=True)

# # Add robot to the simulator
# env.add(arm)

# # Optional: Add an origin frame for reference
# origin = sg.Axes(length=0.2)
# env.add(origin)

# # Step the simulation
# env.step()

# # Pause to view
# time.sleep(5)


# from roboticstoolbox import ERobot
# import swift  # For live 3D visualization
# import time

# # Path to your generated URDF
# urdf_path = "/home/lauren/.local/lib/python3.10/site-packages/rtbdata/xacro/RoboticArm_description/urdf/RoboticArm.urdf"

# # Load robot model
# robot = ERobot.URDF(urdf_path)  # Removed the 'tld' argument

# # Print joint info (optional)
# print(robot)

# # Launch Swift for interactive 3D viewing
# env = swift.Swift()
# env.launch(reload=True)
# env.add(robot)

# # Set joint angles (e.g., to zero position)
# robot.q = [0] * robot.n  # Set all joints to zero

# # Render initial pose
# env.step()

# # Keep open for viewing
# while True:
#     time.sleep(0.05)

# from roboticstoolbox import ERobot
# from spatialmath import SE3
# import swift
# import time


# panda = ERobot.URDF("/home/lauren/.local/lib/python3.10/site-packages/rtbdata/xacro/RoboticArm_description/urdf/RoboticArm.urdf")
# # panda = ERobot.URDF("/home/lauren/.local/lib/python3.10/site-packages/rtbdata/xacro/RoboticArm_description/urdf/RoboticArm.urdf", tld="/home/lauren/.local/lib/python3.10/site-packages/rtbdata/xacro")
# print(panda)
# env =swift.Swift()
# env.launch()
# env.add(panda)

# # panda.q = [0, 0, 0, 0, 0, 0, 0]
# # env.step()

# while True:
#     time.sleep(1)

# import roboticstoolbox as rtb
# import swift


# # Load the robot model from the URDF file
# robot = rtb.models.UArm()

# # Print robot information (optional)
# print(robot)

# # Initialize Swift simulator
# q = [0,0,0,0]
# env = swift.Swift()
# env.launch()
# env.add(robot)
# # Add the robot to the simulator
# robot.q = [0, 0, 0, 0]  # Set initial joint angles to zero
# env.step()
# robot.q = ([10, 10, 10, 10])  # Set joint angles to a specific configuration
# env.step()

# # Keep the simulator open
# input("Press Enter to exit...")

from spatialmath import SE3
import roboticstoolbox as rtb
import swift

robot = rtb.models.DH.Panda()
print(robot)
T = robot.fkine(robot.qz)
print(T)

# IK

T = SE3(0.7, 0.2, 0.1) * SE3.OA([0, 1, 0], [0, 0, -1])
sol = robot.ikine_LMS(T)  # solve IK, ignore additional outputs
print(sol.q)  # display joint angles
# FK shows that desired end-effector pose was achieved
print(robot.fkine(sol.q))


qtraj = rtb.jtraj(robot.qz, sol.q, 50)
robot.plot(qtraj.q, movie="panda1.gif")

# URDF + Swift version
dt = 0.050  # simulation timestep in seconds
robot = rtb.models.URDF.Panda()
print(robot)

env = swift.Swift()   # instantiate 3D browser-based visualizer
env.launch("chrome")        # activate it
env.add(robot)              # add robot to the 3D scene
env.start_recording("panda2", 1 / dt)
for qk in qtraj.q:             # for each joint configuration on trajectory
    robot.q = qk          # update the robot state
    env.step()            # update visualization
env.stop_recording()

# from roboticstoolbox import DHRobot, Link, ERobot
# from spatialmath import SE3
# import swift
# from pathlib import Path
# from spatialgeometry import Mesh

# # Path to STL files (assuming STL files are in a folder named "meshes")
# stl_path = Path("uarm_ros/meshes")

# # Define the DH parameters and links (modify according to your robot)
# # Example: 6 links robot, add more or less as per your requirement
# links = [
#     Link(geometry=[Mesh(filename=str(stl_path / "BaseLink0.stl"), scale=[0.001, 0.001, 0.001])], pose=SE3(0, 0, 0)),
#     Link(geometry=[Mesh(filename=str(stl_path / "BaseLink1.stl"), scale=[0.001, 0.001, 0.001])], pose=SE3(0, 0, 0.1)),
#     Link(geometry=[Mesh(filename=str(stl_path / "BaseLink2.stl"), scale=[0.001, 0.001, 0.001])], pose=SE3(0, 0, 0.2)),
#     Link(geometry=[Mesh(filename=str(stl_path / "BaseLink3.stl"), scale=[0.001, 0.001, 0.001])], pose=SE3(0, 0, 0.3)),
#     Link(geometry=[Mesh(filename=str(stl_path / "BaseLink4.stl"), scale=[0.001, 0.001, 0.001])], pose=SE3(0, 0, 0.4)),
#     Link(geometry=[Mesh(filename=str(stl_path / "BaseLink5.stl"), scale=[0.001, 0.001, 0.001])], pose=SE3(0, 0, 0.5)),
#     Link(geometry=[Mesh(filename=str(stl_path / "BaseLink6.stl"), scale=[0.001, 0.001, 0.001])], pose=SE3(0, 0, 0.6))  # Add more if you have additional links
# ]

# # Create the robot using the DHRobot class and the list of links
# robot = ERobot(links, name='MyRobot')

# # Launch Swift for visualization
# env = swift.Swift()
# env.launch()

# # Add robot to the Swift environment
# env.add(robot)

# # Move the robot to its home configuration (or any other configuration you want)
# robot.q = robot.qz  # This is the home position for the robot

# # Step to update Swift and show the robot
# env.step()

# # Wait until the user presses Enter to close
# input("Press Enter to exit and close Swift...")

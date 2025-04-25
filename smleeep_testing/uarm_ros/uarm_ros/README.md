# 🤖 RoboticArm class with Python Robotics Toolbox

This project implements a custom 6-DOF robotic arm using Peter Corke's [Robotics Toolbox for Python](https://github.com/petercorke/roboticstoolbox-python), including STL mesh visualization in the Swift simulator.

### ✅ System Requirements
To run the custom robotic arm simulation, you'll need the following Python libraries:

- [`roboticstoolbox-python`](https://github.com/petercorke/roboticstoolbox-python): Kinematics, dynamics, and simulation tools for robots.
- [`swift-sim`](https://github.com/petercorke/swift): A lightweight web-based simulator for visualizing robots in 3D.
- [`spatialmath-python`](https://github.com/petercorke/spatialmath-python): Tools for SE3 and SO3 transformations used in robotics.

- Python 3.8+
- pip (Python package manager)
- A modern web browser for Swift visualization (I use chrome)

To install the required libraries:

```bash
pip install roboticstoolbox-python spatialmath-python swift-sim


## 📁 Project Structure
To implement a custom robot, you need to create a Robot.py class file, STL files for the links and a urdf file to define the robot joints.

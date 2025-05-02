import pybullet as p
import pybullet_data
import time

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)
p.loadURDF("plane.urdf")

# Load test boxes stacked vertically
for i in range(10):
    p.loadURDF("./boxes/test_box.urdf", basePosition=[0, 0, 0.05 + i * 0.06])

while True:
    p.stepSimulation()
    time.sleep(1/240)

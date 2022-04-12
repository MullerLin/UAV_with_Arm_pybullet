import time

import pybullet as p
import pybullet_data
from time import sleep

use_gui = True
if use_gui:
    cid = p.connect(p.GUI)
else:
    cid = p.connect(p.DIRECT)

p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
plane_id = p.loadURDF("plane.urdf", useMaximalCoordinates=False)
robot_id = p.loadURDF("r2d2.urdf", basePosition=[0, 0, 0.5], useMaximalCoordinates=False)

p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
p.setGravity(0, 0, -10)
p.setRealTimeSimulation(1)

while True:
    p.stepSimulation()
    keys = p.getKeyboardEvents()

    if ord("w") in keys and keys[ord("w")] & p.KEY_WAS_TRIGGERED:
        print("w KEY_WAS_TRIGGERED")
    elif ord("w") in keys and keys[ord("w")] & p.KEY_IS_DOWN:
        print("w KEY_IS_DOWN")
    elif ord("w") in keys and keys[ord("w")] & p.KEY_WAS_RELEASED:
        print("w KEY_WAS_RELEASED")

    time.sleep(1/480)

p.disconnect(cid)
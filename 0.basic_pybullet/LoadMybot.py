import os
import pybullet as p
import time
import pybullet_data
from time import sleep
import numpy as np
from pprint import pprint

use_gui = True
if use_gui:
    serve_id = p.connect(p.GUI)
else:
    serve_id = p.connect(p.DIRECT)

# 配置渲染机制
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)

# 配置渲染机制
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)


# 添加资源路径
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# 设置环境重力加速度
p.setGravity(0, 0, -10)

# 加载URDF模型，此处是加载蓝白相间的陆地
planeId = p.loadURDF("plane.urdf")

# 加载机器人，并设置加载的机器人的位姿
startPos = [0, -1, 1]
startOrientation = p.getQuaternionFromEuler([0, 0, 0])
#boxId = p.loadURDF("r2d2.urdf", startPos, startOrientation)
drone = np.zeros(3)
drone[0] = p.loadURDF(os.path.dirname(os.path.abspath(__file__))+"/asset/grasp_drone.urdf",  startPos, startOrientation)

startPos = [1, 1, 1]
drone[1] = p.loadURDF(os.path.dirname(os.path.abspath(__file__))+"/asset/hb.urdf",  startPos, startOrientation)

startPos = [1, 0, 1]
arm = p.loadURDF(os.path.dirname(os.path.abspath(__file__))+"/asset/dofbot_arm.urdf",   basePosition=[0,-0.2,0], useFixedBase=True)
dobot = p.loadURDF("/asset/dobot/dobot.urdf",useFixedBase=True)
#box = p.loadURDF("/asset/miniBox.urdf", useFixedBase=True)
# change the appearance of DOFBOT parts
p.changeVisualShape(arm, -1, rgbaColor=[0, 0, 0, 1])
p.changeVisualShape(arm, 0, rgbaColor=[0, 1, 0, 1])
p.changeVisualShape(arm, 1, rgbaColor=[1, 1, 0, 1])
p.changeVisualShape(arm, 2, rgbaColor=[0, 1, 0, 1])
p.changeVisualShape(arm, 3, rgbaColor=[1, 1, 0, 1])
p.changeVisualShape(arm, 4, rgbaColor=[0, 0, 0, 1])
p.changeVisualShape(arm, 5, rgbaColor=[0, 1, 0, 0])
p.changeVisualShape(arm, 6, rgbaColor=[0, 0, 1, 0])
p.changeVisualShape(arm, 7, rgbaColor=[1, 0, 0, 0.5])
# reset pose of all DOFBOT joints
rest_poses_dofbot = [0, 0, 0, 0, 0]  # stay upright

for i in range(5):
    p.resetJointState(arm, i, rest_poses_dofbot[i])

p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
p.setGravity(0, 0, -10)
p.setRealTimeSimulation(0)

while True:
    p.stepSimulation()
    sleep(1 / 240)
# # 开始一千次迭代，也就是一千次交互，每次交互后停顿1/240
# for i in range(1000):
#     p.stepSimulation()
#     time.sleep(1 / 240)


# 获取位置与方向四元数
# cubePos, cubeOrn = p.getBasePositionAndOrientation(drone)
# print("-" * 20)
# print(f"机器人的位置坐标为:{cubePos}\n机器人的朝向四元数为:{cubeOrn}")
# print("-" * 20)

# 断开连接
p.disconnect()
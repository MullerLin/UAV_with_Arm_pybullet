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

# dobot = p.loadURDF("/asset/dobot/dobot.urdf", useFixedBase=True)

# 加载机器人，并设置加载的机器人的位姿
# startPos = [0, -1, 1]
startOrientation = p.getQuaternionFromEuler([0, 0, 0])


drone = np.zeros(3)
startPos = [0, -1.5, 1.5]
drone[0] = p.loadURDF(os.path.dirname(os.path.abspath(__file__))+"/asset/grasp_drone.urdf",  startPos, startOrientation, useFixedBase=True)
#
# startPos = [0, -1, 1.5]
# drone[1] = p.loadURDF(os.path.dirname(os.path.abspath(__file__))+"/asset/hb.urdf",  startPos, startOrientation, useFixedBase=True)

startPos = [0, -0.5, 1.5]
drone[1] = p.loadURDF(os.path.dirname(os.path.abspath(__file__))+"/asset/grasp_drone_2.urdf",  startPos, startOrientation, useFixedBase=True)

startPos = [0, 1, 1.5]
drones = p.loadURDF(os.path.dirname(os.path.abspath(__file__))+"/asset/grasp_drone_3.urdf",  startPos, startOrientation, useFixedBase=True)


startPos = [1, -1, 1]
arm_down = p.loadURDF(os.path.dirname(os.path.abspath(__file__))+"/asset/dofbot_arm_down.urdf",   basePosition=startPos, useFixedBase=True)
startPos = [1, 1, 1]
gripper = p.loadURDF("tray/traybox.urdf",  startPos, startOrientation, useFixedBase=False)
# startPos = [1, 0, 1]
# arm = p.loadURDF(os.path.dirname(os.path.abspath(__file__))+"/asset/dofbot_arm.urdf",   basePosition=startPos, useFixedBase=True)

print(os.path.dirname(os.path.abspath(__file__)))

# change the appearance of DOFBOT parts
# p.changeVisualShape(arm, -1, rgbaColor=[0, 0, 0, 1])
# p.changeVisualShape(arm, 0, rgbaColor=[0, 1, 0, 1])
# p.changeVisualShape(arm, 1, rgbaColor=[1, 1, 0, 1])
# p.changeVisualShape(arm, 2, rgbaColor=[0, 1, 0, 1])
# p.changeVisualShape(arm, 3, rgbaColor=[1, 1, 0, 1])
# p.changeVisualShape(arm, 4, rgbaColor=[0, 0, 0, 1])
# p.changeVisualShape(arm, 5, rgbaColor=[0, 1, 0, 0])
# p.changeVisualShape(arm, 6, rgbaColor=[0, 0, 1, 0])
# p.changeVisualShape(arm, 7, rgbaColor=[1, 0, 0, 0.5])

# reset pose of all DOFBOT joints
rest_poses_dofbot = [0, 1.308, 0.26, 0]  # stay upright
grasp_index = 5
#rest_poses_dofbot = [0, 0, 0, 0]
for i in range(4):
    p.resetJointState(int(drone[1]), (grasp_index+i), rest_poses_dofbot[i])

rest_poses_dofbot_1 = [0, -1.57,  0]
for i in range(3):
    p.resetJointState(drones, (grasp_index+i), rest_poses_dofbot_1[i])

p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
p.setGravity(0, 0, -10)
p.setRealTimeSimulation(0)

while True:
    p.stepSimulation()
    for i in range(4):
        p.resetJointState(int(drone[1]), (grasp_index + i), rest_poses_dofbot[i])
    # for i in range(3):
    #     p.resetJointState(drones, (grasp_index + i), rest_poses_dofbot[i])
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
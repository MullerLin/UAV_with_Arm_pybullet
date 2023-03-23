import math
import os
import pybullet as p
import pybullet_data
import time
import numpy as np
from pprint import pprint

# 连接物理引擎
use_gui = True
if use_gui:
    serve_id = p.connect(p.GUI)
else:
    serve_id = p.connect(p.DIRECT)

# 添加资源路径
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# 配置渲染机制
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)

# 设置重力，加载模型
p.setGravity(0, 0, -10)
_ = p.loadURDF("plane.urdf", useMaximalCoordinates=True)

startPos = [0, -0.5, 1.5]
startOrientation = [0, 0, 0, 1]
print(os.path.dirname(os.path.abspath(__file__)))
drone = p.loadURDF(os.path.dirname(os.path.abspath(__file__))+"/asset/hbarm.urdf",  startPos, startOrientation, useFixedBase=False)

#change the appearance of DOFBOT parts
p.changeVisualShape(drone, -1, rgbaColor=[0.25, 0.25, 0.25, 1])
#p.changeVisualShape(drone, -1, rgbaColor=[0.663, 0.664, 0.665, 1])
p.changeVisualShape(drone, 0, rgbaColor=[0, 1, 0, 0.5])
p.changeVisualShape(drone, 1, rgbaColor=[0, 1, 0, 0.5])
p.changeVisualShape(drone, 2, rgbaColor=[0, 1, 0, 0.5])
p.changeVisualShape(drone, 3, rgbaColor=[0, 1, 0, 0.5])
p.changeVisualShape(drone, 4, rgbaColor=[0, 0, 0, 1])

p.changeVisualShape(drone, 5, rgbaColor=[0.95, 0.95, 0.95, 1])
p.changeVisualShape(drone, 6, rgbaColor=[0.8, 0.8, 0.8, 1])
p.changeVisualShape(drone, 7, rgbaColor=[0.95, 0.95, 0.95, 1])
p.changeVisualShape(drone, 8, rgbaColor=[0.5, 0.5, 0.5, 1])
p.changeVisualShape(drone, 9, rgbaColor=[0, 1, 0, 0])
p.changeVisualShape(drone, 10, rgbaColor=[0, 0, 1, 0])
p.changeVisualShape(drone, 11, rgbaColor=[1, 1, 0.75, 0.1])

# reset pose of all DOFBOT joints
# rest_poses_dofbot = [0, 1.308, 0.262, 0]  # stay upright
# grasp_index = 5
# joint_index = 4
# for i in range(joint_index):
#     p.resetJointState(drone, (grasp_index+i), rest_poses_dofbot[i])

rest_poses_dofbot = [1.32, 0, 0]  # stay upright
grasp_index = 6
joint_index = 3
for i in range(joint_index):
    p.resetJointState(drone, (grasp_index+i), rest_poses_dofbot[i])


p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
p.setGravity(0, 0, -10)
p.setRealTimeSimulation(0)

# 预备工作结束，重新开启渲染
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
# 关闭实时模拟步
p.setRealTimeSimulation(0)

for i in range(10000):
    p.stepSimulation()
    # p.resetBasePositionAndOrientation(drone, [0, -0.5, 1.5], [0, 0, 0, 1])

    target_orn = [0, 0, 0]
    target_orn = p.getQuaternionFromEuler(target_orn)
    # target_position = [j * math.sin(float(i)*math.pi/6) for j in startPos]
    target_position = [1, -1, 1]
    p.resetBasePositionAndOrientation(
        bodyUniqueId=drone,
        posObj=target_position,
        ornObj=target_orn,
    )


    target_pose = [-0.785, 1.57, 0]
    for j in range(joint_index):
        # p.resetJointState(drone, (grasp_index + j), rest_poses_dofbot[j])
        p.setJointMotorControl2(
            bodyUniqueId=drone,
            jointIndex=grasp_index + j,
            controlMode=p.POSITION_CONTROL,
            targetPosition=target_pose[j],
            force=0.5,
        )
        # jointPosition, _, _, _ =p.getJointState(
        #     bodyUniqueId=drone,
        #     jointIndex=grasp_index + j,
        # )
        # print("joint {:d}".format(grasp_index + j),
        #       " value {:+06.2f}".format((jointPosition)))
    p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING)


    time.sleep(1/240)

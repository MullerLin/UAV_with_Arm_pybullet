import pybullet as p
import pybullet_data
from time import sleep
import math

use_gui = True
if use_gui:
    serve_id = p.connect(p.GUI)
else:
    serve_id = p.connect(p.DIRECT)

# 配置渲染机制
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)

# 添加资源路径
p.setAdditionalSearchPath(pybullet_data.getDataPath())
plane_id = p.loadURDF("plane.urdf", useMaximalCoordinates=True)
robot_id = p.loadURDF("r2d2.urdf", basePosition=[0, 0, 0.5], useMaximalCoordinates=True)

# 创建一面墙
visual_shape_id = p.createVisualShape(
    shapeType=p.GEOM_BOX,
    halfExtents=[60, 5, 5]
)

collison_box_id = p.createCollisionShape(
    shapeType=p.GEOM_BOX,
    halfExtents=[60, 5, 5]
)

wall_id = p.createMultiBody(
    baseMass=10000,
    baseCollisionShapeIndex=collison_box_id,
    baseVisualShapeIndex=visual_shape_id,
    basePosition=[0, 10, 5],
    useMaximalCoordinates=True
)

p.setGravity(0, 0, -10)
p.setRealTimeSimulation(1)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

# 设置轮子速度
for i in range(p.getNumJoints(robot_id)):
    if "wheel" in p.getJointInfo(robot_id, i)[1].decode("utf-8"):  # 如果是轮子的关节，则为马达配置参数，否则禁用马达
        p.setJointMotorControl2(
            bodyUniqueId=robot_id,
            jointIndex=i,
            controlMode=p.VELOCITY_CONTROL,
            targetVelocity=30,
            force=60
        )
    else:
        p.setJointMotorControl2(
            bodyUniqueId=robot_id,
            jointIndex=i,
            controlMode=p.VELOCITY_CONTROL,
            force=0
        )

# 如果不需要将激光可视化出来，置为False
useDebugLine = True
hitRayColor = [0, 1, 0]
missRayColor = [1, 0, 0]

rayLength = 15  # 激光长度
rayNum = 16  # 激光数量

while True:
    p.stepSimulation()
    # 获取需要发射得rayNum激光的起点与终点
    begins, _ = p.getBasePositionAndOrientation(robot_id)
    rayFroms = [begins for _ in range(rayNum)]
    rayTos = [
        [
            begins[0] + rayLength * math.cos(2 * math.pi * float(i) / rayNum),
            begins[1] + rayLength * math.sin(2 * math.pi * float(i) / rayNum),
            begins[2]
        ]
        for i in range(rayNum)]

    # 调用激光探测函数
    results = p.rayTestBatch(rayFroms, rayTos)
    print(results[1])
    # 染色前清楚标记
    p.removeAllUserDebugItems()

    # 根据results结果给激光染色
    for index, result in enumerate(results):
        if result[0] == -1:
            p.addUserDebugLine(rayFroms[index], rayTos[index], missRayColor)
        else:
            p.addUserDebugLine(rayFroms[index], rayTos[index], hitRayColor)

p.disconnect(serve_id)
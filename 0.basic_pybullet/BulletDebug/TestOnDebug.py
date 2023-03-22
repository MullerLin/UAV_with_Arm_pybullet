import pybullet as p
import pybullet_data
import time
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
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 1)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)

# 设置重力，加载模型
p.setGravity(0, 0, -10)
_ = p.loadURDF("plane.urdf", useMaximalCoordinates=True)
robot_id = p.loadURDF("r2d2.urdf", useMaximalCoordinates=True)

# 绘制直线
froms = [[1, 1, 0], [-1, 1, 0], [-1, 1, 3], [1, 1, 3]]
tos = [[-1, 1, 0], [-1, 1, 3], [1, 1, 3], [1, 1, 0]]
for f, t in zip(froms, tos):
    p.addUserDebugLine(
        lineFromXYZ=f,
        lineToXYZ=t,
        lineColorRGB=[0, 1, 0],
        lineWidth=2
    )

# 可以使用的关节
available_joints_indexes = [i for i in range(p.getNumJoints(robot_id)) if p.getJointInfo(robot_id, i)[2] != p.JOINT_FIXED]
print(available_joints_indexes)
pprint([p.getJointInfo(robot_id, i)[1] for i in available_joints_indexes])

# 预备工作结束，重新开启渲染
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
# 关闭实时模拟步
p.setRealTimeSimulation(0)

head_joints_indexes = [i for i in available_joints_indexes if "head" in str(p.getJointInfo(robot_id, i)[1])]
target_v = 20                  # 电机达到的预定角速度（rad/s）

# 获取轮子的关节索引
wheel_link_tuples = [(p.getJointInfo(robot_id, i)[0], p.getJointInfo(robot_id, i)[1].decode("utf-8"))   # 0:序号 1:名称
    for i in range(p.getNumJoints(robot_id))
    if "wheel" in p.getJointInfo(robot_id, i)[1].decode("utf-8")]

print("wheel_link_tuple",wheel_link_tuples)
# 添加控件
wheel_velocity_params_ids = [p.addUserDebugParameter(
    paramName=wheel_link_tuples[i][1] + " V",
    rangeMin=-50,
    rangeMax=50,
    startValue=10
) for i in range(4)]

wheel_force_params_ids = [p.addUserDebugParameter(
    paramName=wheel_link_tuples[i][1] + " F",
    rangeMin=-100,
    rangeMax=100,
    startValue=10
) for i in range(4)]

head_link_tuples = [(p.getJointInfo(robot_id, i)[0], p.getJointInfo(robot_id, i)[1].decode("utf-8"))   # 0:序号 1:名称
    for i in range(p.getNumJoints(robot_id))
    if "head" in p.getJointInfo(robot_id, i)[1].decode("utf-8")]

print("head_link_tuple",head_link_tuples)
head_velocity_params_ids = [p.addUserDebugParameter(
    paramName=head_link_tuples[0][1] + " V",
    rangeMin=-50,
    rangeMax=50,
    startValue=0
)]

head_force_params_ids = [p.addUserDebugParameter(
    paramName=head_link_tuples[0][1] + " F",
    rangeMin=-100,
    rangeMax=100,
    startValue=0
)]

# 添加按钮控件
btn = p.addUserDebugParameter(
    paramName="reset",
    rangeMin=1,
    rangeMax=0,
    startValue=0
)

p.setDebugObjectColor(
    objectUniqueId=robot_id,
    linkIndex=-1,
    objectDebugColorRGB=[0, 0, 1]
)

location, _ = p.getBasePositionAndOrientation(robot_id)
text_name_id = p.addUserDebugText(
    text="I'm R2D2",
    textPosition=location,
    textColorRGB=[0, 0, 1],
    textSize=1.2,
)

previous_btn_value = p.readUserDebugParameter(btn)


for i in range(10000):
    p.stepSimulation()



    # 将控件的参数值作为输入控制机器人，先获取各组控件值
    velocity = p.readUserDebugParameter(head_velocity_params_ids[0])
    force = p.readUserDebugParameter(head_force_params_ids[0])
    # 控制速度与力量
    p.setJointMotorControl2(
        bodyIndex=robot_id,
        jointIndex=head_link_tuples[0][0],
        controlMode=p.VELOCITY_CONTROL,
        targetVelocity=velocity,
        force=force
    )

    # 将控件的参数值作为输入控制机器人，先获取各组控件值
    indices = [i for i, _ in wheel_link_tuples]
    velocities = [p.readUserDebugParameter(param_id) for param_id in wheel_velocity_params_ids]
    forces = [p.readUserDebugParameter(param_id) for param_id in wheel_force_params_ids]
    # 根据控件进行控制
    p.setJointMotorControlArray(
        bodyUniqueId=robot_id,
        jointIndices=indices,
        controlMode=p.VELOCITY_CONTROL,
        targetVelocities=velocities,
        forces=forces
    )


    location, _ = p.getBasePositionAndOrientation(robot_id)
    p.resetDebugVisualizerCamera(
        cameraDistance=3,
        cameraYaw=110,
        cameraPitch=-30,
        cameraTargetPosition=location
    )


    # print(location)
    temp = list(location)
    temp[2] = temp[2] + 1.2
    location = tuple(temp)
    text_name_id = p.addUserDebugText(
        text="I'm R2D2",
        textPosition=location,
        textColorRGB=[0, 0, 1],
        textSize=1.2,
        replaceItemUniqueId = text_name_id
    )

    # 如果按钮的累加值发生变化了，说明clicked了
    if p.readUserDebugParameter(btn) != previous_btn_value:
        # 重置速度
        for i in range(p.getNumJoints(robot_id)):
            p.setJointMotorControl2(robot_id, i, p.VELOCITY_CONTROL, 0, 0)
        # 重置位置
        p.resetBasePositionAndOrientation(robot_id, [0, 0, 0.5], [0, 0, 0, 1])
        previous_btn_value = p.readUserDebugParameter(btn)
        p.removeAllUserDebugItems()

    previous_btn_value = p.readUserDebugParameter(btn)

    p.getCameraImage(480, 320)

    time.sleep(1 / 240)         # 模拟器一秒模拟迭代240步

# 断开连接
p.disconnect(serve_id)
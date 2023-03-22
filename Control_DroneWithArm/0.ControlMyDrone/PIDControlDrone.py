import math
import os
import pybullet as p
import pybullet_data
import numpy as np
import time
from pprint import pprint
from Controller.PID_v1 import DroneModel, DroneControl

# from sys import platform
# import pkgutil
# egl = pkgutil.get_loader('eglRenderer')

# 连接物理引擎
use_gui = True
if use_gui:
    serve_id = p.connect(p.GUI)
else:
    serve_id = p.connect(p.DIRECT)

#### Uncomment the following line to use EGL Render Plugin #
#### Instead of TinyRender (CPU-based) in PYB's Direct mode
# if platform == "linux": p.setAdditionalSearchPath(pybullet_data.getDataPath()); plugin = p.loadPlugin(
#     egl.get_filename(), "_eglRendererPlugin"); print("plugin=", plugin)

# 添加资源路径
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# 配置渲染机制
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)

# 设置重力，加载模型
p.setGravity(0, 0, -10)
_ = p.loadURDF("plane.urdf", useMaximalCoordinates=True)

INIT_XYZS = [0, 0, 1]
INIT_RPYS = [0, 0, 0]
startOrientation = p.getQuaternionFromEuler(INIT_RPYS)
print(os.path.dirname(os.path.abspath(__file__)))
drone = p.loadURDF(os.path.dirname(os.path.abspath(__file__))+"/asset/hbarm.urdf",  INIT_XYZS, startOrientation, useFixedBase=False)

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

rest_poses_dofbot = [1.32, 0, 0]  # stay upright
grasp_index = 6
joint_index = 3
for i in range(joint_index):
    p.resetJointState(drone, (grasp_index+i), rest_poses_dofbot[i])


simulation_freq_hz = 240
TIMESTEP = 1 / simulation_freq_hz

control_freq_hz = 120
duration_sec = 12
CTRL_EVERY_N_STEPS = int(np.floor(simulation_freq_hz/control_freq_hz))
PHY_STEPS = simulation_freq_hz * duration_sec
Control_STEPS = control_freq_hz * duration_sec


# 轨迹生成
NUM_WP = control_freq_hz * 12
NUM_TO = control_freq_hz * 10
NUM_HOV = control_freq_hz * 2
Target_POS = [0, 0, 2]
Trajectory_POS = np.zeros((NUM_WP, 3))
for i in range(NUM_TO):
    Trajectory_POS[i, :] = (np.array(Target_POS)) / NUM_TO * (i+1) + np.array(INIT_XYZS)

for i in range (NUM_TO, NUM_WP):
    Trajectory_POS[i, :] = np.array(Target_POS) + np.array(INIT_XYZS)

wp_counter = int(0)

ctrl = DroneControl(drone_model=DroneModel.HBARM)

p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
p.setGravity(0, 0, -10)
p.setTimeStep(TIMESTEP)
# 关闭实时模拟步
p.setRealTimeSimulation(0)


# 预备工作结束，重新开启渲染
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)


for i in range(PHY_STEPS):
    print()
    print("**[START]** TIMESTEP ：", i)

    p.applyExternalForce(
        objectUniqueId=drone,
        linkIndex=-1,
        forceObj=5.0,
        posObj=[0, 0, 0],
        flags=p.LINK_FRAME,
    )

    p.stepSimulation()
    # p.resetBasePositionAndOrientation(drone, [0, -0.5, 1.5], [0, 0, 0, 1])
    target_pose = [0, 0, 0]
    for j in range(joint_index):
        # p.resetJointState(drone, (grasp_index + j), rest_poses_dofbot[j])
        p.setJointMotorControl2(
            bodyUniqueId=drone,
            jointIndex=grasp_index + j,
            controlMode=p.POSITION_CONTROL,
            targetPosition=rest_poses_dofbot[j],
            force=0.5,
        )


    p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING)
    if i % CTRL_EVERY_N_STEPS == 0:
        pos, quat = p.getBasePositionAndOrientation(drone)
        rpy = p.getEulerFromQuaternion(quat)
        vel, ang_v = p.getBaseVelocity(drone)
        target_force, target_torques = ctrl.computeControlFromState(
            control_timestep=CTRL_EVERY_N_STEPS * TIMESTEP ,
            cur_pos=pos,
            cur_quat=quat,
            target_pos=Trajectory_POS[wp_counter, 0:3],
            target_rpy=INIT_RPYS
        )

        p.applyExternalForce(
            objectUniqueId=drone,
            linkIndex=-1,
            forceObj=target_force,
            posObj=[0, 0, 0],
            flags=p.LINK_FRAME,
        )

        p.applyExternalTorque(
            objectUniqueId=drone,
            linkIndex=-1,
            torqueObj=target_torques,
            flags=p.LINK_FRAME,
        )
        print("[DEBUG] velocity of BASE", p.getBaseVelocity(bodyUniqueId=drone))
        print("[INFO] current position : ", pos, "current rpy : ", rpy)
        print("[INFO] target position : ", Trajectory_POS[wp_counter, 0:3], "target rpy : ", INIT_RPYS )
        print("*********************************************************************************************************************************")
        wp_counter = wp_counter + 1



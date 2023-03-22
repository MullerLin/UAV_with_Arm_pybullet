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
# use_gui = False
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

control_freq_hz = 48
duration_sec = 10
CTRL_EVERY_N_STEPS = int(np.floor(simulation_freq_hz/control_freq_hz))
PHY_STEPS = simulation_freq_hz * duration_sec
Control_STEPS = control_freq_hz * duration_sec

To_sec = 2
# 轨迹生成
NUM_WP = control_freq_hz * duration_sec
Trajectory_POS = np.zeros((NUM_WP, 3))
NUM_TO = int (control_freq_hz * To_sec)
# for i in range(NUM_TO):
#     Trajectory_POS[i, :] = (np.array(Target_VEC)) / NUM_TO * (i+1) + np.array(INIT_XYZS)
#
# for i in range (NUM_TO, NUM_WP):
#     Trajectory_POS[i, :] = np.array(Target_VEC) + np.array(INIT_XYZS)

displace = 1
period = int(NUM_WP/(duration_sec/To_sec))
Target_VEC = [displace, displace, displace]
for i in range (period):
    Trajectory_POS[i, :] = np.array(INIT_XYZS)
for i in range (period, 2*period):
    Trajectory_POS[i, :] = np.array(Target_VEC) / NUM_TO * (i+1-NUM_TO) + np.array(INIT_XYZS)
for i in range (2*period, 3*period):
    Trajectory_POS[i, :] = np.array(Target_VEC) + np.array(INIT_XYZS)
pos_drone = np.array(Target_VEC) + np.array(INIT_XYZS)
Target_VEC  = [-displace, -displace, -displace]
for i in range (3*period, 4*period):
    Trajectory_POS[i, :] = np.array(Target_VEC) / NUM_TO * (i+1-3*NUM_TO) + pos_drone
for i in range (4*period, 5*period):
    Trajectory_POS[i, :] = np.array(Target_VEC) + pos_drone
# Target_VEC = [displace, displace, displace]
# for i in range (5*period, 6*period):
#     Trajectory_POS[i, :] = np.array(Target_VEC)  / NUM_TO * (i+1-5*NUM_TO)+ np.array(INIT_XYZS)


wp_counter = int(0)

ctrl = DroneControl(drone_model=DroneModel.HBARM)

p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
p.setGravity(0, 0, -10)
p.setTimeStep(TIMESTEP)
# 关闭实时模拟步
p.setRealTimeSimulation(0)


# 预备工作结束，重新开启渲染
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

# 记录与绘图
LOGGING_FREQ_HZ = 48
state_logger = np.zeros((duration_sec*LOGGING_FREQ_HZ, 30))
timestamps = np.zeros(duration_sec*LOGGING_FREQ_HZ)


for i in range(PHY_STEPS):


    p.stepSimulation()
    p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING)
    # p.resetBasePositionAndOrientation(drone, [0, -0.5, 1.5], [0, 0, 0, 1])
    target_pose = [0, 0, 0]
    for j in range(joint_index):
        # p.resetJointState(drone, (grasp_index + j), rest_poses_dofbot[j])
        p.setJointMotorControl2(
            bodyUniqueId=drone,
            jointIndex=grasp_index + j,
            controlMode=p.POSITION_CONTROL,
            targetPosition=target_pose[j],
            force=0.5,
        )



    if i % CTRL_EVERY_N_STEPS == 0:
        print()
        print("**[START]** TIMESTEP ：", i)
        print("[DEBUG] velocity of BASE", p.getBaseVelocity(bodyUniqueId=drone))

        pos, quat = p.getBasePositionAndOrientation(drone)
        rpy = p.getEulerFromQuaternion(quat)
        vel, ang_v = p.getBaseVelocity(drone)
        target_force, target_torques, target_rpy, pos_e, rpy_e = ctrl.computeControlFromState(
            control_timestep=CTRL_EVERY_N_STEPS * TIMESTEP ,
            cur_pos=pos,
            cur_quat=quat,
            target_pos=Trajectory_POS[wp_counter, 0:3],
            # target_pos=TARGET_POS[wp_counter, 0:3],
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
        print("[INFO] target_force : ", target_force, "target_torques : ", target_torques)
        print("[INFO] current position : X:{:+06.4f}, Y:{:+06.4f}, Z:{:+06.4f}".format(pos[0], pos[1], pos[2]),
              "current rpy : R:{:+06.4f}, Y:{:+06.4f}, P:{:+06.4f}".format(rpy[0], rpy[1], rpy[2]))
        print("[INFO] target position  : X:{:+06.4f}, Y:{:+06.4f}, Z:{:+06.4f}".format(Trajectory_POS[wp_counter, 0],  Trajectory_POS[wp_counter, 1], Trajectory_POS[wp_counter, 2]),
                "target rpy  : R:{:+06.4f}, Y:{:+06.4f}, P:{:+06.4f}".format(INIT_RPYS[0], INIT_RPYS[1], INIT_RPYS[2]))
        print("*********************************************************************************************************************************")


        # 记录

        state_logger[wp_counter, :] = np.hstack([target_force[0:3], target_torques[0:3], pos[0:3], rpy[0:3],
                                                 Trajectory_POS[wp_counter, 0:3], INIT_RPYS[0:3], vel[0:3],
                                                 target_rpy[0:3], pos_e[0:3], rpy_e[0:3]])
        timestamps[wp_counter] = i / simulation_freq_hz

        wp_counter = wp_counter + 1


import matplotlib.pyplot as plt
from cycler import cycler
#### Loop over colors and line styles ######################
plt.rc('axes', prop_cycle=(cycler('color', ['r', 'g', 'b', 'y']) + cycler('linestyle', ['-', '--', ':', '-.'])))
plt.rcParams['figure.figsize']=(25.6, 14.4)
fig, axs = plt.subplots(12, 2)

t = np.arange(0, timestamps.shape[0] / LOGGING_FREQ_HZ, 1 / LOGGING_FREQ_HZ)

#### Column ################################################
col = 0

#### Force_XYZ ###################################################
row = 0

axs[row, col].plot(t, state_logger[:, 0], label="force_x")
axs[row, col].set_xlabel('time')
axs[row, col].set_ylabel('force(N)')

row = 1
axs[row, col].plot(t, state_logger[:, 1], label="force_y")
axs[row, col].set_xlabel('time')
axs[row, col].set_ylabel('force(N)')

row = 2
axs[row, col].plot(t, state_logger[:, 2], label="force_z")
axs[row, col].set_xlabel('time')
axs[row, col].set_ylabel('force(N)')

#### Position ###################################################
row = 3

axs[row, col].plot(t, state_logger[:, 6], label="position_x")
axs[row, col].plot(t, state_logger[:, 12], label="target_x", color="#800080")
axs[row, col].set_xlabel('time')
axs[row, col].set_ylabel('Pos(m)')

row = 4
axs[row, col].plot(t, state_logger[:, 7], label="position_y")
axs[row, col].plot(t, state_logger[:, 13], label="target_y", color="#800080")
axs[row, col].set_xlabel('time')
axs[row, col].set_ylabel('Pos(m)')

row = 5
axs[row, col].plot(t, state_logger[:, 8], label="position_z")
axs[row, col].plot(t, state_logger[:, 14], label="target_z", color="#800080")
axs[row, col].set_xlabel('time')
axs[row, col].set_ylabel('Pos(m)')


#### Euler ###################################################
row = 6

axs[row, col].plot(t, state_logger[:, 9], label="roll_x")
axs[row, col].plot(t, state_logger[:, 15], label="Target_roll_x", color="#800080")
axs[row, col].set_xlabel('time')
axs[row, col].set_ylabel('Ang(rad)')

row = 7
axs[row, col].plot(t, state_logger[:, 10], label="pitch_y")
axs[row, col].plot(t, state_logger[:, 16], label="Target_pitch_y", color="#800080")
axs[row, col].set_xlabel('time')
axs[row, col].set_ylabel('Ang(rad)')

row = 8
axs[row, col].plot(t, state_logger[:, 11], label="yaw_z")
axs[row, col].plot(t, state_logger[:, 17], label="Target_yaw_z", color="#800080")
axs[row, col].set_xlabel('time')
axs[row, col].set_ylabel('Ang(rad)')


#### Error Position ###################################################
row = 9

axs[row, col].plot(t, state_logger[:, 24], label="error_x")
axs[row, col].set_xlabel('time')
axs[row, col].set_ylabel('Error(m)')

row = 10
axs[row, col].plot(t, state_logger[:, 25], label="error_y")
axs[row, col].set_xlabel('time')
axs[row, col].set_ylabel('Error(m)')

row = 11
axs[row, col].plot(t, state_logger[:, 26], label="error_z")
axs[row, col].set_xlabel('time')
axs[row, col].set_ylabel('Error(m)')



#### Column ################################################
col = 1

#### Torque_XYZ ###################################################
row = 0

axs[row, col].plot(t, state_logger[:, 3], label="torque_x")
axs[row, col].set_xlabel('time')
axs[row, col].set_ylabel('force(Nm)')

row = 1
axs[row, col].plot(t, state_logger[:, 4], label="torque_y")
axs[row, col].set_xlabel('time')
axs[row, col].set_ylabel('torque(Nm)')

row = 2
axs[row, col].plot(t, state_logger[:, 5], label="torque_z")
axs[row, col].set_xlabel('time')
axs[row, col].set_ylabel('torque(Nm)')

#### Torque_XYZ ###################################################
row = 3

axs[row, col].plot(t, state_logger[:, 18], label="vel_x")
axs[row, col].set_xlabel('time')
axs[row, col].set_ylabel('vel(m/s)')

row = 4
axs[row, col].plot(t, state_logger[:, 19], label="vel_y")
axs[row, col].set_xlabel('time')
axs[row, col].set_ylabel('vel(m/s)')

row = 5
axs[row, col].plot(t, state_logger[:, 20], label="vel_z")
axs[row, col].set_xlabel('time')
axs[row, col].set_ylabel('vel(m/s)')


#### Euler ###################################################
row = 6

axs[row, col].plot(t, state_logger[:, 9], label="roll_x")
axs[row, col].plot(t, state_logger[:, 21], label="Target_roll_x", color="#800080")
axs[row, col].set_xlabel('time')
axs[row, col].set_ylabel('Ang(rad)')

row = 7
axs[row, col].plot(t, state_logger[:, 10], label="pitch_y")
axs[row, col].plot(t, state_logger[:, 22], label="Target_pitch_y", color="#800080")
axs[row, col].set_xlabel('time')
axs[row, col].set_ylabel('Ang(rad)')

row = 8
axs[row, col].plot(t, state_logger[:, 11], label="yaw_z")
axs[row, col].plot(t, state_logger[:, 23], label="Target_yaw_z", color="#800080")
axs[row, col].set_xlabel('time')
axs[row, col].set_ylabel('Ang(rad)')

#### Error Position ###################################################
row = 9

axs[row, col].plot(t, state_logger[:, 27], label="error_roll")
axs[row, col].set_xlabel('time')
axs[row, col].set_ylabel('Error(rad)')

row = 10
axs[row, col].plot(t, state_logger[:, 28], label="error_pitch")
axs[row, col].set_xlabel('time')
axs[row, col].set_ylabel('Error(rad)')

row = 11
axs[row, col].plot(t, state_logger[:, 29], label="error_yaw")
axs[row, col].set_xlabel('time')
axs[row, col].set_ylabel('Error(rad)')

#### Drawing options #######################################
for i in range(12):
    for j in range(2):
        axs[i, j].grid(True)
        axs[i, j].legend(loc='upper right',
                         frameon=True
                         )
fig.subplots_adjust(left=0.08,
                    bottom=0.05,
                    right=0.96,
                    top=0.96,
                    wspace=0.15,
                    hspace=0.0
                    )
plt.show()
#fig.savefig('./img/images-BeforeModified.png')
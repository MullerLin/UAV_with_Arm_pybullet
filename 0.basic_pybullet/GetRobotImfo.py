import os
import pybullet as p
import pybullet_data
from time import sleep
from pprint import pprint

# 链接物理引擎
p.connect(p.GUI)

# 设置模型加载路径
datapath = pybullet_data.getDataPath()
p.setAdditionalSearchPath(datapath)

# 加载模型
# 加载机器人，并设置加载的机器人的位姿
startPos = [0, 0, 1]
startOrientation = p.getQuaternionFromEuler([0, 0, 0])
#robot_id = p.loadURDF(os.path.dirname(os.path.abspath(__file__))+"/asset/dofbot_arm.urdf",  startPos, startOrientation)
robot_id = p.loadURDF("r2d2.urdf", basePosition=[0, 0, 0.5], useMaximalCoordinates=False)
# 输出基本信息
joint_num = p.getNumJoints(robot_id)
print("robot的节点数量为：", joint_num)

print("robot的关节信息：")
for joint_index in range(joint_num):
    info_tuple = p.getJointInfo(robot_id, joint_index)
    print(f"关节序号：{info_tuple[0]}\n\
            关节名称：{info_tuple[1]}\n\
            关节类型：{info_tuple[2]}\n\
            机器人第一个位置的变量索引：{info_tuple[3]}\n\
            机器人第一个速度的变量索引：{info_tuple[4]}\n\
            保留参数：{info_tuple[5]}\n\
            关节的阻尼大小：{info_tuple[6]}\n\
            关节的摩擦系数：{info_tuple[7]}\n\
            slider和revolute(hinge)类型的位移最小值：{info_tuple[8]}\n\
            slider和revolute(hinge)类型的位移最大值：{info_tuple[9]}\n\
            关节驱动的最大值：{info_tuple[10]}\n\
            关节的最大速度：{info_tuple[11]}\n\
            节点名称：{info_tuple[12]}\n\
            局部框架中的关节轴系：{info_tuple[13]}\n\
            父节点frame的关节位置：{info_tuple[14]}\n\
            父节点frame的关节方向：{info_tuple[15]}\n\
            父节点的索引，若是基座返回-1：{info_tuple[16]}\n\n")

# 可以使用的关节
available_joints_indexes = [i for i in range(p.getNumJoints(robot_id)) if p.getJointInfo(robot_id, i)[2] != p.JOINT_FIXED]
print("可动关节：", available_joints_indexes)
pprint([p.getJointInfo(robot_id, i)[1] for i in available_joints_indexes])
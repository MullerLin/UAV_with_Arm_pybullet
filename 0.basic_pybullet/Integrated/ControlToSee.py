import time
import pybullet as p
import pybullet_data
import numpy as np

BASE_RADIUS = 0.5
BASE_THICKNESS = 0.2

def setCameraPicAndGetPic(robot_id: int, width: int = 224, height: int = 224, physicsClientId: int = 0):
    """
    给合成摄像头设置图像并返回robot_id对应的图像
    摄像头的位置为miniBox前头的位置
    """
    basePos, baseOrientation = p.getBasePositionAndOrientation(robot_id, physicsClientId=physicsClientId)
    # 从四元数中获取变换矩阵，从中获知指向(左乘(1,0,0)，因为在原本的坐标系内，摄像机的朝向为(1,0,0))
    matrix = p.getMatrixFromQuaternion(baseOrientation, physicsClientId=physicsClientId)
    ty_vec = np.array([matrix[1], matrix[4], matrix[7]])  # 变换后的x轴
    tz_vec = np.array([matrix[2], matrix[5], matrix[8]])  # 变换后的z轴

    basePos = np.array(basePos)
    # 摄像头的位置
    # BASE_RADIUS 为 0.5，是机器人底盘的半径。BASE_THICKNESS 为 0.2 是机器人底盘的厚度。
    # 别问我为啥不写成全局参数，因为我忘了我当时为什么这么写的。
    cameraPos = basePos  + BASE_RADIUS * ty_vec + 0.5 * BASE_THICKNESS * tz_vec
    targetPos = cameraPos   + 1 * ty_vec

    viewMatrix = p.computeViewMatrix(
        cameraEyePosition=cameraPos,
        cameraTargetPosition=targetPos,
        cameraUpVector=tz_vec,
        physicsClientId=physicsClientId
    )
    projectionMatrix = p.computeProjectionMatrixFOV(
        fov=50.0,  # 摄像头的视线夹角
        aspect=1.0,
        nearVal=0.01,  # 摄像头焦距下限
        farVal=20,  # 摄像头能看上限
        physicsClientId=physicsClientId
    )

    width, height, rgbImg, depthImg, segImg = p.getCameraImage(
        width=width, height=height,
        viewMatrix=viewMatrix,
        projectionMatrix=projectionMatrix,
        physicsClientId=physicsClientId
    )

    return width, height, rgbImg, depthImg, segImg


if __name__ == '__main__':

    use_gui = True
    if use_gui:
        cid = p.connect(p.GUI)
    else:
        cid = p.connect(p.DIRECT)

    # 配置渲染逻辑
    # p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)  # debug允许

    # 添加资源
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    plane_id = p.loadURDF("plane.urdf", useMaximalCoordinates=False)
    robot_id = p.loadURDF("r2d2.urdf", basePosition=[0, 0, 0.5], useMaximalCoordinates=False)

    # 障碍物
    cube_ind = p.loadURDF('cube.urdf', (3, 1, 2), p.getQuaternionFromEuler([0, 0, 0]))


    shift = [0, 0, 0]
    scale = [1, 1, 1]

    visual_shape_id = p.createVisualShape(
        shapeType=p.GEOM_MESH,
        fileName="sphere_smooth.obj",
        rgbaColor=[1, 1, 1, 1],
        specularColor=[0.4, 0.4, 0],
        visualFramePosition=[0, 0, 0],
        meshScale=scale)
    collision_shape_id = p.createCollisionShape(
        shapeType=p.GEOM_MESH,
        fileName="sphere_smooth.obj",
        collisionFramePosition=[0, 0, 0],
        meshScale=scale)
    p.createMultiBody(
        baseMass=1,
        baseCollisionShapeIndex=collision_shape_id,
        baseVisualShapeIndex=visual_shape_id,
        basePosition=[-2, -1, 1],
        useMaximalCoordinates=True)

    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
    p.setGravity(0, 0, -10)
    p.setRealTimeSimulation(1)

    textColor = [1, 1, 0]

    # 先设置一个空内容
    debug_text_id = p.addUserDebugText(
        text="",
        textPosition=[0, 0, 2],
        textColorRGB=textColor,
        textSize=2.5
    )

    maxV = 20
    maxF = 20

    t = 2  # 左前或右前的轮子的速度差的倍数

    logging_id = p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, "./log/keyboard2.mp4")

    while True:
        p.stepSimulation()

        setCameraPicAndGetPic(robot_id)
        #p.getCameraImage(480, 320)

        key_dict = p.getKeyboardEvents()

        if len(key_dict):
            if p.B3G_UP_ARROW in key_dict and p.B3G_LEFT_ARROW in key_dict:  # 左前
                p.setJointMotorControlArray(  # 2,3为右 6,7为左
                    bodyUniqueId=robot_id,
                    jointIndices=[2, 3],
                    controlMode=p.VELOCITY_CONTROL,
                    targetVelocities=[maxV, maxV],
                    forces=[maxF, maxF]
                )
                p.setJointMotorControlArray(
                    bodyUniqueId=robot_id,
                    jointIndices=[6, 7],
                    controlMode=p.VELOCITY_CONTROL,
                    targetVelocities=[maxV / t, maxV / t],
                    forces=[maxF / t, maxF / t]
                )
                debug_text_id = p.addUserDebugText(
                    text="up + left",
                    textPosition=[0, 0, 2],
                    textColorRGB=textColor,
                    textSize=2.5,
                    replaceItemUniqueId=debug_text_id
                )

            elif p.B3G_UP_ARROW in key_dict and p.B3G_RIGHT_ARROW in key_dict:  # 右前
                p.setJointMotorControlArray(  # 2,3为右 6,7为左
                    bodyUniqueId=robot_id,
                    jointIndices=[6, 7],
                    controlMode=p.VELOCITY_CONTROL,
                    targetVelocities=[maxV, maxV],
                    forces=[maxF, maxF]
                )
                p.setJointMotorControlArray(
                    bodyUniqueId=robot_id,
                    jointIndices=[2, 3],
                    controlMode=p.VELOCITY_CONTROL,
                    targetVelocities=[maxV / t, maxV / t],
                    forces=[maxF / t, maxF / t]
                )
                debug_text_id = p.addUserDebugText(
                    text="up + right",
                    textPosition=[0, 0, 2],
                    textColorRGB=textColor,
                    textSize=2.5,
                    replaceItemUniqueId=debug_text_id
                )

            elif p.B3G_UP_ARROW in key_dict:  # 向前
                p.setJointMotorControlArray(
                    bodyUniqueId=robot_id,
                    jointIndices=[2, 3, 6, 7],
                    controlMode=p.VELOCITY_CONTROL,
                    targetVelocities=[maxV, maxV, maxV, maxV],
                    forces=[maxF, maxF, maxF, maxF]
                )
                debug_text_id = p.addUserDebugText(
                    text="up",
                    textPosition=[0, 0, 2],
                    textColorRGB=textColor,
                    textSize=2.5,
                    replaceItemUniqueId=debug_text_id
                )

            elif p.B3G_DOWN_ARROW in key_dict:  # 向后
                p.setJointMotorControlArray(
                    bodyUniqueId=robot_id,
                    jointIndices=[2, 3, 6, 7],
                    controlMode=p.VELOCITY_CONTROL,
                    targetVelocities=[-maxV / t, -maxV / t, -maxV / t, -maxV / t],
                    forces=[maxF / t, maxF / t, maxF / t, maxF / t]
                )
                debug_text_id = p.addUserDebugText(
                    text="down",
                    textPosition=[0, 0, 2],
                    textColorRGB=textColor,
                    textSize=2.5,
                    replaceItemUniqueId=debug_text_id
                )


            elif p.B3G_LEFT_ARROW in key_dict:  # 原地左转
                p.setJointMotorControlArray(
                    bodyUniqueId=robot_id,
                    jointIndices=[2, 3, 6, 7],
                    controlMode=p.VELOCITY_CONTROL,
                    targetVelocities=[maxV / t, maxV / t, -maxV / t, -maxV / t],
                    forces=[maxF / t, maxF / t, maxF / t, maxF / t]
                )
                debug_text_id = p.addUserDebugText(
                    text="left",
                    textPosition=[0, 0, 2],
                    textColorRGB=textColor,
                    textSize=2.5,
                    replaceItemUniqueId=debug_text_id
                )

            elif p.B3G_RIGHT_ARROW in key_dict:  # 原地右转
                p.setJointMotorControlArray(
                    bodyUniqueId=robot_id,
                    jointIndices=[2, 3, 6, 7],
                    controlMode=p.VELOCITY_CONTROL,
                    targetVelocities=[-maxV / t, -maxV / t, maxV / t, maxV / t],
                    forces=[maxF / t, maxF / t, maxF / t, maxF / t]
                )
                debug_text_id = p.addUserDebugText(
                    text="right",
                    textPosition=[0, 0, 2],
                    textColorRGB=textColor,
                    textSize=2.5,
                    replaceItemUniqueId=debug_text_id
                )
            elif ord("q") in key_dict:
                p.setJointMotorControl2(
                    bodyIndex=robot_id,
                    jointIndex=8,
                    controlMode=p.VELOCITY_CONTROL,
                    targetVelocity=maxV / t,
                    force=maxF / t
                )
                debug_text_id = p.addUserDebugText(
                    text="gripper forward",
                    textPosition=[0, 0, 2],
                    textColorRGB=textColor,
                    textSize=2.5,
                    replaceItemUniqueId=debug_text_id
                )
            elif ord("a") in key_dict:
                p.setJointMotorControl2(
                    bodyIndex=robot_id,
                    jointIndex=8,
                    controlMode=p.VELOCITY_CONTROL,
                    targetVelocity=-maxV / t,
                    force=maxF / t
                )
                debug_text_id = p.addUserDebugText(
                    text="gripper backward",
                    textPosition=[0, 0, 2],
                    textColorRGB=textColor,
                    textSize=2.5,
                    replaceItemUniqueId=debug_text_id
                )

        else:  # 没有按键，则停下
            p.setJointMotorControlArray(
                bodyUniqueId=robot_id,
                jointIndices=[2, 3, 6, 7, 8],
                controlMode=p.VELOCITY_CONTROL,
                targetVelocities=[0, 0, 0, 0, 0],
                forces=[0, 0, 0, 0, 0]
            )
            debug_text_id = p.addUserDebugText(
                text="",
                textPosition=[0, 0, 2],
                textColorRGB=textColor,
                textSize=2.5,
                replaceItemUniqueId=debug_text_id
            )


        time.sleep(1/240)

    p.stopStateLogging(logging_id)

    p.disconnect(cid)
import time

import numpy as np
import pybullet
import pybullet_data


def set_camera(robot_id: int, width: int = 224, height: int = 224, server_id: int = 0):
    position, orientation = pybullet.getBasePositionAndOrientation(robot_id, physicsClientId=server_id)
    r_mat = pybullet.getMatrixFromQuaternion(orientation)
    tx_vec = np.array([r_mat[0], r_mat[3], r_mat[6]])
    ty_vec = np.array([r_mat[1], r_mat[4], r_mat[7]])
    tz_vec = np.array([r_mat[2], r_mat[5], r_mat[8]])
    camera_position = np.array(position)
    target_position = camera_position - 1 * ty_vec

    view_mat = pybullet.computeViewMatrix(cameraEyePosition=camera_position,
                                          cameraTargetPosition=target_position,
                                          cameraUpVector=tz_vec)

    proj_mat = pybullet.computeProjectionMatrixFOV(fov=60.0,  # 摄像头的视线夹角
                                                   aspect=1.0,
                                                   nearVal=0.01,  # 摄像头视距min
                                                   farVal=10)  # 摄像头视距max)

    w, h, rgb, depth, seg = pybullet.getCameraImage(width=width,
                                                    height=height,
                                                    viewMatrix=view_mat,
                                                    projectionMatrix=proj_mat,
                                                    physicsClientId=server_id)

    return w, h, rgb, depth, seg


if __name__ == '__main__':
    client = pybullet.connect(pybullet.GUI)
    pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
    pybullet.setPhysicsEngineParameter(numSolverIterations=10)
    pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_RENDERING, 0)
    # pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_TINY_RENDERER, 0)

    pybullet.setGravity(0, 0, -9.8)
    pybullet.setRealTimeSimulation(1)

    shift = [0, 0, 0]
    scale = [1, 1, 1]

    visual_shape_id = pybullet.createVisualShape(
        shapeType=pybullet.GEOM_MESH,
        fileName="sphere_smooth.obj",
        rgbaColor=[1, 1, 1, 1],
        specularColor=[0.4, 0.4, 0],
        visualFramePosition=[0, 0, 0],
        meshScale=scale)
    collision_shape_id = pybullet.createCollisionShape(
        shapeType=pybullet.GEOM_MESH,
        fileName="sphere_smooth.obj",
        collisionFramePosition=[0, 0, 0],
        meshScale=scale)
    pybullet.createMultiBody(
        baseMass=1,
        baseCollisionShapeIndex=collision_shape_id,
        baseVisualShapeIndex=visual_shape_id,
        basePosition=[-2, -1, 1],
        useMaximalCoordinates=True)

    plane_id = pybullet.loadURDF("plane100.urdf", useMaximalCoordinates=True)
    cube_ind = pybullet.loadURDF('cube.urdf', (3, 1, 2), pybullet.getQuaternionFromEuler([0, 0, 0]))
    r_ind = pybullet.loadURDF('r2d2.urdf', (1, 1, 2), pybullet.getQuaternionFromEuler([0, 0, 1.57]))
    # 创建结束，重新开启渲染
    pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_RENDERING, 1)

    num_joints = pybullet.getNumJoints(r_ind)
    # 获得各关节的信息
    joint_infos = []
    for i in range(num_joints):
        joint_info = pybullet.getJointInfo(r_ind, i)
        if joint_info[2] != pybullet.JOINT_FIXED:
            if 'wheel' in str(joint_info[1]):
                print(joint_info)
                joint_infos.append(joint_info)

    maxforce = 10
    velocity = 3.14
    for i in range(len(joint_infos)):
        pybullet.setJointMotorControl2(bodyUniqueId=r_ind,
                                       jointIndex=joint_infos[i][0],
                                       controlMode=pybullet.VELOCITY_CONTROL,
                                       targetVelocity=velocity,
                                       force=maxforce)

    flag = True
    while True:
        position, orientation = pybullet.getBasePositionAndOrientation(r_ind)
        pybullet.resetDebugVisualizerCamera(cameraDistance=5,
                                            cameraYaw=120,
                                            cameraPitch=-45,
                                            cameraTargetPosition=position)
        pmin, pmax = pybullet.getAABB(r_ind)
        collide_ids = pybullet.getOverlappingObjects(pmin, pmax)
        for collide_id in collide_ids:
            if collide_id[0] != r_ind:
                print('detect robot collide with object id {}!'.format(r_ind))
                flag = False
                break

        set_camera(r_ind)
        pybullet.stepSimulation()
        time.sleep(1. / 240.)
        if not flag:
            break


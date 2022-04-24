import pybullet as p
import pybullet_data
from time import sleep
import numpy as np
import matplotlib.pyplot as plt

use_gui = True
if use_gui:
    cid = p.connect(p.GUI)
else:
    cid = p.connect(p.DIRECT)

# 添加资源
p.setAdditionalSearchPath(pybullet_data.getDataPath())
plane_id = p.loadURDF("plane.urdf")
robot_id = p.loadURDF("r2d2.urdf", basePosition=[0, 0, 0.5])

# 配置渲染逻辑
p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)

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

# 增加文字
p.addUserDebugText(
    text="Destination",
    textPosition=[0, 1, 3],
    textColorRGB=[0, 1, 0],
    textSize=1.2,
)

p.addUserDebugText(
    text="I'm R2D2",
    textPosition=[0, 0, 1.2],
    textColorRGB=[0, 0, 1],
    textSize=1.2,
)

p.setDebugObjectColor(
    objectUniqueId=robot_id,
    linkIndex=-1,
    objectDebugColorRGB=[0, 0, 1]
)

# 添加按钮控件
btn = p.addUserDebugParameter(
    paramName="getCameraImage",
    rangeMin=1,
    rangeMax=0,
    startValue=0
)

previous_btn_value = p.readUserDebugParameter(btn)

p.setGravity(0, 0, -10)
p.setRealTimeSimulation(1)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

while True:
    p.stepSimulation()
    # 如果按钮的累加值发生变化了，说明clicked了
    if p.readUserDebugParameter(btn) != previous_btn_value:
        w, h, rgbPixels, depthPixels, segPixels = p.getCameraImage(800, 600)

        rgbPixels = np.array(rgbPixels)
        depthPixels = np.array(depthPixels)
        segPixels = np.array(segPixels)

        print(rgbPixels.shape)
        print(depthPixels.shape)
        print(segPixels.shape)

        plt.figure(figsize=[12, 9])
        plt.subplot(2, 2, 1)
        plt.imshow(rgbPixels)
        plt.title("rgbPixels")
        plt.axis("off")
        plt.subplot(2, 2, 2)
        plt.imshow(depthPixels, cmap=plt.cm.gray)
        plt.title("depthPixels")
        plt.axis("off")
        plt.subplot(2, 2, 3)
        plt.imshow(segPixels)
        plt.title("segmentationMaskBuffer")
        plt.axis("off")
        plt.show()

        previous_btn_value = p.readUserDebugParameter(btn)

p.disconnect(cid)
import pybullet as p
import pybullet_data
from time import sleep

use_gui = True
if use_gui:
    cid = p.connect(p.GUI)
else:
    cid = p.connect(p.DIRECT)

p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
plane_id = p.loadURDF("plane.urdf", useMaximalCoordinates=False)
robot_id = p.loadURDF("r2d2.urdf", basePosition=[0, 0, 0.5], useMaximalCoordinates=False)

p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
p.setGravity(0, 0, -10)
p.setRealTimeSimulation(1)

text = []  # 存储文本框内容

# 先设置一个空内容
debug_text_id = p.addUserDebugText(
    text="",
    textPosition=[0, 0, 2],
    textColorRGB=[0, 0, 1],
    textSize=1.5
)

while True:
    p.stepSimulation()
    key_dict = p.getKeyboardEvents()  # 获取键盘输入
    if len(key_dict):  # 存在输入
        print(key_dict)
        # 如果有backspace，则删除text的最后一个字符，并重新渲染
        if p.B3G_BACKSPACE in key_dict and key_dict[p.B3G_BACKSPACE] & p.KEY_WAS_TRIGGERED:
            if len(text) != 0:
                text.pop()
                debug_text_id = p.addUserDebugText(
                    text="".join(text),
                    textPosition=[0, 0, 2],
                    textColorRGB=[0, 0, 1],
                    textSize=1.5,
                    replaceItemUniqueId=debug_text_id  # 取代原来的文字
                )
        else:
            for k, v in key_dict.items():
                if v & p.KEY_WAS_TRIGGERED:  # 只考虑刚刚按下的按键
                    if ord("a") <= k <= ord("z"):
                        text.append(chr(k))
                    elif k == p.B3G_SPACE:
                        text.append(" ")
                    elif ord("0") <= k <= ord("9"):
                        text.append(chr(k))

            debug_text_id = p.addUserDebugText(
                text="".join(text),
                textPosition=[0, 0, 2],
                textColorRGB=[0, 0, 1],
                textSize=1.5,
                replaceItemUniqueId=debug_text_id
            )

p.disconnect(cid)
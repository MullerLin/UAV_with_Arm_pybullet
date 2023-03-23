import pybullet as p
import pybullet_envs
from time import sleep
from pybullet_envs.bullet import CartPoleBulletEnv
import gym

cid = p.connect(p.DIRECT)  # 连接服务器
env = CartPoleBulletEnv(renders=True, discrete_actions=False)
# env = gym.make("CartPoleContinuousBulletEnv-v0")


env.render()  # 环境渲染
env.reset()   # 环境重置

for _ in range(100):
    sleep(1 / 60)
    action = env.action_space.sample()
    obs, reward, done, _ = env.step(action)

p.disconnect(cid)
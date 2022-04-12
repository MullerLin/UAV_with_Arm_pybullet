from pybullet_envs.bullet import CartPoleBulletEnv
from stable_baselines3.dqn import DQN
from time import sleep
import pybullet as p

env = CartPoleBulletEnv(renders=True, discrete_actions=True)
model = DQN(policy="MlpPolicy", env=env)
model.load(
    path="./model",
    env=env
)

obs = env.reset()
while True:
    sleep(1 / 60)
    action, state = model.predict(observation=obs)
    print(action)
    obs, reward, done, info = env.step(action)
    if done:
        break
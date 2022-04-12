from pybullet_envs.bullet import CartPoleBulletEnv
from stable_baselines3.dqn import DQN
from time import sleep
import pybullet as p
import sys

def callback(*params):
    info_dict = params[0]
    rewards = info_dict['rewards']
    print(f"episode total reward: {sum(rewards)}")

def callback_test(*params):
    print(params[0])
    print("-" * 20)
    print(params[1])
    sys.exit(-1)

env = CartPoleBulletEnv(renders=False, discrete_actions=True)

model = DQN(policy="MlpPolicy", env=env)

print("开始训练，稍等片刻")
model.learn(total_timesteps=10000, callback=callback)
model.save("./model")
import SimEnv

from stable_baselines3 import PPO
from stable_baselines3.dqn import DQN

env = SimEnv.MySim()

model = DQN(policy="MlpPolicy", env=env)
model.learn(total_timesteps=10000)

obs = env.reset()
# 验证十次
for _ in range(100):
    action, state = model.predict(observation=obs)
    print(action)
    obs, reward, done, info = env.step(action)
    env.render()
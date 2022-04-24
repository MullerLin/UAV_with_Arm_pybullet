from gym_TestToTarget.gym_TestToTarget.envs.TestToTarget_env import ToTarget_env
import sys
import gym
from gym import spaces
import numpy as np

def callback(*params):
    info_dict = params[0]
    num_collected_steps = info_dict['num_collected_steps']
    print(f"TimeStep: {num_collected_steps}", end="")
    rewards = info_dict['rewards']
    print(f" reward: {rewards}")


def callback_test(*params):
    print(params[0])
    print("-" * 20)
    print(params[1])
    sys.exit(-1)


if __name__ == "__main__":
    from stable_baselines3.dqn import DQN
    from stable_baselines3.common.env_checker import check_env

    env = ToTarget_env()
    #print(env.observation_space)
    check_env(env)
    model = DQN(policy="MlpPolicy", env=env)
    model.learn(total_timesteps=1000, callback=callback)

    obs = env.reset()
    # 验证十次
    env = ToTarget_env(gui = True)

    for _ in range(100):
        action, state = model.predict(observation=obs)
        print(action)
        obs, reward, done, info = env.step(action)
        env.render()
        if done:
            break
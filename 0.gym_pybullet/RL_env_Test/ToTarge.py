import gym
from gym import spaces
import numpy as np


class ToTarget(gym.Env):
    def __init__(self, target=5, initpos = 0, gui = False):
        self.action_space = spaces.Discrete(2)  # 离散的动作空间
        # 设置观测空间上下限度
        self.range = 10
        high = np.array(
            [
                self.range,
                self.range,
            ],
            dtype=np.int32
        )
        low = np.array(
            [
                -1,
                -1,
            ],
            dtype=np.int32
        )
        self.observation_space = spaces.Box(low, high, dtype=np.int32)  # 连续的观测空间

        # 初始化
        self.target = target  # 目标位置
        self.initpos = initpos  # 初始位置
        self.state = np.array((self.initpos, self.target))  # 状态
        self.gui = gui  # 是否展现结果

    def step(self, action):

        state, target = self.state  # 获取状态

        # 根据动作进行状态更新
        if action == 0:
            state = state + 1
        elif action == 1:
            state = state - 1

        # 奖励计算
        if target == state:
            reward = 10
        else:
            reward = - abs(target - state)

        info = {}

        # 是否结束
        done = bool(
            state < 0
            or state > self.range
            or state == target
        )

        # 是否展示进程
        if self.gui == True:
            print('Target is ', target, ' ', end="")
            print('State is ', state)
            for i in range(state):
                print('-', end="")
            print()

        self.state = (state, target)
        #print(np.array(self.state))
        return np.array(self.state, dtype=np.int32), reward, done, info

        # self.state = np.array((state, target), dtype=np.int32)
        # # print(np.array(self.state))
        # return self.state, reward, done, info

    def reset(self):
        self.state = (self.initpos, self.target)
        return np.array(self.state, dtype=np.int32)

    def render(self, mode='human'):
        pass

    def seed(self, seed=None):
        pass

import sys

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

    env = ToTarget()
    #print(env.observation_space)
    check_env(env)
    model = DQN(policy="MlpPolicy", env=env)
    model.learn(total_timesteps=500, callback=callback)

    obs = env.reset()
    # 验证十次
    env = ToTarget(gui = True)

    for _ in range(100):
        action, state = model.predict(observation=obs)
        print(action)
        obs, reward, done, info = env.step(action)
        env.render()
        if done:
            break
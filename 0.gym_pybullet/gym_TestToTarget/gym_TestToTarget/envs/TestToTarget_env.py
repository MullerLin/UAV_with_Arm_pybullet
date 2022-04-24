import gym
from gym import spaces
import numpy as np


class ToTarget_env(gym.Env):
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
import gym
from gym import spaces


class MySim(gym.Env):
    def __init__(self):
        self.action_space = spaces.Discrete(5)  # 离散的动作空间，五个动作
        self.observation_space = spaces.Discrete(2)  # 离散的观测空间

    def step(self, action):
        state = 1

        if action == 2:
            reward = 1
        else:
            reward = -1
        done = True
        info = {}
        return state, reward, done, info

    def reset(self):
        state = 0
        return state

    def render(self, mode='human'):
        pass

    def seed(self, seed=None):
        pass


if __name__ == "__main__":
    from stable_baselines3.common.env_checker import check_env

    env = MySim()
    check_env(env)  # 检测环境是都符合stable baseline

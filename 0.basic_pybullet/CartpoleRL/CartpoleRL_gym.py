import gym
import gym.envs.classic_control.cartpole as cartpole

# env = cartpole.CartPoleEnv()
# env = gym.make('CartPole-v0')
env =  gym.envs.classic_control.cartpole.CartPoleEnv()

env.reset()

for _ in range(100):
    env.render()                            # 渲染
    act = env.action_space.sample()         # 在动作空间中随机采样
    obs, reward, done, _ = env.step(act)    # 与环境交互

env.close()
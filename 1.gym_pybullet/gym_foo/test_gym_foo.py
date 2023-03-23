import gym
import gym_foo
from gym import envs

env = gym.make('foo-v0')

envids = [spec.id for spec in envs.registry.all()]
for envid in sorted(envids):
    print(envid)

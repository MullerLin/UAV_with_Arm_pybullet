from gym.envs.registration import register

register(
        id='TestToTarget-v0',
        entry_point='gym_TestToTarget.gym_TestToTarget.envs:ToTarget_env',
)
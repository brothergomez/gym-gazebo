import gym
import gym_gazebo
env = gym.make('Mekamon-v0')
env.reset()
import time

import random

for i in range(100):
    env.reset()
    print("Reset!")
    time.sleep(60)

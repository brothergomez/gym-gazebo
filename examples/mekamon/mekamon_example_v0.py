import random
import time

import gym

import gym_gazebo

env = gym.make('Mekamon-v0')
env.reset()


for i in range(100):
    env.reset()
    print("Reset!")
    for x in range(200):
        observation, reward, done, info = env.step(env.action_space.sample()) # take a random action
        print(observation, reward, done, info)
        # if done: break

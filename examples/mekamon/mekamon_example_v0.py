import gym
import gym_gazebo
env = gym.make('Mekamon-v0')
# env.reset()
import time
import rospy

import random

def main():
    
    for i in range(100):
        time.sleep(5)
       
        print("Reset!")
        for x in range(200):
            observation, reward, done, info = env.step(env.action_space.sample()) # take a random action
            # if done: break

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        raise Exception

import time

import gym
import numpy as np
import roslaunch
import rospy
from control_msgs.msg import JointTrajectoryControllerState
from gazebo_msgs.srv import SetLinkState
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from gym import spaces, utils
from gym.utils import seeding
from mekamon_msgs.msg import JointAngles
from mekamon_msgs.srv import *  # custom message & service types
from std_srvs.srv import Empty
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from gym_gazebo.envs import gazebo_env

# ~/.venv/py3_std/bin/python3 examples/mekamon/mekamon_example_v0.py


class RobotState():

    batt_milliamps = 0  # 16b unsigned
    batt_millivolts = 0

    joints_FLK = 0  # 8b unsigned
    joints_FLT = 0
    joints_FLH = 0

    joints_FRK = 0
    joints_FRT = 0
    joints_FRH = 0

    joints_BLK = 0
    joints_BLT = 0
    joints_BLH = 0

    joints_BRK = 0
    joints_BRT = 0
    joints_BRH = 0

    heading = 0  # 16b unsigned


class GazeboMekamonEnv(gazebo_env.GazeboEnv):

    def __init__(self):
        gazebo_env.GazeboEnv.__init__(self, "GazeboMekamon_v0.launch")

        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.reset_proxy = rospy.ServiceProxy(
            '/gazebo/reset_simulation', Empty)
        self.set_link = rospy.ServiceProxy(
            '/gazebo/set_link_state', SetLinkState)

        rospy.wait_for_service('/gazebo/set_link_state')

        self.reward_range = (-np.inf, np.inf)

        self._seed()
        self.clientSetJointReportRateSvc(5)
        low = 0 * np.ones(12)
        high = 1024 * np.ones(12)

        self.action_space = spaces.Box(low, high, dtype=np.uint16)

        self._pub = rospy.Publisher(
            '/mekamon/joint_states', JointState, queue_size=10)
        self._sub = rospy.Subscriber(
            '/mekamon/joint_states', JointState, self.observation_callback)

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def process_action(self, action):
        robot_state = JointAngles()
        robot_state.front_left_knee = action[0]
        robot_state.front_left_thigh = action[1]
        robot_state.front_left_hip = action[2]
        robot_state.front_right_knee = action[3]
        robot_state.front_right_thigh = action[4]
        robot_state.front_right_hip = action[5]
        robot_state.back_left_knee = action[6]
        robot_state.back_left_thigh = action[7]
        robot_state.back_left_hip = action[8]
        robot_state.back_right_knee = action[9]
        robot_state.back_right_thigh = action[10]
        robot_state.back_right_hip = action[11]
        return robot_state

    def joint_angle_state(self, action):
        joints = ['fl_knee',
                  'fl_hiplink',
                  'fl_hip',
                  'fr_knee',
                  'fr_hiplink',
                  'fr_hip',
                  'bl_knee',
                  'bl_hiplink',
                  'bl_hip',
                  'br_knee',
                  'br_hiplink',
                  'br_hip']
        joint_state = JointState()
        for i in range(0, len(joints)):
            joint_state.name.append(joints[i])
            joint_state.effort.append(action[i] * ((2 * 6.2831) / 1024))

        return joint_state

    def observation_callback(self, message):
        """
        Callback method for the subscriber of JointTrajectoryControllerState
        """
        self._observation_msg = message

    def step(self, action):
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except (rospy.ServiceException) as e:
            print("/gazebo/unpause_physics service call failed")

        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('joints_return', JointAngles, timeout=5)
            except:
                pass
        new_action = self.joint_angle_state(action)
        self._pub.publish(new_action)
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            # resp_pause = pause.call()
            self.pause()
        except (rospy.ServiceException) as e:
            print("/gazebo/unpause_physics service call failed")

        state = action
        reward = 1
        done = False

        return state, reward, done, {}

    def reset(self):

        # Resets the state of the environment and returns an initial observation.
        rospy.wait_for_service('/gazebo/reset_simulation')
        try:
            # reset_proxy.call()
            self.reset_proxy()
        except (rospy.ServiceException) as e:
            print("/gazebo/reset_simulation service call failed")

        # Unpause simulation to make observation
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            # resp_pause = pause.call()
            self.unpause()
        except (rospy.ServiceException) as e:
            print("/gazebo/unpause_physics service call failed")

        # read joint data
        data = None
        while data is None:
            try:
                data = rospy.wait_for_message(
                    'joints_return', JointAngles, timeout=5)
            except:
                pass

        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            # resp_pause = pause.call()
            self.pause()
        except (rospy.ServiceException) as e:
            print("/gazebo/pause_physics service call failed")

        return data

    def clientSetJointReportRateSvc(self, interval):
        # wait for service availability
        # rospy.wait_for_service('set_joint_report_rate')
        try:
            # create a handle for calling the service:
            setJointReportRateSvc = rospy.ServiceProxy(
                'set_joint_report_rate', SetJointReportRate)
            # can now call the service like a function
            setJointReportRateSvc(interval)
            # (ignoring response value from this particular service)
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

import gym
import rospy
import roslaunch
import time
import numpy as np

from gym import utils, spaces
from gym_gazebo.envs import gazebo_env
from std_srvs.srv import Empty

from gym.utils import seeding
from mekamon_msgs.msg import JointAngles
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
from gazebo_msgs.srv import SetLinkState


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

        low = -np.pi/2.0 * np.ones(12)
        high = np.pi/2.0 * np.ones(12)

        self.action_space = spaces.Box(low, high, dtype='float32')

        INITIAL_JOINTS = np.zeros(12)

        JOINT_PUBLISHER = "/joint_states"
        JOINT_SUBSCRIBER = "/joint_states"

        fl_hip_joint = "fl_hip_joint"
        fl_hiplink_joint = "fl_hiplink_joint"
        fl_knee_joint = "fl_knee_joint"
        fr_hip_joint = "fr_hip_joint"
        fr_hiplink_joint = "fr_hiplink_joint"
        fr_knee_joint = "fr_knee_joint"
        bl_hip_joint = "bl_hip_joint"
        bl_hiplink_joint = "bl_hip_joint"
        bl_knee_joint = "bl_knee_joint"
        br_hip_joint = "br_hip_joint"
        br_hiplink_joint = "br_hiplink_joint"
        br_knee_joint = "br_knee_joint"
        body = "body"
        head = "head"

        JOINT_ORDER = [fl_knee_joint,
                       fl_hip_joint,
                       fl_hiplink_joint,
                       fr_knee_joint,
                       fr_hip_joint,
                       fr_hiplink_joint,
                       bl_knee_joint,
                       bl_hip_joint,
                       bl_hiplink_joint,
                       br_knee_joint,
                       br_hip_joint,
                       br_hiplink_joint
                       ]
        reset_condition = {
            'initial_positions': INITIAL_JOINTS,
            'initial_velocities': []
        }

        self._pub = rospy.Publisher(
            JOINT_PUBLISHER, JointAngles, queue_size=1)
        self._sub = rospy.Subscriber(
            JOINT_SUBSCRIBER, JointAngles, self.observation_callback)

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def observation_callback(self, message):
        """
        Callback method for the subscriber of JointTrajectoryControllerState
        """
        self._observation_msg = message

    def get_trajectory_message(self, action, robot_id=0):
        """
        Helper function.
        Wraps an action vector of joint angles into a JointTrajectory message.
        The velocities, accelerations, and effort do not control the arm motion
        """
        # Set up a trajectory message to publish.
        action_msg = JointTrajectory()
        action_msg.joint_names = self.environment['joint_order']
        # Create a point to tell the robot to move to.
        target = JointTrajectoryPoint()
        action_float = [float(i) for i in action]
        target.positions = action_float
        # These times determine the speed at which the robot moves:
        # it tries to reach the specified target position in 'slowness' time.
        target.time_from_start.secs = self.environment['slowness']
        # Package the single point into a trajectory of points with length 1.
        action_msg.points = [target]
        return action_msg

    def take_observation(self):
        """
        Take observation from the environment and return it.
        TODO: define return type
        """
        # Take an observation
        # done = False

        obs_message = self._observation_msg
        print(obs_message)
        if obs_message is None:
            # print("last_observations is empty")
            return None

    def step(self, action):
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except (rospy.ServiceException) as e:
            print("/gazebo/unpause_physics service call failed")

        if self._observation_msg == None:
            while self._observation_msg is None:
                try:
                    self._observation_msg = rospy.wait_for_message(
                        "/joint_states", JointTrajectoryControllerState, timeout=5)
                except:
                    pass
        self._pub.publish(self.get_trajectory_message(action[:12]))
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            # resp_pause = pause.call()
            self.pause()
        except (rospy.ServiceException) as e:
            print("/gazebo/unpause_physics service call failed")

        return state, reward, done, {}

    def reset(self):

        # Resets the state of the environment and returns an initial observation.
        rospy.wait_for_service('/gazebo/pause_physics')
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
                    JOINT_SUBSCRIBER, JointTrajectoryControllerState, timeout=5)
            except:
                pass

        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            # resp_pause = pause.call()
            self.pause()
        except (rospy.ServiceException) as e:
            print("/gazebo/pause_physics service call failed")

        return data

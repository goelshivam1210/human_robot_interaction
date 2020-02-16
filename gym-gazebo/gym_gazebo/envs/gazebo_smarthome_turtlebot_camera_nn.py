###This file is modified to run on a new environment and modifications are made in order to train a new algorithm
# author: Shivam Goel
# email: shivam.goel@wsu.edu
import gym
import rospy
import roslaunch
import time
import numpy as np
import cv2
import sys
import os
import random
import roslib.message
import ast

from gym import utils, spaces
from gym_gazebo.envs import gazebo_env
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from gym.utils import seeding
from cv_bridge import CvBridge, CvBridgeError

import skimage as skimage
from skimage import transform, color, exposure
from skimage.transform import rotate
from skimage.viewer import ImageViewer

from message_converter import _convert_from_ros_type

#from rospy_message_converter import message_converter

#class GazeboCircuit2cTurtlebotCameraNnEnv(gazebo_env.GazeboEnv):
class GazeboHumanSmartHomeTurtlebotLidarCameraEnv(gazebo_env.GazeboEnv):
    def __init__(self):
        print ("SmartHomeCameraLidar Launched!!!!!!!!!!!!!!!!!!!")
        # Launch the simulation with the given launchfile name
        #gazebo_env.GazeboEnv.__init__(self, "GazeboCircuit2cTurtlebotLidar_v0.launch")
        gazebo_env.GazeboEnv.__init__(self, "GazeboHumanSmartHomeTurtlebotLidar_v0.launch")
        self.vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=5)
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        self.pose_sub = rospy.Subscriber('/pose_topic', String, queue_size=5)
        #print (self.pose_sub)
        self.reward_range = (-np.inf, np.inf)

        self._seed()

        self.last50actions = [0] * 50

        self.img_rows = 32
        self.img_cols = 32
        self.img_channels = 1
        
        #self.pose_array = [0.0001] * 18
        self.pose_array = np.zeros(18)
        #self.pose_array.fill(np.zero)
        self.episode_number = 0

    def _get_message_fields(self, message):
        return zip(message.__slots__, message._slot_types)

    
 
    def convert_ros_message_to_dict(self, message):
    #Takes in a ROS message and returns a Python dictionary.

    #Example:
    #    ros_message = std_msgs.msg.String(data="Hello, Robot")
    #    dict_message = convert_ros_message_to_dictionary(ros_message)
    #"""

        dictionary = {}
        message_fields = self. _get_message_fields(message)

        for field_name, field_type in message_fields:
            field_value = getattr(message, field_name)
            dictionary[field_name] = _convert_from_ros_type(field_type, field_value)
        #print "This is the converted {}".format(dictionary)

        return dictionary
    def update_episode_number(self, ep):
        self.episode_number = ep


    def calculate_observation(self,data):
        min_range = 0.21
        done = False
        for i, item in enumerate(data.ranges):
            if (min_range > data.ranges[i] > 0):
                done = True
        return data.ranges,done

    # this function calculates the reward given for detecting the correct pose and punishment for not detecting 
    def calculate_pose_reward(self, pose_data):
        # remove the last element of the array
        #pose_data = pose_data[:-1]
        # case 1: when robot has achieved the best case, faces human from front
        # only eyes, nose, lips, ears, shoulders
        # provide maximum reward 

        #print "Pose data {}".format(pose_data)
        idx_avail_c1 = [0, 1, 2, 5, 14, 15, 16, 17]
        idx_unavail_c1 = [3, 4, 6, 7, 8, 9, 10, 11, 12, 13]
        
        avail_c1 = []
        unavail_c1 = []
        for i in idx_avail_c1:
            avail_c1.append(pose_data[i])
        for i in idx_unavail_c1:
            unavail_c1.append(pose_data[i])


        idx_avail_c2 = [2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 16, 17]
        idx_unavail_c2 = [0, 1, 14, 15]
        
        avail_c2 = []
        unavail_c2 = []
        
        for i in idx_avail_c2:
            avail_c2.append(pose_data[i])
        for i in idx_unavail_c2:
            unavail_c2.append(pose_data[i])

        #print ("POSE_DATA_SUM  {}  AVAIL_C1 {}  UNAVAIL_C1 {}   AVAIL_C2 {}  UNAVAIL_C2 {}".format(sum(pose_data),sum(avail_c1), sum(unavail_c1), sum(avail_c2), sum(unavail_c2)))

        if sum(avail_c1) > 1.6 and sum(unavail_c1) < 2.0:
            reward_pose =1.00

        #case 2: when robot has seen the body, from front
        # eyes, ears, nose lips torso legs detected
        # less than maximum reward, more than least reward
        #print ("SUM OF POSE_DATA === {}".format(sum(pose_data)))
        elif (sum(pose_data) > 3.75):
            reward_pose = 0.75

        # case 3: when robot detected the full back body
        # ears, shoulders legs detected
        # provide least positive reward in this case

        elif (sum(avail_c2)>2.8 and sum(unavail_c2)<0.8):
            reward_pose = 0.45
        # if none of the above conditions hold true then the reward should be negative, reinforcing the robot to find human
        else:
        #print ("C1_avail:{} C2_avail:{} C1_unavail:{} C2_unavail:{}".format(idx_avail_c1, idx_avail_c2, idx_unavail_c1, idx_unavail_c2))
            #print ("Sum (avail C1): {} Sum (avail C2): {} Sum (unavail C1) : {} Sum (unavail C2): {}".format(sum(avail_c1), sum(avail_c2), sum(unavail_c1), sum(unavail_c2)))
            # no negative reward if still not spotted any human
            # check what is the value of sums do we get? 
            reward_pose = 0
            
            '''
            if self.episode_number > 0 and self.episode_number < 500:
                reward_pose = 0 # instead of zero, we can directly give it the sum
            elif self.episode_number > 500 and self.episode_number <1000:
                reward_pose = +0.035
            elif self.episode_number > 1000 and self.episode_number <1750:
                reward_pose = +0.050
            elif self.episode_number > 1750 and self.episode_number < 2500:
                reward_pose = +0.100
            else:
                reward_pose = -0.15
            '''
        # let us first add all the detection points and simply add them to the reward
        #print "Reward_pose====>>>>{} Episode_number=====>>>>{}".format(reward_pose,self.episode_number)      
        return (reward_pose)


    def calculate_pose_data(self, data):
    # we need to process the rospy message data into a dictionary
    # and then convert in to a list comrising of all the pose scores
        #print "DATA in the pose_data function %".format(data)
        data_to_dict = self.convert_ros_message_to_dict(data) 
        #print "DATA_TO_DICT {}".format(data_to_dict)
        # if the dictionary is empty
        if data_to_dict['data'] == '{}':
            data_to_dict = {}
        else:
            data_to_dict = ast.literal_eval(data_to_dict['data'])
            
        if data_to_dict is not None:
            #print data_to_dict
            #print type(data_to_dict)
            
            for key,value in data_to_dict.iteritems():
                self.pose_array[int(key)] = value
        #print (self.pose_array)
    
        return self.pose_array


    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def _step(self, action):
        #print "EPISODE_NUMBER===>>>>{}".format(self.episode_number)
        #print ("Action ========>>>>>{}".format(action))
        pose_reward = 0
        #reward = 0
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except rospy.ServiceException, e:
            print ("/gazebo/unpause_physics service call failed")

        '''# 21 actions
        max_ang_speed = 0.3
        ang_vel = (action-10)*max_ang_speed*0.1 #from (-0.33 to + 0.33)

        vel_cmd = Twist()
        vel_cmd.linear.x = 0.2
        vel_cmd.angular.z = ang_vel
        self.vel_pub.publish(vel_cmd)'''

        # 4 actions
        if action == 0: #FORWARD
            vel_cmd = Twist()
            vel_cmd.linear.x = 0.2
            vel_cmd.angular.z = 0.0
            self.vel_pub.publish(vel_cmd)
        elif action == 1: #LEFT
            vel_cmd = Twist()
            vel_cmd.linear.x = 0.05
            vel_cmd.angular.z = 0.2
            self.vel_pub.publish(vel_cmd)
        elif action == 2: #RIGHT
            vel_cmd = Twist()
            vel_cmd.linear.x = 0.05
            vel_cmd.angular.z = -0.2
            self.vel_pub.publish(vel_cmd)
        elif action == 3: # STOP
            vel_cmd = Twist()
            vel_cmd.linear.x = 0.0
            vel_cmd.angular.z = 0.0
            self.vel_pub.publish(vel_cmd)

        data_scan = None
        data_pose = None
        while data_scan is None:
            try:
                #new data stream needs to be created here
                data_scan = rospy.wait_for_message('/scan', LaserScan, timeout=5)
                data_pose = rospy.wait_for_message('/pose_topic', String, timeout=5)
                #print data_pose
                #two data needs to be merged by appending the array
                #data = rospy.wait_for_message('/scan', LaserScan, timeout=5)
            except:
                pass
        scan_data,done = self.calculate_observation(data_scan)
        scan_data = np.asarray(list(scan_data))
        #data_scan is a tuple of size 100, and we append it with an array of pose confidence
        if data_pose is not None:
            pose_data = self.calculate_pose_data(data_pose)
            state = np.append(scan_data, pose_data)
            pose_reward = self.calculate_pose_reward(pose_data)
        else:
            state = np.append(scan_data, self.pose_array)
        #print ("Pose_reward =====>>>>> {}".format(pose_reward))
        '''#Image data not required in this case
        image_data = None
        success=False
        cv_image = None
        while image_data is None or success is False:
            try:
                image_data = rospy.wait_for_message('/camera/rgb/image_raw', Image, timeout=5)
                h = image_data.height
                w = image_data.width
         pp       cv_image = CvBridge().imgmsg_to_cv2(image_data, "bgr8")
                #temporal fix, check image is not corrupted
                if not (cv_image[h/2,w/2,0]==178 and cv_image[h/2,w/2,1]==178 and cv_image[h/2,w/2,2]==178):
                    success = True
                else:
                    pass
                    #print("/camera/rgb/image_raw ERROR, retrying")
            except:
                pass

        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            #resp_pause = pause.call()
            self.pause()
        except rospy.ServiceException, e:
            print ("/gazebo/pause_physics service call failed")'''

        self.last50actions.pop(0) #remove oldest
        if action == 0:
            self.last50actions.append(0)
        else:
            self.last50actions.append(1)

        action_sum = sum(self.last50actions)


        '''# 21 actions
        if not done:
            # Straight reward = 5, Max angle reward = 0.5
            reward = round(15*(max_ang_speed - abs(ang_vel) +0.0335), 2)
            # print ("Action : "+str(action)+" Ang_vel : "+str(ang_vel)+" reward="+str(reward))
        
            if action_sum > 45: #L or R looping
                #print("90 percent of the last 50 actions were turns. LOOPING")
                reward = -5
        else:
            reward = -200'''

        # Add center of the track reward
        # len(data.ranges) = 100
        laser_len = len(data_scan.ranges)
        left_sum = sum(data_scan.ranges[laser_len-(laser_len/5):laser_len-(laser_len/10)]) #80-90
        right_sum = sum(data_scan.ranges[(laser_len/10):(laser_len/5)]) #10-20

        center_detour = abs(right_sum - left_sum)/5
        
        #pose_reward = self.calculate_pose_reward(pose_data)
        # 4 actions
        if not done:
            if action == 0:
                reward = 1 / float(center_detour+1)
                reward_total = reward+pose_reward
            elif action_sum > 45: #L or R looping
                reward = -0.5
                reward_total = reward+pose_reward
            else: #L or R no looping
                reward = 0.5 / float(center_detour+1)
                reward_total = reward+pose_reward
        # If the robot has a collision
        else:
            reward_total = -1
            reward = -1

        #print("detour= "+str(center_detour)+" :: reward= "+str(reward)+" ::action="+str(action))
        #print ("EPISODE: {} Reward======>>>>>>> {}".format(self.episode_number,reward))
        '''x_t = skimage.color.rgb2gray(cv_image)
        x_t = skimage.transform.resize(x_t,(32,32))
        x_t = skimage.exposure.rescale_intensity(x_t,out_range=(0,255))'''
        
        #cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        #cv_image = cv2.resize(cv_image, (self.img_rows, self.img_cols))
        #cv_image = cv_image[(self.img_rows/20):self.img_rows-(self.img_rows/20),(self.img_cols/10):self.img_cols] #crop image
        #cv_image = skimage.exposure.rescale_intensity(cv_image,out_range=(0,255))

        info = [reward, pose_reward]
        #state = cv_image.reshape(1, 1, cv_image.shape[0], cv_image.shape[1])
        #print (state)
        return np.asarray(state), reward_total, done, info

        # test STACK 4
        #cv_image = cv_image.reshape(1, 1, cv_image.shape[0], cv_image.shape[1])
        #self.s_t = np.append(cv_image, self.s_t[:, :3, :, :], axis=1)
        #return self.s_t, reward, done, {} # observation, reward, done, info

    def _reset(self):
        # update the variable with the latest episode number
        #self.episode_number = ep
       
        #print ("_reset function activated")
        self.last50actions = [0] * 50 #used for looping avoidance

        # Resets the state of the environment and returns an initial observation.
        rospy.wait_for_service('/gazebo/reset_simulation')
        try:
            #reset_proxy.call()
            self.reset_proxy()
        except rospy.ServiceException, e:
            print ("/gazebo/reset_simulation service call failed")

        # Unpause simulation to make observation
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            #resp_pause = pause.call()
            self.unpause()
            print (self.unpause())
        except rospy.ServiceException, e:
            print ("/gazebo/unpause_physics service call failed")

        data_scan = None
        data_pose = None
        #print data
        while data_scan is None:
            try:
                data_scan = rospy.wait_for_message('/scan', LaserScan, timeout = 5)
                #print (data_scan)
                data_pose = rospy.wait_for_message('/pose_array', Posearray, timeout=5)
                #print (data_pose)
            except:
                pass
        
        # dont need this
        ''' 
        image_data = None
        success=False
        cv_image = None
        while image_data is None or success is False:
            try:
                image_data = rospy.wait_for_message('/camera/rgb/image_raw', Image, timeout=5)
                h = image_data.height
                w = image_data.width
                cv_image = CvBridge().imgmsg_to_cv2(image_data, "bgr8")
                #temporal fix, check image is not corrupted
                if not (cv_image[h/2,w/2,0]==178 and cv_image[h/2,w/2,1]==178 and cv_image[h/2,w/2,2]==178):
                    success = True
                else:
                    pass
                    #print("/camera/rgb/image_raw ERROR, retrying")
            except:
                pass
        '''
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            #resp_pause = pause.call()
            self.pause()
        except rospy.ServiceException, e:
            print ("/gazebo/pause_physics service call failed")

        '''x_t = skimage.color.rgb2gray(cv_image)
        x_t = skimage.transform.resize(x_t,(32,32))
        x_t = skimage.exposure.rescale_intensity(x_t,out_range=(0,255))'''


        #cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        #cv_image = cv2.resize(cv_image, (self.img_rows, self.img_cols))
        ##cv_image = cv_image[(self.img_rows/20):self.img_rows-(self.img_rows/20),(self.img_cols/10):self.img_cols] #crop image
        ##cv_image = skimage.exposure.rescale_intensity(cv_image,out_range=(0,255))
        state, done = self.calculate_observation(data_scan)
        state = list(state)
        state = np.asarray(state)
        #print data_pose
        if data_pose is not None:
            pose_data = self.calculate_pose_data(data_pose)
            state = np.append(state, pose_data)
        else:
            state = np.append(state, self.pose_array)
        #print (self.pose_array.shape)

        #print (state.shape)
        #print ("STATE SPACE")
        #print (state)
        #state = cv_image.reshape(1, 1, cv_image.shape[0], cv_image.shape[1])
        return (state)

        # test STACK 4
        #self.s_t = np.stack((cv_image, cv_image, cv_image, cv_image), axis=0)
        #self.s_t = self.s_t.reshape(1, self.s_t.shape[0], self.s_t.shape[1], self.s_t.shape[2])
        #return self.s_t

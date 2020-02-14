#!/usr/bin/env python

import numpy as np
from mpmath import *
from sympy import *
import scipy.optimize as opt
import rospy
from std_msgs.msg import Int32
import time
from friction_finger_gripper.srv import*
# from visualservoing.srv import *

from geometry_msgs.msg import Point
from common_msgs_gl.srv import *
from std_msgs.msg       import Float32
from common_msgs_gl.msg import Motor_position

class FFEnv(object):
    state_size = 7
    action_size = 5

    def __init__(self):
        # ff_info
        self.ff_info = np.zeros(2, dtype=[('t', np.float32), ('o', np.float32), ('g', np.float32)])
        #print("ff_info : ", self.ff_info['t'], self.ff_info['o'], self.ff_info['g'])

        # Initialise the Hardware Actuation Class
        self.actuate = Actuate()

        # Initialising the position of fingers in state 
        self.update_encoder_pos()    

        # Initialising the Object Position
        self.update_current_object_pos()

        # Initialising a goal
        self.goal = {'x': 0., 'y': 0.} # Goal Position
        # Single Goal
        self.ff_info['g'][0] = 1.912
        self.ff_info['g'][1] = 10.737
        # Multi-Goal
        #self.goal = self.get_goal() # Getting a random goal for multi goal RL       
        
        # Delta theta rate = 0.05
        self.dt = 0.05

        # Encoder Limits
        self.e_l_min = 0.41  
        self.e_l_max = 0.66
        self.e_r_min = 0.38
        self.e_r_max = 0.61
    
    def reset(self):
        
        # Set both fingers at initial position
        self.actuate.set_fingers_initial()
        
        # Get Finger Encoder Positions (in degrees)
        # Initialising the position of fingers in state 
        self.update_encoder_pos()

        # Check Object Position 
        # Reset and Get object position
        self.update_current_object_pos()

        # Get goal Position
        # Single Goal
        self.ff_info['g'][0] = 1.912
        self.ff_info['g'][1] = 10.737

        # Set on_goal to zero

        # Concatenate state = {theta_l, theta_r, O_x, O_y, g_x, g_y, on_goal}
        return np.concatenate((self.ff_info['t'][0], 
                               self.ff_info['t'][1], 
                               self.ff_info['o'][0],
                               self.ff_info['o'][1],
                               self.ff_info['g'][0],
                               self.ff_info['g'][1]), axis=None)

    def step(self, action):

        if (action == 0): # Action is not on Sliding Right nor Left
            self.update_encoder_pos()

        elif (action == 1): # Action is on Sliding Left Finger Up
            self.update_encoder_pos()
            t = self.ff_info['t'][1] - self.dt
            
            # Check limits and actuate
            if (t >= self.e_r_min and t <= self.e_r_max):
                self.actuate.slide_left_up(t)

        elif (action == 2): # Action is on Sliding Left Finger Down
            self.update_encoder_pos()
            t = self.ff_info['t'][0] - self.dt

            # Check limits and actuate
            if (t >= self.e_l_min and t <= self.e_l_max):
                self.actuate.slide_left_down(t)

        elif (action == 3): # Action is on Sliding Right Finger Up
            self.update_encoder_pos()
            t = self.ff_info['t'][0] - self.dt

            # Check limits and actuate
            if (t >= self.e_l_min and t <= self.e_l_max):
                self.actuate.slide_right_up(t)

        elif (action == 4): # Action is on Sliding Right Finger Down
            self.update_encoder_pos()
            t = self.ff_info['t'][1] - self.dt

            # Check limits and actuate
            if (t >= self.e_r_min and t <= self.e_r_max):
                self.actuate.slide_right_down(t)
        

    """
    def get_goal(self):

        return 
    """

    ######################### ENCODER ANGLE CONVERSIONS ############################

    def update_current_angle_pos(self):
        theta = self.actuate.encoders_2_angle_conversion()
        self.ff_info['t'][0] = theta['l']
        self.ff_info['t'][1] = theta['r']

    def update_encoder_pos(self):
        self.ff_info['t'][0], self.ff_info['t'][1] = self.actuate.read_pos()

    def update_current_object_pos(self):
        self.actuate.get_Object_pose()
        self.ff_info['o'][0] =  self.actuate.o_x
        self.ff_info['o'][1] =  self.actuate.o_y


class Actuate:

    def __init__(self):
        # Initializing a node
        rospy.init_node('ff_env')

        # Friction Configuration of the finger
        self.finger_state = None
        # 1-> Slide left Down, 2-> Slide Left Up, 3-> Slide Right Down, 4 -> Slide Right Up, 5-> rotate

        # Object Position 
        self.o_x = None
        self.o_y = None
        self.get_Object_pose() # Getting Object Pose

        # Parameters
        # Left Finger 
        self.m1 = -0.002408
        self.c1 = 0.798985
        # Right Finger
        self.m2 = 0.002259
        self.c2 = 0.324572
    

    ############# FUNCTIONS TO SET ACTUATOR CONTROL MODES AND FRICTION SURFACES ###############
    def set_actuator_modes(self, size, modes):
        rospy.wait_for_service("set_operating_mode")
        try:
            client_operating_mode = rospy.ServiceProxy('set_operating_mode', SendIntArray)
            resp1 = client_operating_mode(modes)
            return 1
        except rospy.ServiceException, e:
            print "Actuator modes service call failed"

    def command_position(self, num,position):
        rospy.wait_for_service('cmd_pos_ind')
        try:
            client_position = rospy.ServiceProxy('cmd_pos_ind', SendDoubleArray)
            resp1 = client_position([num, position])
            return 1

        except rospy.ServiceException, e:
            print "Position Service call failed"

    def command_torque(self, num, torque):
        rospy.wait_for_service('cmd_torque_ind')
        try:
            client_torque = rospy.ServiceProxy('cmd_torque_ind', SendDoubleArray)
            resp1 = client_torque([num, torque])
            return 1

        except rospy.ServiceException, e:
            print "Torque Service Call Failed"

    def set_friction_right(self, friction_surface):
        rospy.wait_for_service('Friction_surface_Left')
        try:
            client_right_surface = rospy.ServiceProxy('Friction_surface_Left', SendBool)
            resp1 = client_right_surface(friction_surface)
            return 1

        except rospy.ServiceException, e:
            print "Right Friction Surface failed"

    def set_friction_left(self, friction_surface):
        rospy.wait_for_service('Friction_surface_Right')
        try:
            client_left_surface = rospy.ServiceProxy('Friction_surface_Right', SendBool)
            resp1 = client_left_surface(friction_surface)
            return 1

        except rospy.ServiceException, e:
            print "Left Friction Surface Failed"

	################################### OBJECT POSITION AND ORIENTATION ########################

    def callbackObjectPose(self, msg):
        #rospy.loginfo(str(msg.x) + "  " + str(msg.y))
        self.o_x = msg.x * 100.
        self.o_y = msg.y * 100.

    def get_Object_pose(self):
        rospy.Subscriber("/object_position", Point, self.callbackObjectPose)
        rospy.sleep(0.05)
        
    ##################### READ ENCODER POSITION AND ANGLE CONVERSION ############################

    def read_pos(self): # Reads Position
        rospy.wait_for_service('read_pos')
        try:
            read_position_handler = rospy.ServiceProxy('read_pos', GetDoubleArray)
            value = read_position_handler()
            return value.data
        except rospy.ServiceException, e:
            print ("Service call failed: %s" % e)

    def encoders_2_angle_conversion(self):
        # Encoder values
        enc = self.read_pos()

        theta = {'l':0., 'r':0.}
        (e_l, e_r) = enc
        theta['l'] = (e_l - self.c1)/self.m1
        theta['r'] = (e_r - self.c2)/self.m2
        return theta

    def angle_2_encoder_conversion(self, angles):
        e = [0, 0]
        e[0] = angle[0] *self.m1 + self.c1 
        e[1] = angle[1] *self.m2 + self.c2
        return e

    ################################### SLIDING ACTIONS #########################################

    def slide_left_down(self, theta):

		if self.finger_state != 1:
			modes = [3, 0]			# Set modes - Left -> Position, Right -> Torque (3 -> Position, 0 -> Torque)
			set_modes = self.set_actuator_modes(2, modes)
			set_friction_l = self.set_friction_right(0)
			set_friction_r = self.set_friction_left(1)
			time.sleep(1)
			self.finger_state = 1

		send_torque = self.command_torque(1, 0.15)
		send_pos = self.command_position(0, theta)

    def slide_left_up(self, theta):
		
        if self.finger_state != 2:
            modes = [0, 3]
            set_modes = self.set_actuator_modes(2, modes)
            set_friction_l = self.set_friction_right(0)
            set_friction_r = self.set_friction_left(1)
            time.sleep(1)
            self.finger_state = 1

        send_torque = self.command_torque(0, 0.15)
        send_pos = self.command_position(1, theta)

    def slide_right_down(self, theta):

        if self.finger_state != 3:
            modes = [0, 3]
            set_modes = self.set_actuator_modes(2, modes)
            set_friction_l = self.set_friction_right(1)
            set_friction_r = self.set_friction_left(0)
            time.sleep(1)
            self.finger_state = 2

        send_torque = self.command_torque(0, 0.15)
        send_pos = self.command_position(1, theta)

    def slide_right_up(self, theta):

        if self.finger_state != 4:
            modes = [3, 0]
            set_modes = self.set_actuator_modes(2, modes)
            set_friction_l = self.set_friction_right(1)
            set_friction_r = self.set_friction_left(0)
            time.sleep(0)
            self.finger_state = 2

        send_torque = self.command_torque(1, 0.15)
        send_pos = self.command_position(0, theta)

    ############################## GETTING FINGER TO INITIAL POSITION ############################

    def set_fingers_initial(self):
        # Function : Will set the fingers in initial position given the encoder values
        e1 = 0.5694   # Left Encoder value
        e2 = 0.49938  # Right Encoder value
        rospy.wait_for_service('Hold_object')
        try: 
            set_fingers = rospy.ServiceProxy('Hold_object', Holdcommand)
            resp1 = set_fingers(e1, e2)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

if __name__ == '__main__':
    #act = Actuate()
    
    env = FFEnv()
    #env.reset()
    env.step(2)
    #print(env.actuate.update_encoder_pos())

    """
    # Getting angles from encoders
    (e1, e2) = act.read_pos()
    print(e1)
    print(e2)
    """
    """
    # Getting angles from encoders and converting back to angles in degrees
    theta = act.encoders_2_angle_conversion()
    print(theta['l'])
    print(theta['r'])
    """
    """
    # Getting Object Position
    act.get_Object_pose()
    print(act.o_x)
    print(act.o_y)
    """
    

    
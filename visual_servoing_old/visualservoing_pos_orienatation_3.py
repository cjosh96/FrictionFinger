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

TOLERANCE=0.3
MAX_ITERATIONS=40
JACOBIAN_STEPS=1


[params, ns] = rosparam.load_file('/home/gsathyanarayanan/finger_ws_backup/src/friction_finger_gripper/config/beg.yaml')
for params, ns in paramlist:
    rosparam.upload_params(ns,params)
    a_left = params['a_left']
    b_left = params['b_left']
    a_right = params['a_right']
    b_right = params['b_right']

wp = 6.2
w0 = 2.5
fw = 1.8


def angle_conversion(angle, flag):
    
    angle = 180. * angle / np.pi
    
    # 0-> Left, 1-> Right
    if(flag == 1):
        n_angle =  a_right*angle+ b_right
    else:
        n_angle = a_left*angle + b_left

    return (n_angle)


def encoder_gripper_angle_conversion(enc,flag):
    
    # 0-> Left, 1-> Right
    if(flag==1):
        theta= (enc - b_right)/a_right
    else:
        theta= (enc - b_left)/a_left
    return theta


def slide_left_finger_down(p):

    rospy.wait_for_service('Slide_Left_Finger_Down')

    try:
        slide_left_down = rospy.ServiceProxy('Slide_Left_Finger_Down', PositionCommand)
        print 'step 2', time.time()-start_time
        resp1 = slide_left_down(p)
        print 'step 3', time.time()-start_time
    except rospy.ServiceException, e:
        print ("Service call failed: %s" % e)


def slide_left_finger_up(p):

    rospy.wait_for_service('Slide_Left_Finger_Up')
    
    try:
        slide_left_up = rospy.ServiceProxy('Slide_Left_Finger_Up', PositionCommand)
        print 'step 2', time.time()-start_time
        resp1 = slide_left_up(p)
        print 'step 3', time.time()-start_time
    except rospy.ServiceException, e:
        print ("Service call failed: %s" % e)


def slide_right_finger_down(p):

    rospy.wait_for_service('Slide_Right_Finger_Down')
    
    try:
        slide_right_down = rospy.ServiceProxy('Slide_Right_Finger_Down', PositionCommand)
        print 'step 2', time.time()-start_time
        resp1 = slide_right_down(p)
        print 'step 3', time.time()-start_time
    except rospy.ServiceException, e:
        print ("Service call failed: %s" % e)


def slide_right_finger_up(p):
    
    rospy.wait_for_service('Slide_Right_Finger_Up')
    
    try:
        slide_right_up = rospy.ServiceProxy('Slide_Right_Finger_Up', PositionCommand)
        print 'step 2', time.time()-start_time
        resp1 = slide_right_up(p)
        print 'step 3', time.time()-start_time
    except rospy.ServiceException, e:
        print ("Service call failed: %s" % e)

def rotate_object_anticlockwise(p):
    
    rospy.wait_for_service('Rotate_anticlockwise')
    
    try:
        rotate_anti = rospy.ServiceProxy('Rotate_anticlockwise',PositionCommand)
        resp1 = rotate_anti(p)
    except rospy.ServiceException, e:
        print ("Service call failed: %s"%e)

def rotate_object_clockwise(p): 
    
    rospy.wait_for_service('Rotate_clockwise')
    
    try:
        Rotate_clockwise = rospy.ServiceProxy('Rotate_clockwise',PositionCommand)
        resp1 = Rotate_clockwise(p)
    except rospy.ServiceException, e:
        print ("Service call failed: %s"%e)


def read_pos():
    
    rospy.wait_for_service('read_pos')
    
    try:
        read_position_handler = rospy.ServiceProxy('read_pos', GetDoubleArray)
        values = read_position_handler()
        #print values.data
        return values.data
    except rospy.ServiceException, e:
        print ("Service call failed: %s" % e)





class visual_servoing:

    def __init__(self):
        # Desired Coordinates
        self.X_d = np.array([0,0])
        self.theta_d=0
        # Geometric Parameters
        self.w0 = 2.0
        self.fw = 1.8
        self.wp = 6.2

        self.x = None
        self.y = None
        self.t1 = None
        self.t2 = None
        self.d1 = None
        self.d2 = None
        self.action = None

        self.Block_orientation = None

        #publishers for recording the data
        self.pub = rospy.Publisher('Action', Int32, queue_size=10000)
        self.Motor_value_pub=rospy.Publisher('Finger_motor_position',Motor_position,queue_size=10000)

        #loop rate
        rate = rospy.Rate(20)

    def goal_update(self,goal_x,goal_y,goal_theta):
        self.X_d = np.array([goal_x, goal_y])
        self.theta_d = goal_theta

    def callbackObjectPosition(self, msg):

        self.x = msg.x * 100.
        self.y = msg.y * 100.
        #print 'position =', self.x, self.y
        values=Motor_position()
        val=read_pos()
        values.Left=val[0]
        values.Right=val[1]
        self.Motor_value_pub.publish(values)

    def callbackObjectOrientation(self, msg):
        
     
        #Radians to degree conversion(Values lies between -180 to 180)
        if(msg.data>=0 and msg.data<=3.14):
            self.Block_orientation=msg.data*180/np.pi
        else:
            self.Block_orientation=180+ abs(msg.data)*180/np.pi
        
        #print Block_orientation


    def listener_object_pose(self):
        #rospy.init_node('Vs')
        rospy.Subscriber("/object_position", Point, self.callbackObjectPosition)
        rospy.Subscriber("/object_orientation", Float32, self.callbackObjectOrientation)
        #rospy.spin()

    def translateLeft(self):
        # Center Coordinates
        x_square = (self.d2 + self.w0 / 2.) * np.cos(np.float64(self.t2)) + (self.fw + self.w0 / 2.) * np.sin(np.float64(self.t2))
        y_square = (self.d2 + self.w0 / 2.) * np.sin(np.float64(self.t2)) - (self.fw + self.w0 / 2.) * np.cos(np.float64(self.t2))

        # Calculate theta2, d2
        d2v = np.array([self.d2 * np.cos(np.float64(self.t2)), self.d2 * np.sin(np.float64(self.t2))])
        w0v = np.array([self.w0 * np.sin(np.float64(self.t2)), -self.w0 * np.cos(np.float64(self.t2))])
        wpv = np.array([self.wp, 0.])
        f1v = np.array([self.fw * np.sin(np.float64(self.t2)), -self.fw * np.cos(np.float64(self.t2))])
        av = d2v + f1v + w0v - wpv
        self.d1 = np.sqrt(float(abs((av * av).sum() - self.fw * self.fw)))
        self.t1 = np.arctan2(float(av[1]), float(av[0])) - np.arctan2(self.fw, self.d1)

    def translateRight(self):
        # Center Coordinates of square
        x_square = self.wp + (self.d1 + self.w0 / 2.) * np.cos(self.t1) - (self.w0 / 2. + self.fw) * np.sin(self.t1)
        y_square = (self.d1 + self.w0 / 2.) * np.sin(self.t1) + (self.w0 / 2. + self.fw) * np.cos(self.t1)
        # print 't1 = ', self.t1
        # Calculate theta1, d1
        d1v = np.array([self.d1 * np.cos(self.t1), self.d1 * np.sin(self.t1)])
        w0v = np.array([self.w0 * np.sin(self.t1), self.w0 * np.cos(self.t1)])
        wpv = np.array([self.wp, 0.])
        f2v = np.array([self.fw * np.sin(self.t1), self.fw * np.cos(self.t1)])
        av = d1v - w0v - f2v + wpv
        self.d2 = np.sqrt(float((av * av).sum() - self.fw * self.fw))
        self.t2 = np.arctan2(float(av[1]), float(av[0])) + np.arctan2(self.fw, self.d2)

    #################### Solving using Inverse kinematics using Symbolic Variables ####################################
    '''
    def ik_leftFinger(self):
        t2_sol, d2_sol = symbols('t2_sol d2_sol')
        eqn1 = (d2_sol + self.w0 / 2.) * cos(t2_sol) - (self.fw + self.w0 / 2.) * sin(t2_sol)
        eqn2 = (d2_sol + self.w0 / 2.) * sin(t2_sol) + (self.fw + self.w0 / 2.) * cos(t2_sol)
        eqn3 = (self.x - self.wp)**2 + self.y**2 - eqn1**2 - eqn2**2
        sold2 = solve(eqn3, d2_sol)
        solt2 = solve(eqn1.subs(d2_sol, sold2[1]) - (self.x - self.wp), t2_sol)
        print 'd2, t2 = ', sold2[1], solt2[1]
        d2v = np.array([sold2[1] * cos(solt2[1]), sold2[1] * sin(solt2[1])])
        w0v = np.array([self.w0 * sin(solt2[1]), -self.w0 * cos(solt2[1])])
        wpv = np.array([self.wp, 0.])
        f1v = np.array([self.fw * sin(solt2[1]), -self.fw * cos(solt2[1])])
        av = d2v - f1v - w0v + wpv

        self.d1 = sqrt(float((av * av).sum() - self.fw * self.fw))
        self.t1 = np.arctan2(float(av[1]), float(av[0])) + np.arctan2(float(self.fw), float(self.d1))
        self.d2 = float(sold2[1])
        self.t2 = float(solt2[1])


    def ik_rightFinger(self):
        t1_sol, d1_sol = symbols('t1_sol d1_sol')
        eqn1 = (d1_sol + self.w0 / 2.) * cos(t1_sol) + (self.fw + self.w0 / 2.) * sin(t1_sol)
        eqn2 = (d1_sol + self.w0 / 2.) * sin(t1_sol) - (self.fw + self.w0 / 2.) * cos(t1_sol)
        eqn3 = self.x**2 + self.y**2 - eqn1**2 - eqn2**2
        sold1 = solve(eqn3, d1_sol)
        solt1 = solve(eqn1.subs(d1_sol, sold1[1]) - self.x, t1_sol)
        print 't2, d2 = ', sold1[1], solt1[1]
        d1v = np.array([sold1[1] * cos(solt1[1]), sold1[1] * sin(solt1[1])])
        w0v = np.array([self.w0 * sin(solt1[1]), -self.w0 * cos(solt1[1])])
        wpv = np.array([self.wp, 0.])
        f1v = np.array([self.fw * sin(solt1[1]), -self.fw * cos(solt1[1])])
        av = d1v + f1v + w0v - wpv
        self.t1 = float(solt1[1])
        self.d1 = float(sold1[1])
        self.d2 = sqrt((av * av).sum() - self.fw * self.fw)
        self.t2 = np.arctan2(float(av[1]), float(av[0])) - np.arctan2(float(self.fw), float(self.d2))

    '''

    ####################### Solving Inverse Kinematics Numerically ####################################

    def left_equations_d(self, variables):
        #t2_sol, d2_sol = symbols('t2_sol d2_sol')
        #(t2_sol, d2_sol) = variables
        #t2_sol = symbols('t2_sol')
        t2_sol = 0
        d2_sol = variables
        #print 'x, y = ', self.x, self.y
        eqn1 = (d2_sol + self.w0 / 2.) * cos(t2_sol) - (self.fw + self.w0 / 2.) * sin(t2_sol)
        eqn2 = (d2_sol + self.w0 / 2.) * sin(t2_sol) + (self.fw + self.w0 / 2.) * cos(t2_sol)
        eqn3 = (self.x - self.wp)**2 + self.y**2 - eqn1**2 - eqn2**2
        return simplify(eqn3[0])

    
    def left_equations_theta(self, variables, d2_sol):
        #t2_sol, d2_sol = symbols('t2_sol d2_sol')
        #(t2_sol, d2_sol) = variables
        #t2_sol = symbols('t2_sol')
        t2_sol = variables
        eqn1 = (d2_sol + self.w0 / 2.) * cos(t2_sol) - (self.fw + self.w0 / 2.) * sin(t2_sol)
        return eqn1
    

    def solve_d2(self):
        initial_guess_d2 = 5.8
        # initial_guess_t1 = np.pi/2
        solution = opt.fsolve(self.left_equations_d, initial_guess_d2)
        #solution = opt.fsolve(self.right_equations, initial_guess)
        #if solution[2]==1 and solution[0][0]>0 and solution[0][0]<3.14 and solution[0][1]<3.14 and solution[0][1]>0:
        #if solution[2]==1: 
        sold2 =  solution[0]
        self.d2 = sold2
        #return sold2

    def solve_left(self, variables):
        t2_sol = variables[0]
        self.solve_d2() 
        eqn1 = (self.d2 + self.w0 / 2.) * cos(t2_sol) - (self.fw + self.w0 / 2.) * sin(t2_sol) - (self.x - self.wp)
        return eqn1
    

    def ik_leftFinger(self):
        #t2_sol, d2_sol = symbols('t2_sol d2_sol')
        #eqn1 = (d2_sol + self.w0 / 2.) * cos(t2_sol) - (self.fw + self.w0 / 2.) * sin(t2_sol)
        #eqn2 = (d2_sol + self.w0 / 2.) * sin(t2_sol) + (self.fw + self.w0 / 2.) * cos(t2_sol)
        #eqn3 = (self.x - self.wp)**2 + self.y**2 - eqn1**2 - eqn2**2
        #sold2 = opt.fsolve(eqn3, d2_sol)
        #solt2 = opt.fsolve(eqn1.subs(d2_sol, sold2[1]) - (self.x - self.wp), t2_sol)
        #initial_guess = (np.pi/2, 5.8)
        #initial_guess = (5.8, np.pi/2)
        
        start_time_1 = time.time()
        
        # for i in range(1,5):
        #     initial_guess_t1 = np.pi * i / 10
        #     solution = opt.fsolve(self.solve_left, initial_guess_t1, full_output=True)
        #     if solution[2]==1 and solution[0] > 0 and solution[0] < np.pi:
        #         print initial_guess_t1, solution[0]
        # print time.time() - start_time_1


#commented out with Berk
        start_time_1 = time.time()
        initial_guess_t1 = np.pi/3
        solution = opt.fsolve(self.solve_left, initial_guess_t1)
        # print time.time() - start_time_1
        solt2 = solution[0]
        sold2 = self.d2
        # solt2, sold2 = solution[0], solution[1]
        # d2v = np.array([sold2[1] * cos(solt2[1]), sold2[1] * sin(solt2[1])])
        # w0v = np.array([self.w0 * sin(solt2[1]), -self.w0 * cos(solt2[1])])
        # wpv = np.array([self.wp, 0.])
        # f1v = np.array([self.fw * sin(solt2[1]), -self.fw * cos(solt2[1])])
        # av = d2v - f1v - w0v + wpv

        # self.d1 = sqrt(float((av * av).sum() - self.fw * self.fw))
        # self.t1 = np.arctan2(float(av[1]), float(av[0])) + np.arctan2(float(self.fw), float(self.d1))
        # self.d2 = float(sold2[1])
        # self.t2 = float(solt2[1])
        d2v = np.array([sold2 * cos(solt2), sold2 * sin(solt2)])
        w0v = np.array([self.w0 * sin(solt2), -self.w0 * cos(solt2)])
        wpv = np.array([self.wp, 0.])
        f1v = np.array([self.fw * sin(solt2), -self.fw * cos(solt2)])
        av = d2v - f1v - w0v + wpv

        self.d1 = sqrt(float((av * av).sum() - self.fw * self.fw))
        self.t1 = np.arctan2(float(av[1]), float(av[0])) + np.arctan2(float(self.fw), float(self.d1))
        self.d2 = float(sold2)
        self.t2 = float(solt2)
        print 'left'
        print self.d1, self.d2, self.t1, self.t2

    ##### INVERSE KINEMATICS FOR SLIDING ALONG THE RIGHT FINGER #####
    def right_equations_d(self, variables):

        d1_sol = variables
        
        eqn3 = self.x**2 + self.y**2 - (d1_sol + self.w0 / 2.)**2 - (self.fw + self.w0 / 2.)**2
        return (eqn3[0])

    
    def solve_d1(self):
        
        initial_guess_d1 = 5.8
        
        solution = opt.fsolve(self.right_equations_d, initial_guess_d1, xtol= 1e-5)
        sold1 =  solution[0]

        self.d1 = sold1


    def right_equations_theta(self, variables):
        
        t1_sol = variables[0]
        
        self.solve_d1()
        eqn1 = (self.d1 + self.w0 / 2.) * cos(t1_sol) + (self.fw + self.w0 / 2.) * sin(t1_sol)
        
        return eqn1
    
    def ik_rightFinger(self):
    
        initial_guess_t1 = np.pi/3
        
        solution_t1 = opt.fsolve(self.right_equations_theta, initial_guess_t1, xtol= 1e-5)
        self.t1 = float(solution_t1[0])
        
        d1v = np.array([self.d1 * cos(self.t1), self.d1 * sin(self.t1)])
        w0v = np.array([self.w0 * sin(self.t1), -self.w0 * cos(self.t1)])
        wpv = np.array([self.wp, 0.])
        f1v = np.array([self.fw * sin(self.t1), -self.fw * cos(self.t1)])
        av = d1v + f1v + w0v - wpv

        self.d2 = sqrt((av * av).sum() - self.fw * self.fw)
        self.t2 = np.arctan2(float(av[1]), float(av[0])) - np.arctan2(float(self.fw), float(self.d2))




        
    #### ROTATION #####
    def clockwise(self):
        theta= read_pos()
        Motor_value = theta[0]
        '''
        while Motor_value > 0.65:
            slide_right_finger_up(Motor_value)
            theta= read_pos()
            Motor_value = theta[0] - 0.05
        '''


        
        self.action = 4
        self.pub.publish(self.action)
        
        
        theta = read_pos()
        Motor_value = theta[1]-0.1

        rotate_object_anticlockwise(Motor_value)
        while(1):
                
                theta = read_pos()
               
                Motor_value = theta[1]-0.02
                rotate_object_anticlockwise(Motor_value)
                finger_angle = encoder_gripper_angle_conversion(theta[1],1)
                
                condition1 = (abs(self.Block_orientation-finger_angle))%90 < 7
                condition2 = ((abs(self.Block_orientation-finger_angle)))%90 > 83
                condition3 = (abs(self.Block_orientation+finger_angle))%90 < 7
                condition4 = (abs(self.Block_orientation+finger_angle))%90 > 83
                condition5 = self.Block_orientation <= 180
                condition6 = self.Block_orientation >= 180

                Right_Limit_condition = Motor_value >= 0.65
                
                if ((condition1 or condition2) and condition6 and Right_Limit_condition):
                    break
                if ((condition3 or condition4) and condition5 and Right_Limit_condition):
                    break     
            
           

    def anticlockwise(self):
        
        theta= read_pos()
        Motor_value = theta[1]
        
        # Alternative to move to extreme position
        '''
        while Motor_value > 0.60:
            slide_left_finger_up(Motor_value)
            theta= read_pos()
            Motor_value = theta[1] - 0.05
            
        '''

        
        self.action = 5
        self.pub.publish(self.action)
                
        theta= read_pos()
        Motor_value = theta[0] - 0.07
            
        rotate_object_clockwise(Motor_value)
        
        while(1):
            
            
            theta= read_pos()
            Motor_value = theta[0] - 0.02
            rotate_object_clockwise(Motor_value)
            finger_angle = encoder_gripper_angle_conversion(theta[1],1)
            

            condition1 = (abs(self.Block_orientation-finger_angle))%90 < 10
            condition2 = ((abs(self.Block_orientation-finger_angle)))%90 > 80
            condition3 = (abs(self.Block_orientation+finger_angle))%90 < 10
            condition4 = (abs(self.Block_orientation+finger_angle))%90 > 80
            condition5 = self.Block_orientation <= 180
            condition6 = self.Block_orientation >= 180
            Left_Limit_condition = Motor_value >= 0.50
            
            if ((condition1 or condition2) and condition6 and Left_Limit_condition):
                break
            if ((condition3 or condition4) and condition5 and Left_Limit_condition):
                break
                        



    def controller(self,goal_x,goal_y, goal_theta, TOLERANCE = 0.5):

        print "Visual Servoing started"
        print "Tolerance = ", TOLERANCE

        # Time interval 
        dt_1 = 0.75
        Kp = 0.25
        pos = np.zeros((2,4))
        errors = np.zeros(4)
        self.goal_update(goal_x,goal_y,goal_theta)
        time.sleep(1)
        self.listener_object_pose()
        
        time.sleep(1)
        #self.ik_rightFinger()
        if (self.theta_d== 90):
            self.anticlockwise()
        if (self.theta_d == -90):
            self.clockwise()
        if (self.theta_d == 180):
            self.anticlockwise()
            self.anticlockwise()
        X = np.array([self.x, self.y])
        e = self.X_d - X
        i = 0
        
        while norm(e) > TOLERANCE:
            
            start_time_1 = time.time()

            X = np.array([self.x, self.y])
            e = X - self.X_d

            # Check if object has reached the desired position
            # if norm(e) < TOLERANCE:
            #     print 'x, y = ', self.x, self.y
            #     print 'reached'
            #     return
            
            # Estimate the expected error if object slides on right finger 
            self.ik_rightFinger()
            J_right = np.matrix([[-(self.d1 + self.w0 / 2.0) * sin(self.t1) + (self.w0 / 2.0 + self.fw) * cos(self.t1)], [(self.d1 + self.w0 / 2.0) * cos(self.t1) + (self.w0 / 2.0 + self.fw) * sin(self.t1)]], dtype='float')
            dtheta_right = -np.linalg.pinv(J_right) *  (e.reshape(e.shape[0], 1))
            X_right = np.array([(self.d1 + self.w0 / 2.0) * cos(self.t1 + dtheta_right[0, 0]) + (self.w0 / 2.0 + self.fw) * sin(self.t1 + dtheta_right[0, 0]), (self.d1 + self.w0 / 2.0) * sin(self.t1 + dtheta_right[0, 0]) - (self.w0 / 2.0 + self.fw) * cos(self.t1 + dtheta_right[0, 0])])
            e_right = norm(X_right - self.X_d)
            
            # Estimate the expected error if object slides on left finger
            self.ik_leftFinger()
            J_left = np.matrix([[-(self.d2 + self.w0 / 2.0) * sin(self.t2) - (self.w0 / 2.0 + self.fw) * cos(self.t2)], [(self.d2 + self.w0 / 2.0) * cos(self.t2) - (self.w0 / 2.0 + self.fw) * sin(self.t2)]], dtype='float')
            dtheta_left = -np.linalg.pinv(J_left) *  (e.reshape(e.shape[0], 1))
            X_left = np.array([self.wp + (self.d2 + self.w0 / 2.0) * np.cos(self.t2 + dtheta_left[0,0]) - (self.w0 / 2.0 + self.fw) * sin(self.t2 + dtheta_left[0,0]), (self.d2 + self.w0 / 2.0) * sin(self.t2 + dtheta_left[0,0]) + (self.w0 + self.fw) * cos(self.t2 + dtheta_left[0,0])])
            e_left = norm(X_left - self.X_d)
            
            
            # Executing the action which has the lowest estimated error
            if e_left < e_right:

                self.t2 = self.t2 + Kp*dtheta_left[0, 0]
                self.translateLeft()

                if dtheta_left[0, 0] > 0:
                    
                    self.action = 6
                    self.pub.publish(self.action)
                    slide_left_finger_down(angle_conversion(self.t1,0))
                   
                else:
                    
                    self.action = 8
                    self.pub.publish(self.action)
                    slide_left_finger_up(angle_conversion(self.t2,1))
                    
            else: 

                self.t1 = self.t1 + Kp*dtheta_right[0, 0]
                self.translateRight()     
                
                if dtheta_right[0, 0] < 0:
        
                    self.action = 7
                    self.pub.publish(self.action)
                    slide_right_finger_down(angle_conversion(self.t2,1))
                    
                else:
    
                    self.action = 9
                    self.pub.publish(self.action)
                    slide_right_finger_up(angle_conversion(self.t1,0))
                    

            print 'while loop time', time.time() - start_time_1


            X = np.array([self.x, self.y])
            e = X - self.X_d




def Visual_Servoing(req):

    v = visual_servoing()
    if req.goal_theta == 0:
        v.controller(req.goal_x,req.goal_y,req.goal_theta,req.Tol)

    if req.goal_theta == 180:
        # Find Cartesian coordinates of the extreme position 
        d1 = 7
        t1 = 0.87
        x_square = wp + (d1 + w0 / 2.) * np.cos(t1) - (w0 / 2. + fw) * np.sin(t1)
        y_square = (d1 + w0 / 2.) * np.sin(t1) + (w0 / 2. + fw) * np.cos(t1)
        
        v.controller(x_square,y_square,0, 1.5)
        v.controller(x_square,y_square, 90, 200)
        v.controller(x_square,y_square,0, 1.5)
        v.controller(req.goal_x,req.goal_y,90,req.Tol)

    if req.goal_theta == 90:
        # Find Cartesian coordinates of the extreme position
        d1 = 7
        t1 = 0.87
        x_square = wp + (d1 + w0 / 2.) * np.cos(t1) - (w0 / 2. + fw) * np.sin(t1)
        y_square = (d1 + w0 / 2.) * np.sin(t1) + (w0 / 2. + fw) * np.cos(t1)
        
        x_square = 8.0
        y_square = 7.0
        v.controller(x_square,y_square,0, 1.5)
        v.controller(req.goal_x,req.goal_y,req.goal_theta,req.Tol)

    if req.goal_theta == -90:
        # Find Cartesian coordinates of the extreme position
        d2 = 7
        t2 = 2.267
        x_square = (d2 + w0 / 2.) * np.cos(np.float64(t2)) + (fw + w0 / 2.) * np.sin(np.float64(t2))
        y_square = (d2 + w0 / 2.) * np.sin(np.float64(t2)) - (fw + w0 / 2.) * np.cos(np.float64(t2))
        
        v.controller(x_square,y_square,0, 1.5)
        v.controller(req.goal_x,req.goal_y,req.goal_theta,req.Tol)

    return 1

def Visual_Sevoing_server():
    rospy.init_node('Visual_Servoing')
    vs=rospy.Service('Visual_servoing',Visual_servo_goal,Visual_Servoing)
    rospy.spin()


if __name__ == '__main__':
    Visual_Sevoing_server()
   


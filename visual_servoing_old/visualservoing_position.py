#!/usr/bin/env python

import numpy as np
from mpmath import *
from sympy import *
import rospy
import time
from friction_finger_gripper.srv import*
# from visualservoing.srv import *

from geometry_msgs.msg import Point
from common_msgs_gl.srv import *
from std_msgs.msg import Int32
TOLERANCE=0.5
MAX_ITERATIONS=20
JACOBIAN_STEPS=3

# def angle_conversion(angle, flag):
#     angle = 180. * angle / np.pi
#     # 0-> Left, 1-> Right
#     print 'angle = ', angle
#     if(flag == 1):
#         n_angle = 0.002574*angle + 0.56208
#     else:
#         n_angle = -0.00292*angle + 0.509542
#     print("n_angle = ", n_angle)
#     return (n_angle)

def angle_conversion(angle, flag):
    angle = 180. * angle / np.pi
    # 0-> Left, 1-> Right
    print 'angle = ', angle
    if(flag == 1):
        n_angle =  0.002244*angle+ 0.563388
    else:
        n_angle = -0.002223*angle + 1.043235
    print("n_angle = ", n_angle)
    return (n_angle)



def encoder_gripper_angle_conversion(enc,flag):
    # if(flag==1):
    #     theta=-(enc-1.03396)/0.0029
    # else:
    #     theta=(enc-0.14021)/0.00389
    # return theta
    if(flag==1):
        theta=(enc-0.563388)/0.002244
    else:
        theta=-(enc-1.043235)/0.002223
    return theta



def slide_left_finger_down(p):
    # Call Left_Slide_Down(till left most position) Assume t1 = pi/6
    rospy.wait_for_service('Slide_Left_Finger_Down')
    try:
        slide_left_down = rospy.ServiceProxy('Slide_Left_Finger_Down', PositionCommand)
        resp1 = slide_left_down(p)
    except rospy.ServiceException, e:
        print ("Service call failed: %s" % e)


def slide_left_finger_up(p):
    rospy.wait_for_service('Slide_Left_Finger_Up')
    try:
        slide_left_up = rospy.ServiceProxy('Slide_Left_Finger_Up', PositionCommand)
        resp1 = slide_left_up(p)
    except rospy.ServiceException, e:
        print ("Service call failed: %s" % e)


def slide_right_finger_down(p):
    # Call Left_Slide_Down(till left most position) Assume t1 = pi/6
    rospy.wait_for_service('Slide_Right_Finger_Down')
    try:
        slide_right_down = rospy.ServiceProxy('Slide_Right_Finger_Down', PositionCommand)
        resp1 = slide_right_down(p)
    except rospy.ServiceException, e:
        print ("Service call failed: %s" % e)


def slide_right_finger_up(p):
    rospy.wait_for_service('Slide_Right_Finger_Up')
    try:
        slide_right_up = rospy.ServiceProxy('Slide_Right_Finger_Up', PositionCommand)
        resp1 = slide_right_up(p)
    except rospy.ServiceException, e:
        print ("Service call failed: %s" % e)

def read_pos():
    rospy.wait_for_service('read_pos')
    try:
        read_position_handler = rospy.ServiceProxy('read_pos', GetDoubleArray)
        values = read_position_handler()
        print values.data
        return values.data
    except rospy.ServiceException, e:
        print ("Service call failed: %s" % e)






class Finger:

    def __init__(self):
        # Desired Coordinates
        self.X_d = np.array([0,0])
        # Geometric Parameters
        self.w0 = 2.5
        self.fw = 1.8
        self.wp = 6.2

        self.x = None
        self.y = None
        self.t1 = None
        self.t2 = None
        self.d1 = None
        self.d2 = None
        self.config = None

    def goal_update(self,goal_x,goal_y):
        self.X_d = np.array([goal_x, goal_y])

    def callback(self, msg):

        self.x = msg.x * 100.
        self.y = msg.y * 100.

    def listener(self):
        #rospy.init_node('Vs')
        rospy.Subscriber("/object_position", Point, self.callback)

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
        print 'inside translateleft'
        print 'av=',av
        self.d1 = np.sqrt(float(abs((av * av).sum() - self.fw * self.fw)))
        print 'd1=',self.d1
        self.t1 = np.arctan2(float(av[1]), float(av[0])) - np.arctan2(self.fw, self.d1)

    def translateRight(self):
        # Center Coordinates of square
        x_square = self.wp + (self.d1 + self.w0 / 2.) * np.cos(self.t1) - (self.w0 / 2. + self.fw) * np.sin(self.t1)
        y_square = (self.d1 + self.w0 / 2.) * np.sin(self.t1) + (self.w0 / 2. + self.fw) * np.cos(self.t1)
        print 't1 = ', self.t1
        # Calculate theta1, d1
        d1v = np.array([self.d1 * np.cos(self.t1), self.d1 * np.sin(self.t1)])
        w0v = np.array([self.w0 * np.sin(self.t1), self.w0 * np.cos(self.t1)])
        wpv = np.array([self.wp, 0.])
        f2v = np.array([self.fw * np.sin(self.t1), self.fw * np.cos(self.t1)])
        av = d1v - w0v - f2v + wpv
        self.d2 = np.sqrt(float((av * av).sum() - self.fw * self.fw))
        self.t2 = np.arctan2(float(av[1]), float(av[0])) + np.arctan2(self.fw, self.d2)

    def ik_rightFinger(self):
        t2_sol, d2_sol = symbols('t2_sol d2_sol')
        eqn1 = (d2_sol + self.w0 / 2.) * cos(t2_sol) - (self.fw + self.w0 / 2.) * sin(t2_sol)
        eqn2 = (d2_sol + self.w0 / 2.) * sin(t2_sol) + (self.fw + self.w0 / 2.) * cos(t2_sol)
        eqn3 = (self.x - self.wp)**2 + self.y**2 - eqn1**2 - eqn2**2
        sold2 = solve(eqn3, d2_sol)
        solt2 = solve(eqn1.subs(d2_sol, sold2[1]) - (self.x - self.wp), t2_sol)

        d2v = np.array([sold2[1] * cos(solt2[1]), sold2[1] * sin(solt2[1])])
        w0v = np.array([self.w0 * sin(solt2[1]), -self.w0 * cos(solt2[1])])
        wpv = np.array([self.wp, 0.])
        f1v = np.array([self.fw * sin(solt2[1]), -self.fw * cos(solt2[1])])
        av = d2v - f1v - w0v + wpv

        self.d1 = sqrt(float((av * av).sum() - self.fw * self.fw))
        self.t1 = np.arctan2(float(av[1]), float(av[0])) + np.arctan2(float(self.fw), float(self.d1))
        self.d2 = float(sold2[1])
        self.t2 = float(solt2[1])


    def ik_leftFinger(self):
        t1_sol, d1_sol = symbols('t1_sol d1_sol')
        eqn1 = (d1_sol + self.w0 / 2.) * cos(t1_sol) + (self.fw + self.w0 / 2.) * sin(t1_sol)
        eqn2 = (d1_sol + self.w0 / 2.) * sin(t1_sol) - (self.fw + self.w0 / 2.) * cos(t1_sol)
        eqn3 = self.x**2 + self.y**2 - eqn1**2 - eqn2**2
        sold1 = solve(eqn3, d1_sol)
        solt1 = solve(eqn1.subs(d1_sol, sold1[1]) - self.x, t1_sol)
        print 'sold1=',sold1
        print 'solt1=',solt1
        d1v = np.array([sold1[1] * cos(solt1[1]), sold1[1] * sin(solt1[1])])
        w0v = np.array([self.w0 * sin(solt1[1]), -self.w0 * cos(solt1[1])])
        wpv = np.array([self.wp, 0.])
        f1v = np.array([self.fw * sin(solt1[1]), -self.fw * cos(solt1[1])])
        av = d1v + f1v + w0v - wpv
        print 'check= ', solt1[1]
        self.t1 = float(solt1[1])
        self.d1 = float(sold1[1])
        self.d2 = sqrt((av * av).sum() - self.fw * self.fw)
        self.t2 = np.arctan2(float(av[1]), float(av[0])) - np.arctan2(float(self.fw), float(self.d2))
        print 'theta1 = ', solt1[1]
        print 'theta2 = ', self.t2
        print 'd1 = ', self.d1
        print 'd2 = ', self.d2
        print 'x = ', self.x
        print 'y = ', self.y

    def visualCtrl(self,goal_x,goal_y):
        print "Visual Servoing started"
        # Time interval
        dt = 0.3
        # Visual Control
        #theta = 0.814407
        #rospy.init_node('Vs')
        #Update goal
        self.goal_update(goal_x,goal_y)
        for i in range(MAX_ITERATIONS):
            self.listener()
            time.sleep(2)
            # Right Finger Slide
            for i in range(JACOBIAN_STEPS):
                #dtheta = [[1]]
                #n=0
            
                #while abs(dtheta[0][0]) >=0.01 and n<JACOBIAN_STEPS:   
                
                self.ik_rightFinger()
                theta= read_pos()
                print theta
                # self.t1=theta[0]
                # self.t2=theta[1]
                #self.t1=encoder_gripper_angle_conversion(theta[0],0)*np.pi/180
                #self.t2=encoder_gripper_angle_conversion(theta[1],1)*np.pi/180
                #print self.t1
                X = np.array([self.x, self.y])
                e = X - self.X_d
                if norm(e) < TOLERANCE:
                    time.sleep(1)
                    print 'x, y = ', self.x, self.y
                    print 'reached'
                    return
                J = np.matrix([[-(self.d1 + self.w0 / 2.0) * sin(self.t1) + (self.w0 / 2 + self.fw) * cos(self.t1)], [(self.d1 + self.w0 / 2.0) * cos(self.t1) + (self.w0 / 2 + self.fw) * sin(self.t1)]], dtype='float')
                dtheta = -np.linalg.pinv(J) * e.reshape(e.shape[0], 1) * dt
                self.t1 = self.t1 + dtheta[0, 0]
                config = 1
                self.translateRight()
                print 'dtheta =', dtheta
                
                if dtheta[0, 0] < 0:
                    # print 'x position = ', self.x, 'y position = ', self.y
                    # print 't1 =', self.t1, 't2 = ', self.t2
                    # print 'right slide down'
                    print "SLide Right Down "
                    # slide_right_finger_down(theta[1]-0.04)
                    slide_right_finger_down(angle_conversion(self.t2,1))

                else:
                    # print 'x position = ', self.x, 'y position = ', self.y
                    # print 't1 =', self.t1, 't2 = ', self.t2
                    # print 'right slide up'\
                    print "SLide Right Up "
                    # slide_right_finger_up(theta[0]-0.04)
                    slide_right_finger_up(angle_conversion(self.t1,0))
                #n = n+1
                # Send to the robot??
                #time.sleep(1)
                print 'x, y = ', self.x, self.y
            # Left Finger Slide
            for i in range(JACOBIAN_STEPS):
                #dtheta = [[1]]
                #n=0
                #while abs(dtheta[0][0]) >=0.01 and n<JACOBIAN_STEPS:
                #dtheta = [[1]]
                #n=0
                #abs(dtheta[0][0]) >=0.01 and 
                #while n<JACOBIAN_STEPS:
                
                self.ik_leftFinger()
                print 'theta2 = ', self.t2
                #theta= read_pos()
                #print theta
                # self.t1=theta[0]
                # self.t2=theta[1]
                #self.t1=encoder_gripper_angle_conversion(theta[0],0)*np.pi/180
                #self.t2=encoder_gripper_angle_conversion(theta[1],1)*np.pi/180
                #print self.t1
                X = np.array([self.x, self.y])
                e = X - self.X_d
                if norm(e) < TOLERANCE:
                    time.sleep(1)
                    print 'x, y = ', self.x, self.y
                    print 'reached'
                    return
                J = np.matrix([[-(self.d2 + self.w0 / 2.0) * sin(self.t2) - (self.w0 / 2 + self.fw) * cos(self.t2)], [(self.d2 + self.w0 / 2) * cos(self.t2) - (self.w0 / 2 + self.fw) * sin(self.t2)]], dtype='float')
                dtheta = -np.linalg.pinv(J) * e.reshape(e.shape[0], 1) * dt
                self.t2 = self.t2 + dtheta[0, 0]
                config = 2
                self.translateLeft()
                print 'theta1 = ', self.t1
                print 'theta2 = ', self.t2
                print 'd1 = ', self.d1
                print 'd2 = ', self.d2
                print 'dtheta =', dtheta
                if dtheta[0, 0] > 0:
                    # print 'x position = ', self.x, 'y position = ', self.y
                    # print 't1 =', self.t1, 't2 = ', self.t2
                    # print 'left slide down'
                    print "SLide Left Down "
                    # slide_left_finger_down(theta[0]-0.04)
                    slide_left_finger_down(angle_conversion(self.t1,0))

                else:
                    # print 'x position = ', self.x, 'y position = ', self.y
                    # print 't1 =', self.t1, 't2 = ', self.t2
                    # print 'left slide up'
                    print "SLide Left Up "
                    # slide_left_finger_up(theta[1]-0.04)
                    slide_left_finger_up(angle_conversion(self.t2,1))

                #time.sleep(1)
                print 'x, y = ', self.x, self.y
                #n = n + 1
                # for i in range(2):

                #     theta = theta - 0.1
                #     slide_right_finger_up(theta)
                #     print theta
                #     time.sleep(2)
                    
                    
                    # theta = theta + 0.1
                    # slide_left_finger_down(angle_conversion(theta, 0))
                    

def Visual_Servoing(req):
    f = Finger()
    f.visualCtrl(req.goal_x,req.goal_y)
    return 1

def Visual_Sevoing_server():
    rospy.init_node('Visual_Servoing')
    vs=rospy.Service('Visual_servoing',Visual_servo_goal,Visual_Servoing)
    rospy.spin()

if __name__ == '__main__':

   Visual_Sevoing_server()
   


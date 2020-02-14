#!/usr/bin/env python

import rospy
from common_msgs_gl.srv import *


def command_position(num,position):
	rospy.wait_for_service('cmd_pos_ind')
	try:
		client_position = rospy.ServiceProxy('cmd_pos_ind', SendDoubleArray)
		resp1 = client_position(num, position)



if __name__ == '__main__':
	for i in range(0.85,0.75,-0.01):
		send_pos = command_position(1, i)	

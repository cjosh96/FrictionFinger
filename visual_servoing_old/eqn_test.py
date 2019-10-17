import numpy as np
import scipy.optimize as opt
from mpmath import *
from sympy import *
import time

w0 = 2.5
wp = 6.0
x = 2.5
y = 6.8
fw = 1.6

def right_equations(variables):
    #t1_sol = symbols('t1_sol')
    (t1_sol, d1_sol) = variables
    #d1_sol = variables
    #print 'x, y = ', self.x, self.y
    eqn1 = (d1_sol + w0 / 2.) * cos(t1_sol) + (fw + w0 / 2.) * sin(t1_sol)
    eqn2 = (d1_sol + w0 / 2.) * sin(t1_sol) - (fw + w0 / 2.) * cos(t1_sol)
    eqn3 = x**2 + y**2 - eqn1**2 - eqn2**2
    
    return [eqn1, eqn2]

def left_equations(variables):
    #t2_sol, d2_sol = symbols('t2_sol d2_sol')
    #(t2_sol, d2_sol) = variables
    t2_sol = np.pi/3
    d2_sol = variables
    #print 'x, y = ', self.x, self.y
    eqn1 = (d2_sol + w0 / 2.) * cos(t2_sol) - (fw + w0 / 2.) * sin(t2_sol)
    eqn2 = (d2_sol + w0 / 2.) * sin(t2_sol) + (fw + w0 / 2.) * cos(t2_sol)
    eqn3 = (x - wp)**2 + y**2 - eqn1**2 - eqn2**2
    return simplify(eqn3[0])


def ik_leftFinger():
    #t2_sol, d2_sol = symbols('t2_sol d2_sol')
    #eqn1 = (d2_sol + self.w0 / 2.) * cos(t2_sol) - (self.fw + self.w0 / 2.) * sin(t2_sol)
    #eqn2 = (d2_sol + self.w0 / 2.) * sin(t2_sol) + (self.fw + self.w0 / 2.) * cos(t2_sol)
    #eqn3 = (self.x - self.wp)**2 + self.y**2 - eqn1**2 - eqn2**2
    #sold2 = opt.fsolve(eqn3, d2_sol)
    #solt2 = opt.fsolve(eqn1.subs(d2_sol, sold2[1]) - (self.x - self.wp), t2_sol)
    #initial_guess = (np.pi/2, 5.8)
    #initial_guess = (5.8, np.pi/2)

    initial_guess = 5.8
    solution = opt.fsolve(left_equations, initial_guess)
    #solution = opt.fsolve(self.right_equations, initial_guess)
    #if solution[2]==1 and solution[0][0]>0 and solution[0][0]<3.14 and solution[0][1]<3.14 and solution[0][1]>0:
    #if solution[2]==1: 
    print solution


if __name__ == '__main__':
	start_time = time.time()
	ik_leftFinger()
	print time.time()-start_time
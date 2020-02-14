import numpy as np
import scipy.optimize as opt
from mpmath import *
# Geometric Parameters
wp = 6.2
w0 = 2.5
fw = 1.8

x = -2.2
y = 4.2

##### INVERSE KINEMATICS FOR SLIDING ALONG THE RIGHT FINGER #####
def left_equations_d(variables):
	d2_sol = variables
	eqn3 = (x - wp)**2 + y**2 - (d2_sol + w0 / 2.)**2 - (fw + w0 / 2.)**2
	return (eqn3[0])


def solve_d2():
	initial_guess_d2 = 5.8
	solution = opt.fsolve(left_equations_d, initial_guess_d2) 
	sold2 =  solution[0]

	d2 = sold2
	return d2



def solve_left(variables):
	t2_sol = variables[0]
	d2 = solve_d2() 
	eqn1 = (d2 + w0 / 2.) * cos(t2_sol) - (fw + w0 / 2.) * sin(t2_sol) - (x - wp)
	return eqn1


def ik_leftFinger_n():

	initial_guess_t1 = np.pi/3
	solution_t2 = opt.fsolve(solve_left, initial_guess_t1)
	print 't2 = ', solution_t2
	for i in range(1,5):
		initial_guess_t1 = np.pi * i / 5
		solution = opt.fsolve(solve_left, initial_guess_t1, full_output=True)
		# print 't2 = ', solution_t2[0]
		if solution[2]==1 and solution[0] > 0 and solution[0] < np.pi:
			solt2 = solution_t2[0]
			t2 = float(solt2)


	d2v = np.array([d2 * np.cos(np.float64(t2)), d2 * np.sin(np.float64(t2))])
	w0v = np.array([w0 * np.sin(np.float64(t2)), -w0 * np.cos(np.float64(t2))])
	wpv = np.array([wp, 0.])
	f1v = np.array([fw * np.sin(np.float64(t2)), -fw * np.cos(np.float64(t2))])
	av = d2v - f1v - w0v + wpv
	d1 = np.sqrt(float(abs((av * av).sum() - fw * fw)))
	t1 = np.arctan2(float(av[1]), float(av[0])) + np.arctan2(fw, d1)


##### INVERSE KINEMATICS FOR SLIDING ALONG THE RIGHT FINGER #####
def right_equations_d(variables):

	d1_sol = variables
    
	eqn3 = x**2 + y**2 - (d1_sol + w0 / 2.)**2 - (fw + w0 / 2.)**2
	return (eqn3[0])


def solve_d1():
    
	initial_guess_d1 = 5.8
    
	solution = opt.fsolve(right_equations_d, initial_guess_d1, xtol= 1e-5)
	sold1 =  solution[0]

	d1 = sold1
	return d1


def right_equations_theta(variables):
    
	t1_sol = variables[0]
    
	d1 = solve_d1()
	eqn1 = (d1 + w0 / 2.) * cos(t1_sol) + (fw + w0 / 2.) * sin(t1_sol) - x
	return eqn1

def ik_rightFinger_n():

	initial_guess_t1 = np.pi/3
	solution_t1 = opt.fsolve(right_equations_theta, initial_guess_t1, xtol= 1e-5)
	print 't1 =', solution_t1
	for i in range(1,10):
		print i
		initial_guess_t1 = np.pi * i / 10
		solution = opt.fsolve(right_equations_theta, initial_guess_t1, full_output=True)
		print 't1 = ', float(solution_t1[0])
		if solution[2]==1 and solution[0] > 0 and solution[0] < np.pi:
			t1 = float(solution_t1[0])
			

	d1v = np.array([d1 * np.cos(t1), d1 * np.sin(t1)])
	w0v = np.array([w0 * np.sin(t1), w0 * np.cos(t1)])
	wpv = np.array([wp, 0.])
	f2v = np.array([fw * np.sin(t1), fw * np.cos(t1)])
	av = d1v + w0v + f2v - wpv
	d2 = np.sqrt(float((av * av).sum() - fw * fw))
	t2 = np.arctan2(float(av[1]), float(av[0])) - np.arctan2(fw, d2)
	print 'd1 =', d1, 't1 =', t1, 'd2 =', d2, 't2 = ', t2

def main():
	ik_leftFinger_n()

if __name__ == '__main__':
	main()
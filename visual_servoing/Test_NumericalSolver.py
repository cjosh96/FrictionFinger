import numpy as np
import scipy.optimize as opt
from mpmath import *
# Geometric Parameters
wp = 6.2
w0 = 2.5
fw = 1.8

x = -2.2
y = 4.2

wp = 6.2
w0 = 2.5
fw = 1.8
contact_right = 0
contact_left = 0
# Vertices of the shape
X = np.array([w0/2, w0, w0/2, -w0/2, -w0, -w0])
Y = np.array([float(w0*sqrt(3)/2), 0, -float(w0*sqrt(3)/2), -float(w0*sqrt(3)/2), 0, float(w0*sqrt(3)/2)])
n = X.shape[0]
contact = 1

distances = np.array([])
x_d = np.array([])
y_d = np.array([])

gamma = np.array([])

for i in range(n-1):

	gamma = np.append(gamma, np.arctan2(Y[i+1]- Y[i], X[i+1]- X[i]))
	distances = np.append(distances, (X[i+1]*Y[i] - Y[i+1]*X[i])/sqrt((Y[i+1]-Y[i])**2 + (X[i+1]-X[i])**2))
	x_d = np.append(x_d, (Y[i]*X[i+1]-X[i]*Y[i+1])*(X[i+1]-X[i])*(Y[i+1]-Y[i])/((Y[i+1] - Y[i])**2 + (X[i+1]-X[i])**2))
	if(Y[i+1]-Y[i] != 0):
		y_d = np.append(y_d, (X[i+1] - X[i])*x_d[0]/(Y[i+1]-Y[i]))
	else:
		y_d = np.append(y_d, Y[i])

gamma = np.append(gamma, np.arctan2(Y[0]- Y[n-1], X[0]- X[n-1]))
distances = np.append(distances, (X[0]*Y[n-1] - Y[0]*X[n-1])/sqrt((Y[0]-Y[n-1])**2 + (X[0]-X[n-1])**2))
x_d = np.append(x_d, (Y[n-1]*X[0]-X[n-1]*Y[0])*(X[0]-X[n-1])*(Y[0]-Y[n-1])/((Y[0] - Y[n-1])**2 + (X[0]-X[n-1])**2))

if(Y[0]-Y[n-1] != 0):
	y_d = np.append(y_d, (X[0] - X[n-1])*x_d[0]/(Y[0]-Y[n-1]))
else:
	y_d = np.append(y_d, Y[0])



##### INVERSE KINEMATICS FOR SLIDING ALONG THE RIGHT FINGER #####
# def left_equations_d(variables):
# 	d2_sol = variables
# 	eqn3 = (x - wp)**2 + y**2 - (d2_sol + w0 / 2.)**2 - (fw + w0 / 2.)**2
# 	return (eqn3[0])


# def solve_d2():
# 	initial_guess_d2 = 5.8
# 	solution = opt.fsolve(left_equations_d, initial_guess_d2) 
# 	sold2 =  solution[0]

# 	d2 = sold2
# 	return d2



# def solve_left(variables):
# 	t2_sol = variables[0]
# 	d2 = solve_d2() 
# 	eqn1 = (d2 + w0 / 2.) * cos(t2_sol) - (fw + w0 / 2.) * sin(t2_sol) - (x - wp)
# 	return eqn1


# def ik_leftFinger_n():

# 	initial_guess_t1 = np.pi/3
# 	solution_t2 = opt.fsolve(solve_left, initial_guess_t1)
# 	print 't2 = ', solution_t2
# 	for i in range(1,5):
# 		initial_guess_t1 = np.pi * i / 5
# 		solution = opt.fsolve(solve_left, initial_guess_t1, full_output=True)
# 		# print 't2 = ', solution_t2[0]
# 		if solution[2]==1 and solution[0] > 0 and solution[0] < np.pi:
# 			solt2 = solution_t2[0]
# 			t2 = float(solt2)


# 	d2v = np.array([d2 * np.cos(np.float64(t2)), d2 * np.sin(np.float64(t2))])
# 	w0v = np.array([w0 * np.sin(np.float64(t2)), -w0 * np.cos(np.float64(t2))])
# 	wpv = np.array([wp, 0.])
# 	f1v = np.array([fw * np.sin(np.float64(t2)), -fw * np.cos(np.float64(t2))])
# 	av = d2v - f1v - w0v + wpv
# 	d1 = np.sqrt(float(abs((av * av).sum() - fw * fw)))
# 	t1 = np.arctan2(float(av[1]), float(av[0])) + np.arctan2(fw, d1)


##### INVERSE KINEMATICS FOR SLIDING ALONG THE RIGHT FINGER #####
##### INVERSE KINEMATICS FOR SLIDING ALONG THE RIGHT FINGER #####
def left_equations_d(variables):
	d2_sol = variables
	eqn3 = (x - wp)**2 + y**2 - (d2_sol + norm(X[contact_right] - x_d[contact_right], Y[contact_right] - y_d[contact_right]))**2 - (fw + distances[contact_right])**2
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
	eqn1 = (d2 +  norm(X[contact_right] - x_d[contact_right], Y[contact_right] - y_d[contact_right])) * cos(t2_sol) - (fw + distances[contact_right]) * sin(t2_sol) - (x - wp)
	return eqn1

def solve_t1_slide_left(variables, coords):
	solt2 = variables[0]
	eqn = coords[1] - coords[0]*tan(solt2) + fw*sec(solt2)
	return eqn

def ik_leftFinger_n():

	initial_guess_t1 = np.pi/3
	solution_t2 = opt.fsolve(solve_left, initial_guess_t1)

	for i in range(1,5):
		initial_guess_t1 = np.pi * i / 5
		solution = opt.fsolve(solve_left, initial_guess_t1, full_output=True)
		if solution[2]==1 and solution[0] > 0 and solution[0] < np.pi:
			solt2 = solution[0]
			t2 = float(solt2)

	R = np.array([[cos(-gamma[contact_right] + t2), -sin(-gamma[contact_right] + t2), x], [sin(-gamma[contact_right] + t2), cos(-gamma[contact_right] + t2), y], [0, 0, 1]])
	coords = np.dot(R, np.concatenate(([X], [Y], np.ones((1,n)))))
	
	beta = -9999
	contact_left = -1

	for i in range(n):
		initial_guess_t1 = np.pi/2	
		solution = opt.fsolve(solve_t1_slide_left, initial_guess_t1, args = coords[:,i], full_output=True)
		if (solution[2]==1 and solution[0] > 0 and solution[0] < np.pi and solution[0] > beta):
			beta = solution[0]
			contact_left = i
	t1 = beta[0]

	d1 = sqrt((coords[0, contact_left] - wp)**2 + coords[1, contact_left]**2 -fw**2)
	print d1, t1, t2


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

def solve_t2_slide_right(variables, coords):
	solt2 = variables[0]
	eqn = coords[1] - (coords[0] - wp)*tan(solt2) - fw*sec(solt2)
	return eqn

def right_equations_theta(variables):
	t1_sol = variables[0]
	d1  = solve_d1()
	eqn1 = (d1 + w0 / 2.) * cos(t1_sol) + (fw + w0 / 2.) * sin(t1_sol) - x
	return eqn1

def ik_rightFinger_n():

	initial_guess_t1 = np.pi/3
	solution_t1 = opt.fsolve(right_equations_theta, initial_guess_t1, xtol= 1e-5)
    
	for i in range(1,5):
		initial_guess_t1 = np.pi * i / 5
		solution = opt.fsolve(solve_left, initial_guess_t1, full_output=True)
		if (solution[2]==1 and solution[0] > 0 and solution[0]<np.pi):
			t1 = float(solution[0])
    		
	
	R = np.array([[cos(-gamma[contact_left] + t1), -sin(-gamma[contact_left] + t1), x], [sin(-gamma[contact_left] + t1), cos(-gamma[contact_left] + t1), y], [0, 0, 1]])
	coords = np.dot(R, np.concatenate(([X], [Y], np.ones((1,n)))))




	beta = 99999
	contact_right = -1
	
	for i in range(n):
		initial_guess_t2 = np.pi/2
		solution = opt.fsolve(solve_t2_slide_right, initial_guess_t2, args = coords[:,i], full_output=True)
		if (solution[2]==1 and solution[0] > 0 and solution[0] < np.pi and solution[0] < beta):
			beta = solution[0]
			contact_right = i
			# solt2 = solution_t2[0]
	
	t2 = beta[0]	
	d2 = sqrt((coords[0, contact_right] - wp)**2 + coords[1, contact_right]**2 - fw**2)
	print d2, t1, t2


def main():
	ik_leftFinger_n()

if __name__ == '__main__':
	main()
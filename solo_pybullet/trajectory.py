import numpy as np
import matplotlib.pyplot as plt
from scipy import interpolate
from math import pi

def jumpTrajectory(length, q_start):
	
	# define the different configurations of the jump
	pos_crouch = np.array([[0, pi/2, -pi], \
						[0, pi/2, -pi], \
						[0, -pi/2, pi], \
						[0, -pi/2, pi]])
	q_crouch = np.zeros(12)
	q_air = np.zeros(12)
	q_stand = q_start[7:].reshape(12)
	for leg in range(4):
			for art in range(3):
				q_crouch[3*leg+art] = 0.7*pos_crouch[leg, art]
				q_air[3*leg+art] = 0.4*pos_crouch[leg,art]
	
	# interpolation with a cubic spline
	x = np.array([0,0.3, 0.7, 1])
	y = np.zeros((12,4))
	y[:,0] = q_stand;
	y[:,1] = q_crouch;
	y[:,2] = q_stand;
	y[:,3] = q_air;
	
	xnew = np.arange(0, 1, 1/length)
	q_traj = np.zeros((12,len(xnew)))
	for art in range(12):
		tck = interpolate.splrep(x, y[art,:], k=3)
		ynew = interpolate.splev(xnew, tck, der=0)
		q_traj[art,:] = ynew
		
	# let's set the gain during the trajectory
	gains = np.zeros((2,len(xnew)))
	for i in range(len(xnew)):
		if xnew[i] < 0.3:
			gains[:,i] = np.array([1,20])
		if xnew[i] > 0.3 and xnew[i] < 0.7:
			gains[:,i] = np.array([0.3,30])
		if xnew[i] > 0.7:
			gains[:,i] = np.array([1,20])
	
	
	return q_traj, gains

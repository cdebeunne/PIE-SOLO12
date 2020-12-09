import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline
from scipy import interpolate
from math import pi

def jumpTrajectory(q_start, t_crouch, t_jump, t_air, dt):
	
	# define the different configurations of the jump
	pos_crouch = np.array([[0, pi/2, -pi], \
						[0, pi/2, -pi], \
						[0, -pi/2, pi], \
						[0, -pi/2, pi]])
	q_crouch = np.zeros(12)
	q_stand = q_start[7:].reshape(12)
	q_crouch = np.zeros(12)
	q_air = np.zeros(12)
	for leg in range(4):
			for art in range(3):
				q_crouch[3*leg+art] = 0.7*pos_crouch[leg, art]
				q_air[3*leg+art] = 0.4*pos_crouch[leg,art]
	
	# interpolation with a cubic spline
	
	# part 1 : a smooth trajectory for the crouching
	x = np.array([0, t_crouch])
	y = np.zeros((12,2))
	y[:,0] = q_stand
	y[:,1] = q_crouch
	
	xnew0 = np.arange(0, t_crouch, dt)
	traj0 = np.zeros((12,len(xnew0)))
	qtraj0 = np.zeros((12,len(xnew0)))
	for art in range(12):
		cs = CubicSpline(x, y[art,:], bc_type='clamped')
		traj0[art,:] = cs(xnew0)
		qtraj0[art,:] = cs(xnew0,1)
			
	# part 2 : the jump
	x = np.array([t_crouch,t_jump])
	y = np.zeros((12,2))
	y[:,0] = q_crouch
	y[:,1] = q_stand
	
	xnew1 = np.arange(t_crouch, t_jump, dt)
	traj1 = np.zeros((12,len(xnew1)))
	qtraj1 = np.zeros((12,len(xnew1)))

	for art in range(12):
		cs = CubicSpline(x, y[art,:], bc_type='clamped')
		traj1[art,:] = cs(xnew1)
		qtraj1[art,:] = cs(xnew1,1)
	
	# part 3 : reaching a good position for the reception
	x = np.array([t_jump, t_air])
	y = np.zeros((12,2))
	y[:,0] = q_stand
	y[:,1] = q_air
	
	xnew2 = np.arange(t_jump, t_air, dt)
	traj2 = np.zeros((12,len(xnew2)))
	qtraj2 = np.zeros((12,len(xnew2)))
	for art in range(12):
		cs = CubicSpline(x, y[art,:], bc_type='clamped')
		traj2[art,:] = cs(xnew2)
		qtraj2[art,:] = cs(xnew2,1)
		
	# part 4 : building the whole trajectory	
	xnew = np.concatenate((xnew0,xnew1, xnew2))
	q_traj = np.concatenate((traj0, traj1, traj2), axis = 1)
	qdot_traj = np.concatenate((qtraj0, qtraj1, qtraj2), axis = 1)
		
		
	# let's set the gain during the trajectory
	gains = np.zeros((2,len(xnew)))
	for i in range(len(xnew)):
		if xnew[i] < t_crouch:
			gains[:,i] = np.array([1,10])
		if xnew[i] > t_crouch and xnew[i] < t_jump:
			gains[:,i] = np.array([0.5,10])
		else:
			gains[:,i] = np.array([1,10])

	
	
	return q_traj, qdot_traj, gains

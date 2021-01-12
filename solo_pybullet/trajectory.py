import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline
from scipy import interpolate
from math import pi
from example_robot_data import loadSolo  # Functions to load the SOLO quadruped
import pinocchio as pin  # Pinocchio library

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

########################
## JUMPING_CONTROLLER ##
########################

# Method : Inverse Kinematics

# Initialization of the controller's parameters
q_ref = np.zeros((19, 1))
flag_q_ref = True


# function defining the feet's trajectory
def jump_traj(t, x0, y0, z0):  # arguments : time, initial position x, y and z
	T = 0.2  # period of the foot trajectory
	dx = 0.03  # displacement amplitude by x
	dz = 0.06  # displacement amplitude by z

	x = []
	z = []
	y = []
	if t >= T:
		t %= T
	
	x.append(x0 * 1.1)

	y.append(y0)

	if t <= T / 2.:
		z.append(z0 + dz * np.sin(2 * np.pi * t / T))
	else:
		z.append(z0)

	return np.array([x[-1], y[-1], z[-1]])


def kinInv_3D(q, qdot, dt, solo, t_simu, ftraj, display=True):
	from numpy.linalg import pinv
	global q_ref, flag_q_ref

	K = 100.  # Convergence gain

	# unactuated, [x, y, z] position of the base + [x, y, z, w] orientation of the base (stored as a quaternion)
	# qu = q[:7]
	# [v_x, v_y, v_z] linear velocity of the base and [w_x, w_y, w_z] angular velocity of the base along x, y, z axes
	# of the world
	# qu_dot = qdot[:6]

	qa_dot_ref = np.zeros((12, 1))  # target angular velocities for the motors

	if flag_q_ref:
		q_ref = solo.q0.copy()
		flag_q_ref = False

	# Initialization of the variables
	xF0 = 0.19  # initial position of the front feet
	xH0 = -0.19  # initial position of the hind feet
	yL0 = 0.147  # Initial position of the left feet
	yR0 = -0.147 # Initial position of the right feet
	z0 = 0.01  # initial altitude of each foot

	# Compute/update all the joints and frames
	pin.forwardKinematics(solo.model, solo.data, q_ref)
	pin.updateFramePlacements(solo.model, solo.data)

	# Get the frame index of each foot
	ID_FL = solo.model.getFrameId("FL_FOOT")
	ID_FR = solo.model.getFrameId("FR_FOOT")
	ID_HL = solo.model.getFrameId("HL_FOOT")
	ID_HR = solo.model.getFrameId("HR_FOOT")

	# Get the current position of the feet
	xyz_FL = solo.data.oMf[ID_FL].translation
	xyz_FR = solo.data.oMf[ID_FR].translation
	xyz_HL = solo.data.oMf[ID_HL].translation
	xyz_HR = solo.data.oMf[ID_HR].translation

	# Desired foot trajectory
	xyzdes_FL = ftraj(t_simu, xF0, yL0, z0)
	xyzdes_HR = ftraj(t_simu, xH0, yR0, z0)
	xyzdes_FR = ftraj(t_simu, xF0, yR0, z0)
	xyzdes_HL = ftraj(t_simu, xH0, yL0, z0)

	# Calculating the error
	err_FL = (xyz_FL - xyzdes_FL)
	err_FR = (xyz_FR - xyzdes_FR)
	err_HL = (xyz_HL - xyzdes_HL)
	err_HR = (xyz_HR - xyzdes_HR)

	# Computing the local Jacobian into the global frame
	oR_FL = solo.data.oMf[ID_FL].rotation
	oR_FR = solo.data.oMf[ID_FR].rotation
	oR_HL = solo.data.oMf[ID_HL].rotation
	oR_HR = solo.data.oMf[ID_HR].rotation

	# Getting the different Jacobians
	fJ_FL3 = pin.computeFrameJacobian(solo.model, solo.data, q_ref, ID_FL)[:3, -12:]  # Take only the translation terms
	oJ_FL3 = oR_FL.dot(fJ_FL3)  # Transformation from local frame to world frame
	oJ_FLxyz = oJ_FL3[0:3, -12:]  # Take the x,y & z components

	fJ_FR3 = pin.computeFrameJacobian(solo.model, solo.data, q_ref, ID_FR)[:3, -12:]
	oJ_FR3 = oR_FR.dot(fJ_FR3)
	oJ_FRxyz = oJ_FR3[0:3, -12:]

	fJ_HL3 = pin.computeFrameJacobian(solo.model, solo.data, q_ref, ID_HL)[:3, -12:]
	oJ_HL3 = oR_HL.dot(fJ_HL3)
	oJ_HLxyz = oJ_HL3[0:3, -12:]

	fJ_HR3 = pin.computeFrameJacobian(solo.model, solo.data, q_ref, ID_HR)[:3, -12:]
	oJ_HR3 = oR_HR.dot(fJ_HR3)
	oJ_HRxyz = oJ_HR3[0:3, -12:]

	# Displacement error
	nu = np.hstack([err_FL, err_FR, err_HL, err_HR])
	nu = np.matrix(nu)
	nu = np.transpose(nu)

	# Making a single x&y&z-rows Jacobian vector
	J = np.vstack([oJ_FLxyz, oJ_FRxyz, oJ_HLxyz, oJ_HRxyz])

	# Computing the velocity
	qa_dot_ref = -K * pinv(J) * nu
	q_dot_ref = np.concatenate((np.zeros([6, 1]), qa_dot_ref))

	# Computing the updated configuration
	q_ref = pin.integrate(solo.model, q_ref, q_dot_ref * dt)
	qa_ref = q_ref[7:].reshape(12,1)
  
	if display:
		# DONT FORGET TO RUN GEPETTO-GUI BEFORE RUNNING THIS PROGRAMM #
		solo.display(q_ref)  # display the robot in the viewer Gepetto-GUI given its configuration q

	# Return configuration of the robot
	return q_ref, q_dot_ref

def pause():
    programPause = input("Press the <ENTER> key to continue...")

if __name__ == '__main__':
	solo = loadSolo(False)
	solo.initViewer(loadModel=True)

	dt = 0.0001

	solo.q0[2] = 0.32
	q = solo.q0.copy()
	q_dot = np.zeros((solo.model.nv, 1))

	for t in np.arange(0, 4*0.2, dt):
		q, q_dot  = kinInv_3D(q, q_dot, dt, solo, t, jump_traj)
		# pause()
		


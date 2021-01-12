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
## WALKING_CONTROLLER ##
########################

# Method : Inverse Kinematics

# Initialization of the controller's parameters
q_ref = np.zeros((19, 1))
flag_q_ref = True

def c_walking_IK(q, qdot, dt, solo, t_simu):
	# unactuated, [x, y, z] position of the base + [x, y, z, w] orientation of the base (stored as a quaternion)
	# qu = q[:7]
	qa = q[7:]  # actuated, [q1, q2, ..., q8] angular position of the 8 motors
	# [v_x, v_y, v_z] linear velocity of the base and [w_x, w_y, w_z] angular velocity of the base along x, y, z axes
	# of the world
	# qu_dot = qdot[:6]
	qa_dot = qdot[6:]  # angular velocity of the 8 motors

	qa_ref = np.zeros((12, 1))  # target angular positions for the motors
	qa_dot_ref = np.zeros((12, 1))  # target angular velocities for the motors

	###############################################
	# Insert code here to set qa_ref and qa_dot_ref

	from numpy.linalg import pinv

	global q_ref, flag_q_ref, T, dx, dz

	if flag_q_ref:
		q_ref = solo.q0.copy()
		flag_q_ref = False

	# Initialization of the variables
	K = 100.  # Convergence gain
	T = 0.2  # period of the foot trajectory
	xF0 = 0.19  # initial position of the front feet
	xH0 = -0.19  # initial position of the hind feet
	z0 = 0  # initial altitude of each foot
	dx = 0.06  # displacement amplitude by x
	dz = 0.03  # displacement amplitude by z

	# Get the frame index of each foot
	ID_FL = solo.model.getFrameId("FL_FOOT")
	ID_FR = solo.model.getFrameId("FR_FOOT")
	ID_HL = solo.model.getFrameId("HL_FOOT")
	ID_HR = solo.model.getFrameId("HR_FOOT")

	# function defining the feet's trajectory
	def ftraj(t, x0, z0):  # arguments : time, initial position x and z
		global T, dx, dz
		x = []
		z = []
		if t >= T:
			t %= T
		x.append(x0 - dx * np.cos(2 * np.pi * t / T))
		if t <= T / 2.:
			z.append(z0 + dz * np.sin(2 * np.pi * t / T))
		else:
			z.append(0)
		return np.array([x[-1], z[-1]])

	# Compute/update all the joints and frames
	pin.forwardKinematics(solo.model, solo.data, q_ref)
	pin.updateFramePlacements(solo.model, solo.data)

	# Get the current height (on axis z) and the x-coordinate of the front left foot
	xz_FL = solo.data.oMf[ID_FL].translation[0::2]
	xz_FR = solo.data.oMf[ID_FR].translation[0::2]
	xz_HL = solo.data.oMf[ID_HL].translation[0::2]
	xz_HR = solo.data.oMf[ID_HR].translation[0::2]

	# Desired foot trajectory
	xzdes_FL = ftraj(t_simu, xF0, z0)
	xzdes_HR = ftraj(t_simu, xH0, z0)
	xzdes_FR = ftraj(t_simu + T / 2, xF0, z0)
	xzdes_HL = ftraj(t_simu + T / 2, xH0, z0)

	# Calculating the error
	err_FL = (xz_FL - xzdes_FL)
	err_FR = (xz_FR - xzdes_FR)
	err_HL = (xz_HL - xzdes_HL)
	err_HR = (xz_HR - xzdes_HR)

	# Computing the local Jacobian into the global frame
	oR_FL = solo.data.oMf[ID_FL].rotation
	oR_FR = solo.data.oMf[ID_FR].rotation
	oR_HL = solo.data.oMf[ID_HL].rotation
	oR_HR = solo.data.oMf[ID_HR].rotation

	# Getting the different Jacobians
	fJ_FL3 = pin.computeFrameJacobian(solo.model, solo.data, q_ref, ID_FL)[:3, -12:]  # Take only the translation terms
	oJ_FL3 = oR_FL.dot(fJ_FL3)  # Transformation from local frame to world frame
	oJ_FLxz = oJ_FL3[0::2, -12:]  # Take the x and z components

	fJ_FR3 = pin.computeFrameJacobian(solo.model, solo.data, q_ref, ID_FR)[:3, -12:]
	oJ_FR3 = oR_FR.dot(fJ_FR3)
	oJ_FRxz = oJ_FR3[0::2, -12:]

	fJ_HL3 = pin.computeFrameJacobian(solo.model, solo.data, q_ref, ID_HL)[:3, -12:]
	oJ_HL3 = oR_HL.dot(fJ_HL3)
	oJ_HLxz = oJ_HL3[0::2, -12:]

	fJ_HR3 = pin.computeFrameJacobian(solo.model, solo.data, q_ref, ID_HR)[:3, -12:]
	oJ_HR3 = oR_HR.dot(fJ_HR3)
	oJ_HRxz = oJ_HR3[0::2, -12:]

	# Displacement error
	nu = np.hstack([err_FL, err_FR, err_HL, err_HR])
	nu = np.matrix(nu)
	nu = np.transpose(nu)

	# Making a single x&z-rows Jacobian vector
	J = np.vstack([oJ_FLxz, oJ_FRxz, oJ_HLxz, oJ_HRxz])

	# Computing the velocity
	qa_dot_ref = -K * pinv(J) * nu
	q_dot_ref = np.concatenate((np.zeros([6, 1]), qa_dot_ref))

	# Computing the updated configuration
	q_ref = pin.integrate(solo.model, q_ref, q_dot_ref * dt)
	qa_ref = q_ref[7:].reshape(12,1)
  

	# DONT FORGET TO RUN GEPETTO-GUI BEFORE RUNNING THIS PROGRAMM #
	solo.display(q_ref)  # display the robot in the viewer Gepetto-GUI given its configuration q

	# torques must be a numpy array of shape (8, 1) containing the torques applied to the 8 motors
	return q_ref, q_dot_ref

def pause():
    programPause = input("Press the <ENTER> key to continue...")

if __name__ == '__main__':
	solo = loadSolo(False)
	solo.initViewer(loadModel=True)

	dt = 0.0001
	q_start = np.zeros((19,1))
	t_crouch = 1.2
	t_jump = 1.7
	t_air = 3

	# qTraj, qdotTraj, gains = jumpTrajectory(q_start, t_crouch, t_jump, t_air, dt)
	# x = np.arange(0,t_air, dt)
	# plt.figure()
	# plt.plot(x, qTraj[1,:], 'g', x, qTraj[2,:], 'b')
	# plt.legend(['Shoulder', 'Knee'])
	# plt.axis([0, t_air, -5, 5])
	# plt.title('Cubic-spline trajectory of a front leg')
	# plt.show()
	solo.q0[2] = 0.32
	q = solo.q0.copy()
	q_dot = np.zeros((solo.model.nv, 1))

	for t in np.arange(0, 4*0.2, dt):
		q, q_dot  = c_walking_IK(q, q_dot, dt, solo, t)
		# pause()
		


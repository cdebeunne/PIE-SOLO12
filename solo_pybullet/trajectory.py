import numpy as np
from scipy.interpolate import CubicSpline
from scipy import interpolate
from math import pi
from example_robot_data import loadSolo  # Functions to load the SOLO quadruped
import pinocchio as pin  # Pinocchio library

def jumpTrajectory_1(q_start, t_crouch, t_jump, t_air, dt):
	t_traj = np.arange(0, t_crouch+t_jump+t_air, dt)

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

	# Swapping axis to match correct shape
	q_traj = np.swapaxes(q_traj, 0, 1)
	qdot_traj = np.swapaxes(qdot_traj, 0, 1)
	gains = np.swapaxes(gains, 0, 1)
	
	return t_traj, q_traj, qdot_traj, gains


def jumpTrajectory_2(**kwargs):
	import time
	# params: tf, dt, kp, kd, kps, kds, debug, init_reversed, max_error
	t_traj = np.arange(0, kwargs.get("tf", 10), kwargs.get("dt", .001))
	q_traj = []
	qdot_traj = []
	gains = []
	
	t0 = time.time()
	
	if kwargs.get("debug", False):
		print("Computing Trajectory...")

		for key, value in kwargs.items():
			print("\t", key, ":", value)

	solo = loadSolo(False)

	# Place the robot in a regular configuration
	solo.q0[2] = 0.
	for i in range(4):
		sign = 1 if kwargs.get("init_reversed", False) else -1
		
		if i<2:
			solo.q0[7+3*i+1] = +sign*pi/4 
			solo.q0[7+3*i+2] = -sign*pi/2
		else:
			solo.q0[7+3*i+1] = -sign*pi/4
			solo.q0[7+3*i+2] = +sign*pi/2

	q = solo.q0.copy()
	q_dot = np.zeros((solo.model.nv, 1))

	# Reach the initial position first
	step = 0
	while True:
		q, q_dot, gain, err  = kinInv_3D(q, q_dot, solo, 0, trajFeet_jump1, **kwargs)
		if err<10**(-10):
			break

		step += 1

		if step>1000:
			print("Unable to reach the first position.")
			return q_traj, qdot_traj, gains
	
	# Run the trajectory definition
	flag_errTooHigh = False
	maxE = 0
	for t in t_traj:
		q, q_dot, gain, err = kinInv_3D(q, q_dot, solo, t, trajFeet_jump1, **kwargs)
		
		q_traj.append(q)
		qdot_traj.append(q_dot)
		gains.append(gain)

		if err>kwargs.get("max_error", 0.5):
			flag_errTooHigh = True
			if err>maxE:
				maxE = err

	# If a position wasn't reachead, print it
	if flag_errTooHigh:
		print("The error was pretty high. Maybe the trajectory is trop ambitious (maxE=",maxE, ")", sep='')
	
	# Convert to numpy arrays
	q_traj = np.array(q_traj)
	qdot_traj = np.array(qdot_traj)
	gains = np.array(gains)

	#Initialize angles at zero
	offsets = np.zeros(q_traj.shape[1])
	for i in range(q_traj.shape[1]):
		while abs(q_traj[0][i]-offsets[i]*2*pi) > pi:
			if q_traj[0][i]>0:
				offsets[i] += 1
			else:
				offsets[i] -= 1
	
	for i in range(q_traj.shape[0]):
		q_traj[i] = q_traj[i]-2*pi*offsets
	
	# Print execution time if requiered
	if kwargs.get("debug", False):
		print("Done in", time.time()-t0, "s")

	return t_traj, q_traj, qdot_traj, gains
		

# function defining the feet's trajectory
def trajFeet_jump1(t, footId,  **kwargs):
	t0 = kwargs.get("traj_t0", 1)  # Duration of first step (extension of legs)
	t1 = kwargs.get("traj_t1", 1)  # Duration of second step (spreading legs)

	dz = kwargs.get("traj_dz", 0.25)  # displacement amplitude by z
	dy = kwargs.get("traj_dy", 0.05)  # displacement amplitude by y
	dx = kwargs.get("traj_dx", dy)    # displacement amplitude by x

	# Gains parameters
	param_kp = kwargs.get("kp", 5)	# default parameter of kp
	param_kd = kwargs.get("kd", 1)	# default parameter of kd
	param_kps = kwargs.get("kps", [param_kp, param_kp])	# default parameter of kps
	param_kds = kwargs.get("kds", [param_kp, param_kd])	# default parameter of kds

	# Initialization of the variables
	traj_x0 = 0.190 + kwargs.get("traj_dx0", 0) # initial distance on the x axis from strait
	traj_y0 = 0.147 + kwargs.get("traj_dy0", 0) # initial distance on the y axis from strait
	traj_z0 = kwargs.get("traj_z0", -0.05) # initial distance on the z axis from body
	traj_zf = kwargs.get("traj_zf", -0.2) # initial distance on the z axis from body

	if traj_z0>=0 or traj_zf>=0:
		print("traj_z0 or traj_zf might be positive. This may lead to invalid configuration.")

	x0 = -traj_x0 if int(footId/2)%2 else traj_x0
	y0 = -traj_y0 if footId%2 else traj_y0

	x, y, z = 0, 0, 0
	gains = [0, 0]
	x = x0*1.1

	# If time is overlapping the duration of the sequence, stay in last position
	if t>t0+t1:
		t=t0+t1

	# First part of the jump, push while staying at same position
	if t < t0:
		z = traj_z0-dz*np.sin(np.pi/2 * t / t0)

		x = x0
		y = y0

		gains[0] = param_kps[0]
		gains[1] = param_kds[0]
	# Second part of the jump, exand feets and retract legs
	elif t <= t0+t1:
		t = t-t0

		if x0>0:
			x = x0 + dx * np.sin(np.pi/2 * t/t1)
		else:
			x = x0 - dx * np.sin(np.pi/2 * t/t1)

		if y0>0:
			y = y0 + dy * np.sin(np.pi/2 * t/t1)
		else:
			y = y0 - dy * np.sin(np.pi/2 * t/t1)
		
		z = traj_zf + (-traj_zf+traj_z0-dz)*np.sin(np.pi/2 *(1 - t/t1))

		gains[0] = param_kps[1]
		gains[1] = param_kds[1]

	return np.array([x, y, z]), gains



# Initialization of the controller's parameters
q_ref = np.zeros((19, 1))
flag_q_ref = True

def kinInv_3D(q, qdot, solo, t_simu, ftraj, **kwargs):
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
	xyzdes_FL = ftraj(t_simu, 0, **kwargs)[0]
	xyzdes_FR = ftraj(t_simu, 1, **kwargs)[0]
	xyzdes_HL = ftraj(t_simu, 2, **kwargs)[0]
	xyzdes_HR = ftraj(t_simu, 3, **kwargs)[0]

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
	q_ref = pin.integrate(solo.model, q_ref, q_dot_ref * kwargs['dt'])
	qa_ref = q_ref[7:].reshape(12,1)

	# Return configuration of the robot
	gains = ftraj(t_simu, 0, **kwargs)[1]
	q_dot_ref = np.squeeze(np.array(q_dot_ref))
	err = np.linalg.norm(nu)

	return q_ref, q_dot_ref, gains, err

def TSID_traj(verticalSpeed, disp):
    if (disp):
        solo12 = loadSolo(False)
        solo12.initViewer(loadModel=True)
        
	# initialize TSID

    tsid = solo_tsid.solo_tsid()
    tsid.setCOMRef(np.array([0.0,0.0,-0.05]).T, np.zeros(3), np.zeros(3))
    tsid.setBaseRef()

    comObj1 = tsid.getCOM()+np.array([0.0,0.0,-0.05]).T
    comObj2 = tsid.getCOM()+np.array([0.0,0.0,+0.09]).T

	# initalize trajectories

    N_simu = 1500
    tau    = np.full((tsid.solo12_wrapper.na, N_simu), np.nan)
    q      = np.full((tsid.solo12_wrapper.nq, N_simu + 1), np.nan)
    v      = np.full((tsid.solo12_wrapper.nv, N_simu + 1), np.nan)
    dv     = np.full((tsid.solo12_wrapper.nv, N_simu + 1), np.nan)

	# launch simu

    t = 0.0
    dt = 1e-3
    q[:, 0], v[:, 0] = tsid.q0, tsid.v0

    for i in range(N_simu-2):
        time_start = time.time()
		
        HQPData = tsid.formulation.computeProblemData(t, q[:,i], v[:,i])
        sol = tsid.solver.solve(HQPData)
		
        com = tsid.getCOM()
        deltaCom1 = abs(com[2] - comObj1[2])
        deltaCom2 = abs(com[2] - comObj2[2])
        if deltaCom1 < 2e-2:
            tsid.setCOMRef(np.array([0.0,0.0,0.1]).T, np.array([0.0,0.0,+0.1]), np.zeros(3))
        else:
            print(deltaCom1)
		
        if deltaCom2 < 1e-2:
            break
		
        if sol.status != 0:
            print("Time %.3f QP problem could not be solved! Error code:" % t, sol.status)
            break
		
        tau[:,i] = tsid.formulation.getActuatorForces(sol)
        dv[:,i] = tsid.formulation.getAccelerations(sol)
		
		# numerical integration
        q[:,i + 1], v[:,i + 1] = tsid.integrate_dv(q[:,i], v[:,i], dv[:,i], dt)
        t += dt
		
        if (disp):
            solo12.display(q[:,i])
            time.sleep(1e-3)
    
    # we add the last configuration 
    q[:, i+2], v[:, i+2] = tsid.q0, tsid.v0
			
    return q[:,0:i+2], v[:,0:i+2], dv[:, 0:i+2], tau[:, 0:i+2]


if __name__ == '__main__':
	import time

	dt = 0.01
	t = np.arange(0, 10, dt)
	
	q_traj = jumpTrajectory_2(init_reversed=True, tf=2, dt=dt, debug=True)[1]

	solo = loadSolo(False)
	solo.initViewer(loadModel=True)

	for i in range(min(len(t), len(q_traj))):
		t0 = time.time()
		solo.display(q_traj[i])

		while(time.time()-t0<dt):
			time.sleep(0.00001)
	
		


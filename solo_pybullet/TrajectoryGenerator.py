import numpy as np
from math import pi
from example_robot_data import loadSolo  # Functions to load the SOLO quadruped
import pinocchio as pin  # Pinocchio library

"""
Actuators Trajectory class.
Defines all the elements that are needed to define a trajectory.
"""
class ActuatorsTrajectory:
	def __init__(self, nbActuators=12):
		self.trajectoryElements = {"size": 0}
		self.nbActuators = nbActuators

		# Default entries names
		self.entriesNames = ["t", "q", "q_dot", "torques", "gains"]

		# Types for the entries.
		arrayType = type(np.array(0))
		self.entriesTypes = {"t":arrayType, "q":arrayType, "q_dot":arrayType, "torques":arrayType, "gains":arrayType}

		# Shapes of the entries.
		# If the size if -1, it must be size. Indifferent if size is -2.
		self.entriesShapes = {"t":[-1], "q":[self.nbActuators, -1], "q_dot":[self.nbActuators, -1], "torques":[self.nbActuators, -1], "gains":[2, -1]}

		# Default values of the entries.
		# If a default value is not defined, the trajectory will generate an error.
		self.defaultElements = {"q":np.zeros(self.nbActuators), "q_dot":np.zeros(self.nbActuators), "torques":np.zeros(self.nbActuators), "gains":np.array([5, 1])}
	
		self.printWarnings = False

	"""
	Print information in the console if printWarning is true
	"""
	def printWarn(self, text):
		if not self.printWarnings:
			return
		print(text)

	"""
	Print information in the console if printWarning is true
	"""
	def printExcept(self, text):
		print(text)
		
	"""
	Returns the size of the trajectory (number of elements).
	"""
	def getSize(self):
		return self.trajectoryElements['size']

	"""
	Prints the informations that are stored in the trajectory.
	"""
	def printInfo(self):
		print("Trajectory Class:")
		for key, value in self.trajectoryElements.items():
			if type(value) is int or type(value) is float:
				print("\t{0}:\t{1}".format(key, value))
			else:
				print("\t{0}:\t{1}".format(key, type(value)))

	"""
	Add an element to a trajectory.
	"""
	def addElement(self, entry, data):
		# Check if data type exists
		if entry in self.entriesTypes:
			if type(data) is not self.entriesTypes[entry]:
				self.printExcept("Wrong type for data of {0} ({1}, {2}).".format(entry, type(data), self.entriesTypes[entry]))
				return False
		else:
			return False

		# Check for size compatibility
		if entry in self.entriesShapes:
			shapes = self.entriesShapes[entry]
			
			for i, s in enumerate(shapes):
				valid = True

				if s==-1 : # Size must be equal to trajectory size
					if self.trajectoryElements['size']<=0:
						self.trajectoryElements['size'] = data.shape[i]
					else:
						valid = self.trajectoryElements['size']==data.shape[i]
				elif s==-2: # No constraint on size
					valid = True
				else:
					valid = s==data.shape[i]

				if not valid:
					self.printExcept("Invalid size for entry {0} on axis {1} ({2}, {3})".format(entry, i, data.shape[i], self.trajectoryElements['size'] if s==-1 else s))
					return False
		
		# Add the data to the trajectory
		self.trajectoryElements[entry] = data

		return True

	"""
	Get an element of the trajectory at the given index.
	"""
	def getElement(self, entry, index):
		if not self.containsElement(entry):
			self.printWarn("The entry {0} does not exist.".format(entry))
			if entry in self.defaultElements:
				self.printWarn("Returning default value.")
				return self.defaultElements[entry]
			else:
				self.printExcept("Can't find any default value for entry {0}".format(entry))
				return None

		if type(self.trajectoryElements[entry])==type(np.array(0)):
			if index > self.trajectoryElements['size']:
				self.printExcept("Index for entry {0} exeeds size ({1}, {2})".format(entry, index, self.trajectoryElements['size']))
				return None
			
			if len(self.trajectoryElements[entry].shape)==1:
				return self.trajectoryElements[entry][index]
			elif len(self.trajectoryElements[entry].shape)==2:
				return self.trajectoryElements[entry][:, index]
			else:
				self.printExcept("Unknown size of element in trajectory element")
				return None
		else:
			return self.trajectoryElements[entry]

	"""
	Get an element of the trajectory at the given time.
	It will return the first value after the given time. 
	"""
	def getElementAtTime(self, entry, time):
		if not self.containsElement('t'):
			self.printExcept("Time is not defined yet in the trajectory (entry \'t\').")
			return None

		index = 0
		if time>np.max(self.trajectoryElements['t']):
			index = -1
		else:
			index = np.argmax(self.trajectoryElements['t']>=time)
			
		return self.getElement(entry, index)

	"""
	Check if an entry exists in the trajectory.
	"""
	def containsElement(self, entry):
		return entry in self.trajectoryElements

	"""
	Sets the default value of an entry.
	The default value must had been defined before.
	"""
	def setDefaultElement(self, entry, data):
		if not entry in self.defaultElements:
			self.printExcept("Can't set a default value for this entry.")
			return False
		
		# Check for type if there is any
		if entry in self.entriesTypes:
			if type(data) != self.entriesTypes[entry]:
				self.printExcept("Type of default data does not match: {0} {1}".format(type(data), self.entriesTypes[entry]))
				return False
		
		# Check for shape if array
		if type(data)==type(np.array(0)) and data.shape!=self.defaultElements[entry].shape:
			self.printExcept("Shape of default data does not match: {0} {1}".format(data.shape, self.defaultElements[entry].shape))
			return False
		
		self.defaultElements[entry] = data
		
		return True

	"""
	Returns the default value of an entry.
	"""
	def getDefaultElement(self, entry):
		if entry in self.defaultElements:
			return self.defaultElements[entry]
		else:
			self.printExcept("There isn't any default value for entry {0}.".format(entry))
			return None

	"""
	Plots the trajectory.
	"""
	def plotTrajectory(self, show_gains=False, show_all=True):
		if self.getSize()<=0:
			self.printExcept("No data to plot.")
			return
		
		import matplotlib.pyplot as plt
		
		t = self.trajectoryElements['t']
		qa = self.trajectoryElements.get('q')
		qa_dot = self.trajectoryElements.get('q_dot')
		torques = self.trajectoryElements.get('torques')
		gains = self.trajectoryElements.get('gains')
		titles = ["Heap", "Shoulder", "Knee"]

		nb_rows = 4 if show_gains and gains is not None else 3
		nb_cols = 1
		nb_cols += 1 if qa_dot is not None else 0
		nb_cols += 1 if torques is not None else 0
			
		fig, axes = plt.subplots(nrows=nb_rows, ncols=nb_cols, sharex=True, constrained_layout=True)

		fig.suptitle("Actuators Trajectory", fontsize=16)

		for i in range(3):
			# Position
			axes[i, 0].set_title(titles[i]+" Position")
			for j in range(4 if show_all else 1):
				axes[i, 0].plot(t, qa[i+3*j,:], label=("Leg {0}".format(j+1) if show_all else 'q (rad)'))

			# Speed
			if qa_dot is not None:
				axes[i, 1].set_title(titles[i]+" Speed")
				for j in range(4 if show_all else 1):
					axes[i, 1].plot(t, qa_dot[i+3*j,:], label=(None if show_all else 'q_dot (rad/s)'))

			# FeedForward
			if torques is not None:
				torq_ax = 2 if nb_cols==3 else 1
				axes[i, torq_ax].set_title(titles[i]+" Torque")
				for j in range(4 if show_all else 1):
					axes[i, torq_ax].plot(t, torques[i+3*j,:], label=(None if show_all else 'feedforward (N.m)'))

			for j in range(nb_cols):
				axes[i, j].legend()
				axes[i, j].grid()
				axes[i, j].set_xlabel("Time (s)")
				axes[i, j].set_ylabel("Values")
		
		if show_gains and gains is not None:
			gs = axes[-1, 0].get_gridspec()
			for ax in axes[-1, :]:
				ax.remove()
			axbig = fig.add_subplot(gs[-1, :])

			axbig.set_title("Gains")
			axbig.plot(t, gains[0,:], label='Kp')
			axbig.plot(t, gains[1,:], label='Kd')
			axbig.legend()
			axbig.grid()
			axbig.set_xlabel("Time (s)")
			axbig.set_ylabel("Gains")
		
		plt.show()

"""
Trajectory Generator class.
Class to define a template of trajectory generator.
A trajectory generator must return an ActuatorsTrajectory class.
"""
class TrajectoryGenerator:
	def __init__(self):
		self.name = "Default."
		self.parametersDefaults = {}
		self.parameters = {}

	"""
	Prints info about the trajectory generator (name, parameters, ...)
	"""
	def printInfo(self):
		print("Trajectory Generator:")
		print("\t-name: ", self.name)
		print("\t-parameters:")
		if len(self.parametersDefaults)!=0:
			for key, value in self.parametersDefaults.items():
				if key in self.parameters:
					print("\t\t- {0} : {1} (def: {2})".format(key, self.parameters[key], value))
				else:
					print("\t\t- {0} : {1}".format(key, value))
		else:
			print("\t\tNone.")

	"""
	Returns allowed names of the parameters.
	"""
	def getParametersNames(self):
		return list(self.parametersDefaults.keys())

	"""
	Set the value of a parameter.
	"""
	def setParameter(self, entry, value):
		if not entry in self.getParametersNames():
			print("Can\'t set parameter with name {0}. Please check getParametersNames.".format(entry))
			return False
		
		self.parameters[entry] = value
		return True
	
	"""
	Set the value of parameters using a dictionnary.
	"""
	def setParametersFromDict(self, **kwargs):
		success = True
		for key, value in kwargs.items():
			success = False if not self.setParameter(key, value) else success
		return success

	"""
	Returns the value of a parameter.
	"""
	def getParameter(self, entry):
		if not entry in self.getParametersNames():
			print("Can\'t get parameter with name {0}. Please check getParametersNames.".format(entry))
			return None
		
		if entry in self.parameters:
			return self.parameters[entry]
			
		return self.parametersDefaults[entry]

	"""
	This function should return the trajectory generated using
	the kwargs parameters.
	Values returned must be an ActuatorsTrajectory class.
	"""
	def generateTrajectory(self, **kwargs):
		self.setParametersFromDict(**kwargs)

		return ActuatorsTrajectory(12)
	

"""
Trajectory Generator using spline for configuration of the actuators.
"""
class TrajectoryGen_Splines(TrajectoryGenerator):
	def __init__(self):
		TrajectoryGenerator.__init__(self)

		self.name = "Splines Trajectory"

		# Defining default parameters of the trajectory
		self.parametersDefaults['q_start'] = np.zeros(12)
		self.parametersDefaults['t_crouch'] = 1
		self.parametersDefaults['t_jump'] = 2
		self.parametersDefaults['t_air'] = 3
		self.parametersDefaults['dt'] = 0.05
	
	def generateTrajectory(self, **kwargs):
		from scipy.interpolate import CubicSpline

		# Load parameters of the trajectory
		self.setParametersFromDict(**kwargs)
		t_crouch = self.getParameter('t_crouch')
		t_jump = self.getParameter('t_jump')
		t_air = self.getParameter('t_air')
		dt = self.getParameter('dt')

		# Define trajectory for return
		traj = ActuatorsTrajectory()

		# Define time of the trajectory
		t_traj = np.arange(0, t_air, dt)

		# Define the different configurations of the jump
		pos_crouch = np.array([[0, pi/2, -pi], \
							[0, pi/2, -pi], \
							[0, -pi/2, pi], \
							[0, -pi/2, pi]])
							
		q_stand = self.getParameter('q_start')
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

		# Define trajectory for return
		traj = ActuatorsTrajectory()
		traj.addElement('t', t_traj)
		traj.addElement('q', q_traj)
		traj.addElement('q_dot', qdot_traj)
		traj.addElement('gains', gains)
		
		return traj


"""
Trajectory Generator using inverse kinematics on feet position.
"""
class TrajectoryGen_InvKin(TrajectoryGenerator):
	def __init__(self):
		TrajectoryGenerator.__init__(self)

		self.name = "Inverse Kinematics Trajectory"

		# Defining default parameters of the trajectory
		self.parametersDefaults['tf'] = 10
		self.parametersDefaults['dt'] = 0.001
		self.parametersDefaults['debug'] = False
		self.parametersDefaults['init_reversed'] = False
		self.parametersDefaults['max_init_error'] = 10**(-10)
		self.parametersDefaults['max_init_iterations'] = 1000
		self.parametersDefaults['max_kinv_error'] = 0.5
		self.parametersDefaults['kinv_gain'] = 100
		self.parametersDefaults['feet_traj_func'] = self.trajFeet_jump1
		self.parametersDefaults['feet_traj_params'] = {}

		# Parameters for Inverse Kinematics
		self.q_ref = np.zeros((19, 1))
		self.flag_q_ref = True
	
	def generateTrajectory(self, **kwargs):
		import time

		# Load parameters of the trajectory
		self.setParametersFromDict(**kwargs)

		# params: tf, dt, kp, kd, kps, kds, debug, init_reversed, max_error
		t_traj = np.arange(0, self.getParameter('tf'), self.getParameter('dt'))
		q_traj = []
		qdot_traj = []
		gains = []
		
		t0 = time.time()
		
		if self.getParameter('debug'):
			print("Computing Trajectory...")
			self.printInfo()

		solo = loadSolo(False)

		# Place the robot in a regular configuration
		solo.q0[2] = 0.
		for i in range(4):
			sign = 1 if self.getParameter('init_reversed') else -1
			
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
			q, q_dot, gain, err = self.kinInv_3D(q, q_dot, solo, 0)
			if err<self.getParameter('max_init_error'):
				break

			step += 1

			if step>self.getParameter('max_init_iterations'):
				print("Unable to reach the first position.")
				return None
		
		# Run the trajectory definition
		flag_errTooHigh = False
		maxE = 0
		for t in t_traj:
			q, q_dot, gain, err = self.kinInv_3D(q, q_dot, solo, t)
			
			q_traj.append(q)
			qdot_traj.append(q_dot)
			gains.append(gain)
			
			if err>self.getParameter('max_kinv_error'):
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

		# Initialize angles at zero
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
		if self.getParameter('debug'):
			print("Done in", time.time()-t0, "s")

		# Define trajectory for return
		traj = ActuatorsTrajectory()
		traj.addElement('t', t_traj)
		traj.addElement('q', np.swapaxes(q_traj[:, 7:], 0, 1))
		traj.addElement('q_dot', np.swapaxes(qdot_traj[:, 6:], 0, 1))
		traj.addElement('gains', np.swapaxes(gains, 0, 1))

		return traj
	
	"""
	Kinetic inverse of the robot actuators state from feet position.
	
	:param q current configuration of the robot
	:param qdot current configuration speed of the robot
	:param solo model of the robot
	:param t_simu current time
	"""
	def kinInv_3D(self, q, qdot, solo, t_simu):
		from numpy.linalg import pinv

		ftraj = self.getParameter('feet_traj_func') # function Defining feet traj
		ftraj_kwargs = self.getParameter('feet_traj_params')
		K = self.getParameter('kinv_gain')  # Convergence gain

		# unactuated, [x, y, z] position of the base + [x, y, z, w] orientation of the base (stored as a quaternion)
		# qu = q[:7]
		# [v_x, v_y, v_z] linear velocity of the base and [w_x, w_y, w_z] angular velocity of the base along x, y, z axes
		# of the world
		# qu_dot = qdot[:6]

		qa_dot_ref = np.zeros((12, 1))  # target angular velocities for the motors

		if self.flag_q_ref:
			self.q_ref = solo.q0.copy()
			self.flag_q_ref = False

		# Compute/update all the joints and frames
		pin.forwardKinematics(solo.model, solo.data, self.q_ref)
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
		xyzdes_FL = ftraj(t_simu, 0, **ftraj_kwargs)[0]
		xyzdes_FR = ftraj(t_simu, 1, **ftraj_kwargs)[0]
		xyzdes_HL = ftraj(t_simu, 2, **ftraj_kwargs)[0]
		xyzdes_HR = ftraj(t_simu, 3, **ftraj_kwargs)[0]

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
		fJ_FL3 = pin.computeFrameJacobian(solo.model, solo.data, self.q_ref, ID_FL)[:3, -12:]  # Take only the translation terms
		oJ_FL3 = oR_FL.dot(fJ_FL3)  # Transformation from local frame to world frame
		oJ_FLxyz = oJ_FL3[0:3, -12:]  # Take the x,y & z components

		fJ_FR3 = pin.computeFrameJacobian(solo.model, solo.data, self.q_ref, ID_FR)[:3, -12:]
		oJ_FR3 = oR_FR.dot(fJ_FR3)
		oJ_FRxyz = oJ_FR3[0:3, -12:]

		fJ_HL3 = pin.computeFrameJacobian(solo.model, solo.data, self.q_ref, ID_HL)[:3, -12:]
		oJ_HL3 = oR_HL.dot(fJ_HL3)
		oJ_HLxyz = oJ_HL3[0:3, -12:]

		fJ_HR3 = pin.computeFrameJacobian(solo.model, solo.data, self.q_ref, ID_HR)[:3, -12:]
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
		self.q_ref = pin.integrate(solo.model, self.q_ref, q_dot_ref * self.getParameter('dt'))
		qa_ref = self.q_ref[7:].reshape(12,1)

		# Return configuration of the robot
		gains = ftraj(t_simu, 0, **ftraj_kwargs)[1]
		q_dot_ref = np.squeeze(np.array(q_dot_ref))
		err = np.linalg.norm(nu)

		return self.q_ref, q_dot_ref, gains, err

	def trajFeet_jump1(self, t, footId, **kwargs):
		t0 = kwargs.get("traj_t0", 0.2)  # Duration of initialisation step
		t1 = kwargs.get("traj_t1", 0.25)  # Duration of first step (extension of legs)
		t2 = kwargs.get("traj_t2", 0.15)  # Duration of second step (spreading legs)

		dz = kwargs.get("traj_dz", 0.25)  # displacement amplitude by z
		dy = kwargs.get("traj_dy", 0.05)  # displacement amplitude by y
		dx = kwargs.get("traj_dx", dy)    # displacement amplitude by x

		# Gains parameters
		param_kp = kwargs.get("kp", 10)	# default parameter of kp
		param_kd = kwargs.get("kd", 0.1)	# default parameter of kd
		param_kps = kwargs.get("kps", [param_kp, param_kp])	# default parameter of kps
		param_kds = kwargs.get("kds", [param_kp, param_kd])	# default parameter of kds

		# Initialization of the variables
		traj_x0 = 0.190 + kwargs.get("traj_dx0", 0) # initial distance on the x axis from strait
		traj_y0 = 0.147 + kwargs.get("traj_dy0", 0) # initial distance on the y axis from strait
		traj_z0 = kwargs.get("traj_z0", -0.05) # initial distance on the z axis from body
		traj_zf = kwargs.get("traj_zf", -0.25) # initial distance on the z axis from body

		if traj_z0>=0 or traj_zf>=0:
			print("traj_z0 or traj_zf might be positive. This may lead to invalid configuration.")

		x0 = -traj_x0 if int(footId/2)%2 else traj_x0
		y0 = -traj_y0 if footId%2 else traj_y0

		x, y, z = 0, 0, 0
		gains = [0, 0]

		# If time is overlapping the duration of the sequence, stay in last position
		if t>t0+t1+t2:
			t=t0+t1+t2

		# First part of the jump, push while staying at same position
		if t < t0:
			z = traj_z0-dz/2*np.cos(np.pi/2 * t/t0)

			x = x0
			y = y0

			gains[0] = 10
			gains[1] = 5
		# Second part of the jump, push while staying at same position
		elif t < t0+t1:
			t = t-t0
			z = traj_z0-dz*np.sin(np.pi/2 * t/t1)

			x = x0
			y = y0

			gains[0] = param_kps[0]
			gains[1] = param_kds[0]
		# Third part of the jump, extend feet and retract legs
		elif t <= t0+t1+t2:
			t = t-(t0+t1)

			if x0>0:
				x = x0 + dx * np.sin(np.pi/2 * t/t2)
			else:
				x = x0 - dx * np.sin(np.pi/2 * t/t2)

			if y0>0:
				y = y0 + dy * np.sin(np.pi/2 * t/t2)
			else:
				y = y0 - dy * np.sin(np.pi/2 * t/t2)
			
			z = traj_zf + (-traj_zf+traj_z0-dz)*np.sin(np.pi/2 *(1 - t/t2))

			gains[0] = param_kps[1]
			gains[1] = param_kds[1]

		return np.array([x, y, z]), gains


"""
Trajectory Generator using TSID.
"""
class TrajectoryGen_TSID(TrajectoryGenerator):
	def __init__(self):
		TrajectoryGenerator.__init__(self)

		self.name = "TSID Trajectory"

		# Defining default parameters of the trajectory
		self.parametersDefaults['dt'] = 1e-4
		self.parametersDefaults['kp'] = 5
		self.parametersDefaults['kd'] = 1
		self.parametersDefaults['verticalVelocity'] = 0.1

		self.parametersDefaults['debug'] = True
		self.parametersDefaults['gepetto_viewer'] = False
	
	def generateTrajectory(self, **kwargs):
		import time
		from .solo_tsid import SoloTSID

		# Load parameters of the trajectory
		self.setParametersFromDict(**kwargs)
		dt = self.getParameter('dt')
		param_kp = self.getParameter('kp')
		param_kd = self.getParameter('kd')
		param_vertVel = self.getParameter('verticalVelocity')
		
		t0 = time.time()
		
		if self.getParameter('debug'):
			print("Computing Trajectory...")
			self.printInfo()
		
		# Initialize Viewer if needed
		if self.getParameter('gepetto_viewer'):
			solo12 = loadSolo(False)
			solo12.initViewer(loadModel=True)
			
		# Initialize TSID
		tsid = SoloTSID()
		tsid.setCOMRef(np.array([0.0,0.0,-0.05]).T, np.zeros(3), np.zeros(3))
		tsid.setBaseRef()

		comObj1 = tsid.getCOM()+np.array([0.0,0.0,-0.05]).T
		comObj2 = tsid.getCOM()+np.array([0.0,0.0,0.09]).T

		# Initalize trajectories
		N_simu = 15000
		q      = np.zeros((tsid.solo12_wrapper.nq, N_simu + 1))
		v      = np.zeros((tsid.solo12_wrapper.nv, N_simu + 1))
		dv     = np.zeros((tsid.solo12_wrapper.nv, N_simu + 1))
		tau    = np.zeros((tsid.solo12_wrapper.na, N_simu + 1))
		gains  = np.zeros((2,N_simu+1))
		t_traj = np.arange(0, N_simu+1)/self.getParameter('dt')

		# Launch simu
		t = 0.0
		t_traj[0] = t
		q[:, 0], v[:, 0] = tsid.q0, tsid.v0
		gains[:,0] = np.array([param_kp, param_kd])

		for i in range(N_simu-2):
			
			HQPData = tsid.formulation.computeProblemData(t, q[:,i], v[:,i])
			sol = tsid.solver.solve(HQPData)
			
			com = tsid.getCOM()
			deltaCom1 = abs(com[2] - comObj1[2])
			deltaCom2 = abs(com[2] - comObj2[2])
			if deltaCom1 < 2e-2:
				tsid.setCOMRef(np.array([0.0,0.0,0.1]).T, np.array([0.0,0.0,param_vertVel]), np.zeros(3))
			
			if deltaCom2 < 1e-2:
				break
			
			if sol.status != 0:
				print("Time {0:0.3f} QP problem could not be solved! Error code: {1}".format(t, sol.status))
				break
			
			tau[:,i] = tsid.formulation.getActuatorForces(sol)
			dv[:,i] = tsid.formulation.getAccelerations(sol)
			
			# Numerical integration
			q[:,i + 1], v[:,i + 1] = tsid.integrate_dv(q[:,i], v[:,i], dv[:,i], dt)
			t += dt
			t_traj[i+1] = t
			
			# Set the gains
			gains[:,i+1] = np.array([param_kd, param_kp])
			
			if self.getParameter('gepetto_viewer'):
				solo12.display(q[:,i])
				time.sleep(1e-3)
		
		# Adding the last configuration 
		t_traj[i+1] = t_traj[i]+dt
		q[:, i+1], v[:, i+1] = tsid.q0, tsid.v0
		tau[:, i] = np.zeros(tsid.solo12_wrapper.na)
		tau[:, i+1] = np.zeros(tsid.solo12_wrapper.na)
		gains[:,i+1] = np.array([param_kd, param_kp])

		# Print execution time if requiered
		if self.getParameter('debug'):
			print("Done in", time.time()-t0, "s")
		
		# Define trajectory for return
		traj = ActuatorsTrajectory()
		traj.addElement('t', t_traj[0:i+2])
		traj.addElement('q', q[7:,0:i+2])
		traj.addElement('q_dot', v[6:,0:i+2])
		traj.addElement('gains', gains[:,0:i+2])
		traj.addElement('torques', tau[:, 0:i+2])

		return traj


"""
Trajectory Generator using Crocoddyl.
"""
class TrajectoryGen_Croco(TrajectoryGenerator):
	def __init__(self):
		TrajectoryGenerator.__init__(self)

		self.name = "TSID Trajectory"

		# Defining default parameters of the trajectory
		self.parametersDefaults['display'] = True
		self.parametersDefaults['height'] = 0.25
	
	def generateTrajectory(self, **kwargs):
		import crocoddyl
		from crocoddyl.utils.quadruped import SimpleQuadrupedalGaitProblem, plotSolution

		# Load parameters of the trajectory
		self.setParametersFromDict(**kwargs)
		param_vertVel = self.getParameter('verticalVelocity')
		
		# Loading the solo model
		solo = loadSolo(False)
		robot_model = solo.model
		lims = robot_model.effortLimit

		# Setting up CoM problem
		lfFoot, rfFoot, lhFoot, rhFoot = 'FL_FOOT', 'FR_FOOT', 'HL_FOOT', 'HR_FOOT'
		gait = SimpleQuadrupedalGaitProblem(robot_model, lfFoot, rfFoot, lhFoot, rhFoot)
		
		# Defining the initial state of the robot
		q0 = robot_model.referenceConfigurations['standing'].copy()
		v0 = pin.utils.zero(robot_model.nv)
		x0 = np.concatenate([q0, v0])

		# Defining the CoM gait parameters
		Jumping_gait = {'jumpHeight': self.getParameter('height'), 'jumpLength': [0,0,0.5], 'timeStep': 1e-2, 'groundKnots': 25, 'flyingKnots': 25}

		# Setting up the control-limited DDP solver
		boxddp = crocoddyl.SolverBoxDDP(
			gait.createJumpingProblem(x0, Jumping_gait['jumpHeight'], Jumping_gait['jumpLength'], 
											Jumping_gait['timeStep'], Jumping_gait['groundKnots'],
											Jumping_gait['flyingKnots']))

		# Add the callback functions
		print('*** SOLVE ***')
		cameraTF = [2., 2.68, 0.84, 0.2, 0.62, 0.72, 0.22]
		if self.getParameter('display'):
			display = crocoddyl.GepettoDisplay(solo, 4, 4, cameraTF, frameNames=[lfFoot, rfFoot, lhFoot, rhFoot])
			boxddp.setCallbacks([crocoddyl.CallbackVerbose(), crocoddyl.CallbackDisplay(display)])
		else:
			boxddp.setCallbacks([crocoddyl.CallbackVerbose()])

		xs = [robot_model.defaultState] * (boxddp.problem.T + 1)
		us = boxddp.problem.quasiStatic([solo.model.defaultState] * boxddp.problem.T)

		# Solve the DDP problem
		boxddp.solve(xs, us, 100, False, 0.1)

		# Display the entire motion
		if self.getParameter('display'):
			display = crocoddyl.GepettoDisplay(solo, frameNames=[lfFoot, rfFoot, lhFoot, rhFoot])
			display.displayFromSolver(boxddp)

		# Define trajectory for return
		traj = ActuatorsTrajectory()

		return traj



"""
Simple example to show how to use it
"""
if __name__ == '__main__':
	import time
	
	# Create a instance of trajectory generator
	traj_gen = TrajectoryGen_InvKin()

	# Set a parameter of the generator
	kwargs_trajec = {"traj_dx0":0.05, "traj_t0":0.2, "traj_t1":0.25, "traj_z0":-0.05, "traj_zf":-0.25, "kps":[10, 2], "kds":[0.1, 0.08]}
	traj_gen.setParameter('feet_traj_params', kwargs_trajec)

	# Set multiple parameter of the genetor
	kwargs_trajGen = {'debug':True, 'dt':0.01}
	traj_gen.setParametersFromDict(**kwargs_trajGen)

	# Display Informations about the generator
	traj_gen.printInfo()

	# Generate the trajectory
	traj = traj_gen.generateTrajectory()
	
	# Plot the trajectory
	traj.plotTrajectory()

	# Display the generated trajectory in Gepetto-Gui
	solo = loadSolo(False)
	solo.initViewer(loadModel=True)

	t0 = time.time()
	for i in range(traj.getSize()):
		q = np.concatenate((np.array([0, 0, 0.4, 0 ,0 ,0, 1]), traj.getElement('q', i)))
		solo.display(q)

		while(time.time()-t0<traj.getElement('t', i)):
			time.sleep(0.00001)

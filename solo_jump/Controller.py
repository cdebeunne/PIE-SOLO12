# coding: utf8

# Other modules
import numpy as np

"""
Default controller. Boring standing.
"""
class Controller:
	def __init__(self):
		self.name = "Default Controller: Standing"

		# Defining parameters of the controller
		self.default_parameters = {}
		self.default_parameters['gains'] = np.array([1, 1])
		self.default_parameters['torques_sat'] = 3*np.ones((12, 1))
		
		# Print informations if true
		self.debug = False
		self.debugPD = False
	
	"""
	Returns torques to 0.
	"""
	def stop(self):
		return np.zeros((12,1))

	"""
	Returns torques for standing position.
	"""
	def getTorques(self, q, q_dot, **kwargs):
		qa = q[7:]
		qa_dot = q_dot[6:]

		objective = {}
		objective['qa_ref'] = np.zeros((12, 1))  # target angular positions for the motors
		objective['qa_dot_ref'] = np.zeros((12, 1))  # target angular velocities for the motors

		return self.PD(qa, qa_dot, **objective)

	"""
	Prints current state of the controller in console
	"""
	def printState(self):
		print("Controller State: Doing Nothing")

	"""
	Simple PD controller with feedforward.

	:param qa Current position of the actuators. Must be an np.ndarray(12,1)
	:param qa_dot Current speed of the actuators. Must be an np.ndarray(12,1)
	:param objective Objective to reach. The element of the objective can be: qa_ref, qa_dot_ref, torques_ff, gains
	:ret torques to apply to each joint. It will be an np.ndarray(12,1)
	"""
	def PD(self, qa, qa_dot, **objective):
		qa_ref = objective.get('qa_ref', np.zeros((12,1)))
		qa_dot_ref = objective.get('qa_dot_ref', np.zeros((12,1)))
		torques_ff = objective.get('torques_ff', np.zeros((12,1)))

		gains = objective.get('gains', self.default_parameters['gains'])
		torques_sat = 3 #self.default_parameters['torques_sat']

		# Output torques
		Kp = gains[0]
		Kd = gains[1]
		torques = Kp*(qa_ref - qa) + Kd*(qa_dot_ref - qa_dot) + torques_ff

		# Saturation to limit the maximal value that torques can have
		torques = np.maximum(np.minimum(torques, torques_sat), -torques_sat)

		if self.debugPD:
			print("+------+-----------+-----------+-----------+")
			print("|  ID  |  QA_REF   |     QA    |   TORQUE  |")
			print("+------+-----------+-----------+-----------+")
			for i in range(len(qa_ref)):
				print("| j={0:2d} | ref={1:+3.2f} | cur={2:+3.2f} | tor={3:+3.2f} |".format(i, qa_ref[i][0],qa[i][0], torques[i][0]) )
			print("+------+-----------+-----------+-----------+\n")
			# print(objective)

		return torques

"""
Simple Jumping controller.
"""
class Controller_Jump(Controller):
	def __init__(self, trajectory):
		from math import pi

		Controller.__init__(self)

		self.name = "Basic Jumping Controller"

		# Parameters to store the current state of the jump
		self.isCrouched = False
		self.inAir = False

		# Definition of default parameters
		self.default_parameters['threshold_change'] = 0.5

		# Define the different configurations of the jump
		self.q_crouch = np.zeros((12,1))
		self.q_air = np.zeros((12,1))
		self.q_jump = np.zeros((12,1))

		pos_crouch = np.array([[0, 0.9*pi/2, -0.9*pi], \
							[0, 0.9*pi/2, -0.9*pi], \
							[0, -0.9*pi/2, 0.9*pi], \
							[0, -0.9*pi/2, 0.9*pi]])
		for leg in range(4):
				for art in range(3):
					self.q_crouch[3*leg+art] = 0.8*pos_crouch[leg, art]
					self.q_air[3*leg+art] = 0.5*pos_crouch[leg,art]

	def printState(self):
		print("Controller State:")
		print("\t- isCrouched: {0}".format(self.isCrouched))
		print("\t- inAir: {0}".format(self.inAir))

	"""
	Returns torques for standing position.
	"""
	def getTorques(self, q, q_dot, **kwargs):
		from numpy.linalg import norm

		qa = q[7:]
		qa_dot = q_dot[6:]

		objective = {}

		# Update current state of the jump
		if not self.isCrouched:
			self.isCrouched = norm(qa-self.q_crouch)<self.default_parameters['threshold_change']
		if self.isCrouched and not self.inAir:
			self.inAir = norm(qa-self.q_jump)<self.default_parameters['threshold_change']

		# Get the required congiguration given the state		
		if self.inAir:
			objective['qa_ref'] = self.q_air
			objective['gains'] = np.array([1, 10])
		elif self.isCrouched:
			objective['qa_ref'] = self.q_jump
			objective['gains'] = np.array([0.5, 20])
		else:
			objective['qa_ref'] = self.q_crouch
			objective['gains'] = np.array([1, 5])
		
		return self.PD(qa, qa_dot, **objective)

"""
Controller to get the torques from a given ActuatorsTrajectory.
"""
class Controller_Traj(Controller):
	def __init__(self, trajectory):
		Controller.__init__(self)

		self.name = "Trajectory Controller"
		self.initialized = False
		self.ended = False
		self.offset = 0
		self.stopAtEnd = False
		
		self.default_parameters['init_threshold'] = 0.3
		self.default_parameters['init_gains'] = np.array([1, 0.5])

		self.trajectory = trajectory
	
	def printState(self):
		print("Controller State:")
		print("\t- initialized: {0}".format(self.initialized))
		print("\t- ended: {0}".format(self.ended))

	"""
	Returns torques for standing position.
	"""
	def getTorques(self, q, q_dot, **kwargs):
		if not 't' in kwargs:
			print("This controller needs \'t\' as an arguement. Returning stop.")
			return self.stop()

		qa = q[7:]
		qa_dot = q_dot[6:]
		t = kwargs['t']

		objective = {}

		# Reach the first position of the trajectory first
		if not self.initialized :
			self.offset = t

			objective['qa_ref'] = self.trajectory.getElement('q', 0).reshape((12, 1))
			objective['qa_dot_ref'] = self.trajectory.getElement('q_dot', 0).reshape((12, 1))
			objective['gains'] = self.default_parameters['init_gains']

			# If it is reached, continue
			if np.linalg.norm(objective['qa_ref']-qa) < self.default_parameters['init_threshold']:
				if self.debug:
					print('Reached first state in {0:3.2f} s.'.format(t))
				self.initialized = True
		# Then run the trajectory
		else:
			# Apply offset
			t = t-self.offset

			# Get the current state in the trajectory
			if t>self.trajectory.getElement('t', -1) and not self.ended:
				if self.debug:
					print("End of trajectory.", end='')
					if self.stopAtEnd:
						print("Stopping actuators.")
					else:
						print("Staying in last state.")
				self.ended = True
			
			if self.ended and self.stopAtEnd:
				return self.stop()

			objective['qa_ref'] = 		self.trajectory.getElementAtTime('q', t).reshape((12, 1))
			objective['qa_dot_ref'] =  	self.trajectory.getElementAtTime('q_dot', t).reshape((12, 1))
			objective['torques_ff'] =  	self.trajectory.getElementAtTime('torques', t).reshape((12, 1))
			objective['gains'] = 		self.trajectory.getElementAtTime('gains', t)
		
		return self.PD(qa, qa_dot, **objective)

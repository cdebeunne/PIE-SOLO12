import time

import pybullet as p  # PyBullet simulator

from solo_pybullet.trajectory import *
from solo_pybullet.controller import *
# Functions to initialize the simulation and retrieve joints positions/velocities
from solo_pybullet.initialization_simulation import configure_simulation, getPosVelJoints

def jump_functor(t_crouch, t_jump, t_air):
	####################
	#  INITIALIZATION ##
	####################

	sim_dt = 0.0001  # time step of the simulation
	sim_tfinal = 5 # end time of the simulation
	sim_gravity_enable = True

	enableGUI = False  # enable PyBullet GUI or not
	robotId, solo, revoluteJointIndices = configure_simulation(sim_dt, enableGUI, sim_gravity_enable)

	###############
	#  MAIN LOOP ##
	###############

	q, qdot = getPosVelJoints(robotId, revoluteJointIndices)
	q_traj, qdot_traj, gains = jumpTrajectory(q, t_crouch,t_jump,t_air, sim_dt)
	height_list = np.zeros(int(sim_tfinal/sim_dt))

	for i in range(int(sim_tfinal/sim_dt)):  # run the simulation during dt * i_max seconds (simulation time)
		cur_time = i*sim_dt

		# Get position and velocity of all joints in PyBullet (free flying base + motors)
		q, qdot = getPosVelJoints(robotId, revoluteJointIndices)

		# Call controller to get torques for all joints
		#jointTorques, isCrouched, inAir = jump(q, qdot, solo, sim_dt, isCrouched, inAir)
		jointTorques = splineJump(q, qdot, solo, q_traj, qdot_traj, gains, i, sim_dt)
		height_list[i] = abs(q[3])

		# Set control torques for all joints in PyBullet
		p.setJointMotorControlArray(robotId, revoluteJointIndices, controlMode=p.TORQUE_CONTROL, forces=jointTorques)

		# Compute one step of simulation
		p.stepSimulation()

	# Shut down the PyBullet client
	p.disconnect()
	cost = np.amax(height_list)
	return cost

if __name__ == "__main__":
    # execute only if run as a script
    print(np.amax(jump_functor(1.5,1.65,2)))

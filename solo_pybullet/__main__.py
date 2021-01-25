# coding: utf8

#####################
#  LOADING MODULES ##
#####################

import time

import pybullet as p  # PyBullet simulator

####################
#  INITIALIZATION ##
####################

# Functions to initialize the simulation and retrieve joints positions/velocities
from .initialization_simulation import configure_simulation, getPosVelJoints

sim_dt = 0.0001  # time step of the simulation
sim_tfinal = 5 # end time of the simulation
sim_gravity_enable = True
# If True then we will sleep in the main loop to have a 1:1 ratio of (elapsed real time / elapsed time in the
# simulation)
sim_realTime_enable = True
sim_slowMotion_enable = False
sim_slowMotion_ratio = 100

enableGUI = True  # enable PyBullet GUI or not
robotId, solo, revoluteJointIndices = configure_simulation(sim_dt, enableGUI, sim_gravity_enable)

######################
#  IMPORT TRAJECTORY #
######################

from .trajectory import jumpTrajectory_1, jumpTrajectory_2
from .controller import control_traj
from .security import check_integrity
# MatplotLib must be imported after Pybullet as been initialized in order to solve conflicts.

###############
#  MAIN LOOP ##
###############

q, qdot = getPosVelJoints(robotId, revoluteJointIndices)

# Compute Joint Trajectory
t_traj, q_traj, qdot_traj, gains_traj = jumpTrajectory_1(q, 1, 2, 3, 10*sim_dt)
t_traj, q_traj, qdot_traj, gains_traj = jumpTrajectory_2(init_reversed=False, tf=1, kp=10, dt=0.01, debug=True, traj_dx0=0.1, traj_t0=0.1, traj_t1=0.2, traj_zf=-0.2)

for i in range(int(sim_tfinal/sim_dt)):  # run the simulation during dt * i_max seconds (simulation time)
    cur_time = i*sim_dt

    # Time at the start of the loop
    if enableGUI and (sim_realTime_enable or sim_slowMotion_enable):
        t0 = time.clock()

    # Get position and velocity of all joints in PyBullet (free flying base + motors)
    q, qdot = getPosVelJoints(robotId, revoluteJointIndices)

    # Call controller to get torques for all joints
    jointTorques = control_traj(q, qdot, solo, t_traj, q_traj, qdot_traj, gains_traj, cur_time, sim_dt)

    # Set control torques for all joints in PyBullet
    p.setJointMotorControlArray(robotId, revoluteJointIndices, controlMode=p.TORQUE_CONTROL, forces=jointTorques)

    # Compute one step of simulation
    p.stepSimulation()

    check_integrity(solo, q, qdot)

    if enableGUI:
        solo.display(q)

    # Sleep to get a real time simulation
    if enableGUI and (sim_realTime_enable or sim_slowMotion_enable):
        t_sleep = sim_dt - (time.clock() - t0)
        if t_sleep > 0:
            time.sleep(t_sleep*(sim_slowMotion_ratio if sim_slowMotion_enable else 1))

# Shut down the PyBullet client
p.disconnect()

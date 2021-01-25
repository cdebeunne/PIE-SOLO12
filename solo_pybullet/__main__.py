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
sim_tfinal = 50 # end time of the simulation
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

from .trajectory import *
from .controller import *
# MatplotLib must be imported after Pybullet as been initialized in order to solve conflicts.

###############
#  MAIN LOOP ##
###############

isCrouched = False
inAir = False 
q, qdot = getPosVelJoints(robotId, revoluteJointIndices)

# Compute Joint Trajectory
t_traj, q_traj, qdot_traj, gains_traj = jumpTrajectory_1(q, 1, 2, 3, 10*sim_dt)
t_traj, q_traj, qdot_traj, gains_traj = jumpTrajectory_2(init_reversed=True, traj_T=.5, kp=10, dt=0.01, debug=True)

for i in range(int(sim_tfinal/sim_dt)):  # run the simulation during dt * i_max seconds (simulation time)
    cur_time = i*sim_dt

    # Time at the start of the loop
    if sim_realTime_enable or sim_slowMotion_enable:
        t0 = time.clock()

    # Get position and velocity of all joints in PyBullet (free flying base + motors)
    q, qdot = getPosVelJoints(robotId, revoluteJointIndices)

    # Call controller to get torques for all joints
    jointTorques = control_traj(q, qdot, solo, t_traj, q_traj, qdot_traj, gains_traj, cur_time, sim_dt)

    # Set control torques for all joints in PyBullet
    p.setJointMotorControlArray(robotId, revoluteJointIndices, controlMode=p.TORQUE_CONTROL, forces=jointTorques)

    # Compute one step of simulation
    p.stepSimulation()
    if enableGUI:
        solo.display(q)

    # Sleep to get a real time simulation
    if sim_realTime_enable:
        t_sleep = 0
        #t_sleep = sim_dt - (time.clock() - t0)
        if t_sleep > 0:
            time.sleep(t_sleep)
    elif sim_slowMotion_enable:
        t_sleep = sim_slowMotion_ratio*sim_dt - (time.clock() - t0)
        if t_sleep > 0:
            time.sleep(t_sleep)

# Shut down the PyBullet client
p.disconnect()

# coding: utf8

#####################
#  LOADING MODULES ##
#####################

import time

import pybullet as p  # PyBullet simulator

from .trajectory import *
from .controller import *
# Functions to initialize the simulation and retrieve joints positions/velocities
from .initialization_simulation import configure_simulation, getPosVelJoints

####################
#  INITIALIZATION ##
####################

sim_dt = 0.0001  # time step of the simulation
sim_tfinal = 5 # end time of the simulation
sim_gravity_enable = True
# If True then we will sleep in the main loop to have a 1:1 ratio of (elapsed real time / elapsed time in the
# simulation)
sim_realTime_enable = True
sim_slowMotion_enable = False
sim_slowMotion_ratio = 10

enableGUI = True  # enable PyBullet GUI or not
robotId, solo, revoluteJointIndices = configure_simulation(sim_dt, enableGUI, sim_gravity_enable)

###############
#  MAIN LOOP ##
###############

isCrouched = False
inAir = False 
q, qdot = getPosVelJoints(robotId, revoluteJointIndices)
q_traj, qdot_traj, gains = jumpTrajectory(q, 1.5,1.65,2, sim_dt)


for i in range(int(sim_tfinal/sim_dt)):  # run the simulation during dt * i_max seconds (simulation time)
    cur_time = i*sim_dt

    # Time at the start of the loop
    if sim_realTime_enable or sim_slowMotion_enable:
        t0 = time.clock()

    # Get position and velocity of all joints in PyBullet (free flying base + motors)
    q, qdot = getPosVelJoints(robotId, revoluteJointIndices)

    # Call controller to get torques for all joints
    #jointTorques, isCrouched, inAir = jump(q, qdot, solo, sim_dt, isCrouched, inAir)
    jointTorques = splineJump(q, qdot, solo, q_traj, qdot_traj, gains, i, sim_dt)

    # Set control torques for all joints in PyBullet
    p.setJointMotorControlArray(robotId, revoluteJointIndices, controlMode=p.TORQUE_CONTROL, forces=jointTorques)

    # Compute one step of simulation
    p.stepSimulation()
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

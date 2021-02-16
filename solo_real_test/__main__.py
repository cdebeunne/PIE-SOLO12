# coding: utf8

#####################
#  LOADING MODULES ##
#####################

import time

import pybullet as p  # PyBullet simulator

from solo_jump.TrajectoryGenerator import ActuatorsTrajectory, TrajectoryGen_InvKin, TrajectoryGen_TSID, TrajectoryGen_Croco, TrajectoryGen_Splines
from solo_jump.Controller import Controller_Traj
from solo_jump.SecurityChecker import SecurityChecker

from solo_pybullet.initialization_simulation import configure_simulation, getPosVelJoints

####################
#  INITIALIZATION  #
####################

# MatplotLib must be imported after Pybullet as been initialized in order to solve conflicts.

kwargs_simu = {"dt":0.0001, "max_time":10, "enableGravity":True, "realTime":False, "enableGUI":True, "slowMo":False, "slowMoRatio":100}
robotId, solo, revoluteJointIndices = configure_simulation(**kwargs_simu)

######################
#  IMPORT TRAJECTORY #
######################

# Parameters for 
kwargs_trajec = {"traj_dx0":0.05, "traj_t0":0.2, "traj_t1":0.25, "traj_z0":-0.05, "traj_zf":-0.25, "kps":[10, 2], "kds":[0.1, 0.08]}
kwargs_KinInv = {"init_reversed":True, "tf":1.5, "dt":0.01, "debug":True, "feet_traj_params":kwargs_trajec}
kwargs_splines = {"t_crouch":1, "t_jump":1.2, "t_air":2, "dt":0.05}
kwargs_TSID = {"verticalVelocity":0.4, "kp":10, "kd":5}
kwargs_Croco = {'gepetto_viewer':False, "height":0.1}

# Compute Joint Trajectory
traj_gen = TrajectoryGen_TSID()
traj_gen.setParametersFromDict(**kwargs_TSID)
actuators_traj = traj_gen.generateTrajectory()

# Plot trajectory of the actuators
actuators_traj.plotTrajectory(show_gains=True, show_all=True)

###############
#  CONTROLLER #
###############

control = Controller_Traj(actuators_traj)
control.debug = True

##############
#  SECURITY  #
##############

secu = SecurityChecker()
secu.verbose = True

##############
#  MAIN LOOP #
##############

# Ask if ready
print("Ready to rock !")
print("Do you wanna rock ? [Y] to continue, [n] to abort")
key = input()

if key is not "Y":
    print("Aborting le bateau")
    exit()

# Initialisation of the trajectory
print("\n")
print("Going to the first position of the trajectory.")

reached_init = False
while not reached_init:
    q, qdot = getPosVelJoints(robotId, revoluteJointIndices)
    jointTorques, reached_init = control.gotoFirstPosition(q, qdot)
    p.setJointMotorControlArray(robotId, revoluteJointIndices, controlMode=p.TORQUE_CONTROL, forces=jointTorques)
    p.stepSimulation()

print("Reached first position.")
print("Do you wanna continue ? [Y] to continue, [n] to abort")
key = input()

if key is not "Y":
    print("Aborting le bateau")
    exit()

# Following the trajectory
calibration_done = False
control.initialize(0)
next_tick = time.time()

cur_time = 0
while not control.ended:
    cur_time += kwargs_simu.get("dt", 0.0001)
    q, qdot = getPosVelJoints(robotId, revoluteJointIndices)
    jointTorques = control.getTorques(q, qdot, t=cur_time)

    secu.check_limits(q)
    secu.check_speed(qdot)
    secu.check_torques(jointTorques)

    p.setJointMotorControlArray(robotId, revoluteJointIndices, controlMode=p.TORQUE_CONTROL, forces=jointTorques)
    p.stepSimulation()

# Print out security results
secu.show_results(show_all=True)

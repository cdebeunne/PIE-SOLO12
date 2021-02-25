# coding: utf8

#####################
#  LOADING MODULES ##
#####################

import time
import os

import pybullet as p  # PyBullet simulator

from solo_jump.TrajectoryGenerator import ActuatorsTrajectory, TrajectoryGen_Splines
from solo_jump.Controller import Controller_Traj
from solo_jump.SecurityChecker import SecurityChecker
from solo_pybullet.SoloSimulation import SoloSimulation

####################
#  INITIALIZATION  #
####################

# MatplotLib must be imported after Pybullet as been initialized in order to solve conflicts.
simulator = SoloSimulation(enableGUI=True, enableGravity=True)

##############
# TRAJECTORY #
##############

kwargs_traj = {"t_crouch":1, "t_jump":1.2, "t_air":2, "dt":0.05}

# Compute Joint Trajectory
traj_gen = TrajectoryGen_Splines()
traj_gen.setParametersFromDict(**kwargs_traj)
actuators_traj = traj_gen.generateTrajectory()

actuators_traj.saveTrajectory("/tmp/traj.npz")

###############
#  CONTROLLER #
###############

actuators_traj = ActuatorsTrajectory()
actuators_traj.loadTrajectory("/tmp/traj.npz", verbose=True)

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

# Reache first state
reached_init = False
on_ground = False

simulator.change_gravity(0)

while not reached_init or not on_ground:
    qa, qadot = simulator.get_state_a()

    jointTorques, reached_init = control.goto_first_position(qa, qadot)
    on_ground = simulator.is_on_ground()
    
    if reached_init:
        simulator.change_gravity(-9.81)

    simulator.set_joint_torques(jointTorques)
    simulator.step()

print("Reached first position.")
print("Do you wanna continue ? [Y] to continue, [n] to abort")
key = input()

if key is not "Y":
    print("Aborting le bateau")
    exit()

# Following the trajectory
control.initialize(simulator.simulation_time)

while not control.ended:
    q, qdot = simulator.get_state()
    qa, qadot = simulator.get_state_a()

    jointTorques = control.get_torques(qa, qadot, t=simulator.simulation_time)

    secu.check_integrity(q, qdot, jointTorques)

    simulator.set_joint_torques(jointTorques)
    simulator.step()

# Print out security results
secu.show_results(show_all=True)

# Delete temporary traj file
os.remove("/tmp/traj.npz")

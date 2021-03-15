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

PRESS_KEY = False
name_traj = "trajectories/traj_6.npz"

print("Running trajectory named:", name_traj)

####################
#  INITIALIZATION  #
####################

# MatplotLib must be imported after Pybullet as been initialized in order to solve conflicts.
simulator = SoloSimulation(enableGUI=True, enableGravity=True)

##############
# TRAJECTORY #
##############

actuators_traj = ActuatorsTrajectory()
actuators_traj.loadTrajectory(name_traj, verbose=True)

###############
#  CONTROLLER #
###############

control = Controller_Traj(actuators_traj)
control.debug = False

##############
#  SECURITY  #
##############

secu = SecurityChecker()
secu.verbose = False

##############
#  MAIN LOOP #
##############

# Ask if ready
print("Ready to rock !")
if PRESS_KEY:
    print("Do you wanna rock ? [Y] to continue, [n] to abort")
    key = input()

    if key is not "Y":
        print("Aborting le bateau")
        exit()

    # Initialisation of the trajectory
    print("\n")
print("Going to the first position of the trajectory.")

# Reaching first state
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

# Reached first position
print("Reached first position.")
if PRESS_KEY:
    print("Do you wanna continue ? [Y] to continue, [n] to abort")
    key = input()

    if key is not "Y":
        print("Aborting le bateau")
        exit()
print("Following trajectory.")

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

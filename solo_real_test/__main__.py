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
from solo_jump.Logger import Logger
from solo_pybullet.SoloSimulation import SoloSimulation

LOG = True         # Log everything of the trajectory
PRESS_KEY = False   # Press a key to pass throught steps
SECURITY = False    # Run security checking
name_traj = "trajectories/traj_7.npz"

print("Running trajectory named:", name_traj)

####################
#  INITIALIZATION  #
####################

# MatplotLib must be imported after Pybullet as been initialized in order to solve conflicts.
simulator = SoloSimulation(enableGUI=False, enableGravity=True)

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

############
#  LOGGER  #
############

logger = Logger()

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
offset_time = simulator.simulation_time
control.initialize(simulator.simulation_time)

while not control.ended or not simulator.is_stop():
    q, qdot = simulator.get_state()
    qa, qadot = simulator.get_state_a()
    t = simulator.simulation_time

    jointTorques = control.get_torques(qa, qadot, t=t)

    if LOG:
        logger.add_data(simulator.simulation_time,
                        q, qdot,
                        actuators_traj.getElementAtTime('q', t-offset_time),
                        actuators_traj.getElementAtTime('q_dot', t-offset_time),
                        actuators_traj.getElementAtTime('torques', t-offset_time),
                        jointTorques,
                        actuators_traj.getElementAtTime('gains', t-offset_time))

    if SECURITY:
        secu.check_integrity(q, qdot, jointTorques)

    simulator.set_joint_torques(jointTorques)
    simulator.step()

simulator.end()
logger.end()

############
#  RESULTS #
############

if SECURITY:
    # Print out security results
    secu.show_results(show_all=True)

if LOG:
    import matplotlib.pyplot as plt

    plt.plot(logger.t, logger.q[:, 2])
    plt.title("Evolution of CG z")
    plt.grid()
    plt.show()

    logger.plot_leg()
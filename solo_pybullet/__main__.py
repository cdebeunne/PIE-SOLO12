# coding: utf8

#####################
#  LOADING MODULES ##
#####################

import pybullet as p  # PyBullet simulator
import time

from solo_jump.TrajectoryGenerator import ActuatorsTrajectory, TrajectoryGen_InvKin, TrajectoryGen_TSID, TrajectoryGen_Croco, TrajectoryGen_Splines
from solo_jump.Controller import Controller_Traj
from solo_jump.SecurityChecker import SecurityChecker
from solo_jump.PerfoChecker import PerformancesEvaluator

from .SoloSimulation import SoloSimulation

####################
#  INITIALIZATION  #
####################

# MatplotLib must be imported after Pybullet as been initialized in order to solve conflicts.

kwargs_simu = {"dt":0.0001, "max_time":10, "enableGravity":True, "realTime":False, "enableGUI":True, "slowMo":False, "slowMoRatio":100}
simulator = SoloSimulation(enableGUI=True)

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
# actuators_traj.plotTrajectory(show_gains=True, show_all=True)

###############
#  CONTROLLER #
###############

control = Controller_Traj(actuators_traj)
control.debug = True

##############
#  SECURITY  #
##############

secu = SecurityChecker()

##################
#  PERFORMANCES  #
##################

perfo = PerformancesEvaluator()

###############
#  MAIN LOOP ##
###############

# Reache first state
reached_init = False
while not reached_init:
    qa, qadot = simulator.get_state()
    jointTorques, reached_init = control.gotoFirstPosition(qa, qadot)
    simulator.set_joint_torques(jointTorques)
    simulator.step()

# Following the trajectory
control.initialize(simulator.simulation_time)

while not control.ended:
    q, qdot = simulator.get_state()
    qa, qadot = simulator.get_state_a()

    jointTorques = control.getTorques(q, qdot, t=simulator.simulation_time)

    secu.check_integrity(q, qdot, jointTorques)
    perfo.update_performance(q)

    simulator.set_joint_torques(jointTorques)
    simulator.step()

# Shut down the PyBullet client
simulator.end()

# Print out security results
secu.show_results(show_all=False)

# Print out the performances
perfo.show_results()

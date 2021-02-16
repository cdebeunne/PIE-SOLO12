# coding: utf8

#####################
#  LOADING MODULES ##
#####################

import pybullet as p  # PyBullet simulator
import time

from .TrajectoryGenerator import ActuatorsTrajectory, TrajectoryGen_InvKin, TrajectoryGen_TSID, TrajectoryGen_Croco, TrajectoryGen_Splines
from .controller import Controller_Traj
from .security import SecurityChecker

from .initialization_simulation import configure_simulation, getPosVelJoints

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

###############
#  MAIN LOOP ##
###############

q, qdot = getPosVelJoints(robotId, revoluteJointIndices)

cur_time = 0
while not control.ended:
    cur_time += kwargs_simu.get("dt", 0.0001)

    # Time at the start of the loop
    if kwargs_simu.get("enableGUI", False) and (kwargs_simu.get("realTime", False) or kwargs_simu.get("slowMo", False)):
        t0 = time.clock()

    # Get position and velocity of all joints in PyBullet (free flying base + motors)
    q, qdot = getPosVelJoints(robotId, revoluteJointIndices)

    # Call controller to get torques for all joints
    jointTorques = control.getTorques(q, qdot, t=cur_time)

    # Set control torques for all joints in PyBullet
    p.setJointMotorControlArray(robotId, revoluteJointIndices, controlMode=p.TORQUE_CONTROL, forces=jointTorques)

    # Compute one step of simulation
    p.stepSimulation()

    if control.initialized:
        secu.check_integrity(solo, q, qdot, jointTorques)

    if kwargs_simu.get("enableGUI", False):
        solo.display(q)

        # Sleep to get a real time simulation
        if kwargs_simu.get("realTime", False) or kwargs_simu.get("slowMo", False):
            t_sleep = kwargs_simu.get("dt", 0.0001) - (time.clock() - t0)
            if t_sleep > 0:
                time.sleep(t_sleep*(kwargs_simu.get("slowMoRatio", 100) if kwargs_simu.get("slowMo", False) else 1))

# Shut down the PyBullet client
p.disconnect()

# Print out security results
secu.show_results(show_all=False)

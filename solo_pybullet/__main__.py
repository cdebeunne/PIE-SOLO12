# coding: utf8

#####################
#  LOADING MODULES ##
#####################

import pybullet as p  # PyBullet simulator
import time

from .trajectory import jumpTrajectory_1, jumpTrajectory_2
from .controller import control_traj
from .security import check_integrity, showIntegrity

from .initialization_simulation import configure_simulation, getPosVelJoints


######################
#  IMPORT TRAJECTORY #
######################

# Parameters for 
kwargs_trajec = {"traj_dx0":0.05, "traj_t1":0.2, "traj_t2":0.25, "traj_z0":-0.05, "traj_zf":-0.25, "kps":[10, 2], "kds":[0.1, 0.08]}
kwargs_kininv = {"init_reversed":False, "tf":3, "dt":0.01, "debug":True}

# Compute Joint Trajectory
kwargs_jumpin = {**kwargs_trajec, **kwargs_kininv}
t_traj, q_traj, qdot_traj, gains_traj = jumpTrajectory_2(**kwargs_jumpin)

####################
#  INITIALIZATION  #
####################

# MatplotLib must be imported after Pybullet as been initialized in order to solve conflicts.

kwargs_simu = {"dt":0.0001, "max_time":5, "enableGravity":True, "realTime":False, "enableGUI":False, "slowMo":False, "slowMoRatio":100}
robotId, solo, revoluteJointIndices = configure_simulation(**kwargs_simu)

###############
#  MAIN LOOP  #
###############

q, qdot = getPosVelJoints(robotId, revoluteJointIndices)

height = []
qdot_plot = []

cur_time = 0
while not control_traj.ended:
    cur_time += kwargs_simu.get("dt", 0.0001)

    # Time at the start of the loop
    if kwargs_simu.get("enableGUI", False) and (kwargs_simu.get("realTime", False) or kwargs_simu.get("slowMo", False)):
        t0 = time.clock()

    # Get position and velocity of all joints in PyBullet (free flying base + motors)
    q, qdot = getPosVelJoints(robotId, revoluteJointIndices)

    # Call controller to get torques for all joints
    jointTorques = control_traj(q, qdot, solo, t_traj, q_traj, qdot_traj, gains_traj, cur_time, kwargs_simu.get("dt", 0.0001))

    # Set control torques for all joints in PyBullet
    p.setJointMotorControlArray(robotId, revoluteJointIndices, controlMode=p.TORQUE_CONTROL, forces=jointTorques)

    # Compute one step of simulation
    p.stepSimulation()

    check_integrity(solo, q, qdot)
    if control_traj.initialized:
        height.append(q[2])
        qdot_plot.append(qdot[6+5])

    if kwargs_simu.get("enableGUI", False):
        solo.display(q)

        # Sleep to get a real time simulation
        if kwargs_simu.get("realTime", False) or kwargs_simu.get("slowMo", False):
            t_sleep = kwargs_simu.get("dt", 0.0001) - (time.clock() - t0)
            if t_sleep > 0:
                time.sleep(t_sleep*(kwargs_simu.get("slowMoRatio", 100) if kwargs_simu.get("slowMo", False) else 1))

# Shut down the PyBullet client
p.disconnect()

showIntegrity()

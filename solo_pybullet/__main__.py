# coding: utf8

#####################
#  LOADING MODULES ##
#####################

import pybullet as p  # PyBullet simulator
import time

from .TrajectoryGenerator import ActuatorsTrajectory, TrajectoryGen_InvKin, TrajectoryGen_TSID
from .controller import Controller_Traj
from .security import check_integrity

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
kwargs_kininv = {"init_reversed":True, "tf":1.5, "dt":0.01, "debug":True}
kwargs_invDyn = {"disp":False, "verticalVelocity":0.2, "kp":10, "kd":5}

# Compute Joint Trajectory
kwargs_jumpin = {**kwargs_trajec, **kwargs_kininv}
traj_gen = TrajectoryGen_TSID()
traj_gen.setParametersFromDict(**kwargs_invDyn)
actuators_traj = traj_gen.generateTrajectory()

actuators_traj.plotTrajectory()

###############
#  MAIN LOOP ##
###############

q, qdot = getPosVelJoints(robotId, revoluteJointIndices)
control = Controller_Traj(actuators_traj)
# control.debug = True

cur_time = 0
while True:
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

    check_integrity(solo, q, qdot)

    if kwargs_simu.get("enableGUI", False):
        solo.display(q)

        # Sleep to get a real time simulation
        if kwargs_simu.get("realTime", False) or kwargs_simu.get("slowMo", False):
            t_sleep = kwargs_simu.get("dt", 0.0001) - (time.clock() - t0)
            if t_sleep > 0:
                time.sleep(t_sleep*(kwargs_simu.get("slowMoRatio", 100) if kwargs_simu.get("slowMo", False) else 1))

# Shut down the PyBullet client
p.disconnect()

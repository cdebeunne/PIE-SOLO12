# coding: utf8

#####################
#  LOADING MODULES ##
#####################

import time

import libmaster_board_sdk_pywrap as mbs
import libodri_control_interface_pywrap as oci

from solo_jump.TrajectoryGenerator import ActuatorsTrajectory, TrajectoryGen_InvKin, TrajectoryGen_TSID, TrajectoryGen_Croco, TrajectoryGen_Splines
from solo_jump.Controller import Controller_Traj
from solo_jump.SecurityChecker import SecurityChecker

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

###########
#  ROBOT  #
###########

robot = oci.robot_from_yaml_file('config_solo12.yaml')
joint_calibrator = oci.joint_calibrator_from_yaml_file('config_solo12.yaml', robot.joints)

# Initialize the communication and the session.
robot.start()
robot.wait_until_ready()

# Calibrate the robot if needed.
robot.run_calibration(joint_calibrator)

# Read the values once here. The returned values are views on the data and
# update after the call to `robot.parse_sensor_data()`.
robot.parse_sensor_data()
imu_attitude = robot.imu.attitude_euler
positions = robot.joints.positions
velocities = robot.joints.velocities

init_imu_attitude = imu_attitude.copy()

###############
#  MAIN LOOP ##
###############

c = 0
dt = 0.001
calibration_done = False
next_tick = time.time()

while not robot.is_timeout:
    cur_time = c*dt

    robot.parse_sensor_data()

    if c % 2000 == 0:
        print('IMU attitude:', imu_attitude)
        print('joint pos   :', positions)
        print('joint vel   :', velocities)
        robot.robot_interface.PrintStats()

    torques = control.getTorques(positions, velocities, t=cur_time)

    secu.check_limits(positions)
    secu.check_speed(velocities)
    secu.check_torques(torques)

    robot.joints.set_torques(torques)

    robot.send_command_and_wait_end_of_cycle()
    c += 1

# Print out security results
secu.show_results(show_all=True)

# coding: utf8

#####################
#  LOADING MODULES ##
#####################

import time

import libmaster_board_sdk_pywrap as mbs
import libodri_control_interface_pywrap as oci

from solo_jump.TrajectoryGenerator import ActuatorsTrajectory
from solo_jump.Controller import Controller_Traj
from solo_jump.SecurityChecker import SecurityChecker

traj_file = "test.npz"
traj_plot = False
check_security = False

######################
#  IMPORT TRAJECTORY #
######################

# Compute Joint Trajectory
actuators_traj = ActuatorsTrajectory()
actuators_traj.loadTrajectory(traj_file)

# Plot trajectory of the actuators
if traj_plot:
    actuators_traj.plotTrajectory(show_gains=True, show_all=True)

###############
#  CONTROLLER #
###############

control = Controller_Traj(actuators_traj)
control.debug = False

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

##############
#  MAIN LOOP #
##############

# Ask if ready
print("Ready to rock !")
print("Do you wanna rock ? [Y] to continue, [n] to abort")
key = input()

if key is not "Y":
    exit

# Initialisation of the trajectory
print("\n")
print("Going to the first position of the trajectory.")

c = 0
reached_init = False
while not robot.is_timeout and not reached_init:
    robot.parse_sensor_data()
    torques, reached_init = control.goto_first_position(positions, velocities)

    if c % 2000 == 0:
        print('IMU attitude:', imu_attitude)
        print('joint pos   :', positions)
        print('joint vel   :', velocities)
        print('torques     :', torques)
        robot.robot_interface.PrintStats()

    robot.joints.set_torques(torques)

    robot.send_command_and_wait_end_of_cycle()
    c += 1

print("Reached first position.")
print("Do you wanna continue ? [Y] to continue, [n] to abort")
key = input()

if key is not "Y":
    exit

# Following the trajectory
c = 0
dt = 0.001
calibration_done = False
control.initialize(time.time())
next_tick = time.time()

while not robot.is_timeout:
    robot.parse_sensor_data()
    torques = control.get_torques(positions, velocities, t=time.time())

    if c % 2000 == 0:
        print('IMU attitude:', imu_attitude)
        print('joint pos   :', positions)
        print('joint vel   :', velocities)
        print('torques     :', torques)
        robot.robot_interface.PrintStats()

    if check_security:
        secu.check_limits(positions)
        secu.check_speed(velocities)
        secu.check_torques(torques)

    robot.joints.set_torques(torques)

    robot.send_command_and_wait_end_of_cycle()
    c += 1

# Print out security results
if check_security:
    secu.show_results(show_all=True)

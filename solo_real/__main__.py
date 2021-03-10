# coding: utf8

#####################
#  LOADING MODULES ##
#####################

import time

from .solo12_ISAE import Solo12

import argparse

from solo_jump.TrajectoryGenerator import ActuatorsTrajectory
from solo_jump.Controller import Controller_Traj
from solo_jump.SecurityChecker import SecurityChecker

# Parsing Arguments

parser = argparse.ArgumentParser(description='Example masterboard use in python.')
parser.add_argument('-i',
                    '--interface',
                    required=True,
                    help='Name of the interface (use ifconfig in a terminal), for instance "enp1s0"')
name_interface  = parser.parse_args().interface

######################
#     PARAMETERS     #
######################

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

device = Solo12(name_interface,dt=0.001)
nb_motors = device.nb_motors
device.Init(calibrateEncoders=True)

# Read the values once here. The returned values are views on the data and
# update after the call to `device.UpdateMeasurment()`.
device.UpdateMeasurment()
positions = device.q_mes
velocities = device.v_mes

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
while not device.hardware.IsTimeout() and not reached_init:
    device.UpdateMeasurment()
    torques, reached_init = control.goto_first_position(device.q_mes, device.v_mes)

    if c % 2000 == 0:
        print('joint pos   :', device.q_mes)
        print('joint vel   :', device.v_mes)
        print('torques     :', torques)
        device.Print()

    device.SetDesiredJointTorque(torques)

    device.SendCommand(WaitEndOfCycle=True)
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

while not device.hardware.IsTimeout():
    device.UpdateMeasurment()
    torques = control.get_torques(device.q_mes, device.v_mes, t=time.time())

    if c % 2000 == 0:
        print('joint pos   :', device.q_mes)
        print('joint vel   :', device.v_mes)
        print('torques     :', torques)
        device.Print()

    if check_security:
        secu.check_limits(device.q_mes)
        secu.check_speed(device.v_mes)
        secu.check_torques(torques)

    device.SetDesiredJointTorque(torques)

    device.SendCommand(WaitEndOfCycle=True)
    c += 1

# Print out security results
if check_security:
    secu.show_results(show_all=True)

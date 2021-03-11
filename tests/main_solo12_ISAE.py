# coding: utf8
import numpy as np
import argparse
import math
import threading
from time import clock, sleep
from solo12_ISAE import Solo12

data = np.load("traj.npz")

t_traj = data['t']
q_traj = data['q']
v_traj = data['q_dot']
kp_traj = data['gains'][:, 0]
kd_traj = data['gains'][:, 1]

Kp = 0*0.1
Kd = 0*0.01

key_pressed = False

def get_input():
    global key_pressed
    keystrk = input('Put the robot on the floor and press Enter \n')
    # thread doesn't continue until key is pressed
    key_pressed = True

def put_on_the_floor(device, q_init):
    """Make the robot go to the default initial position and wait for the user
    to press the Enter key to start the main control loop
    Args:
        device (robot wrapper): a wrapper to communicate with the robot
        q_init (array): the default position of the robot
    """
    global key_pressed
    key_pressed = False
    Kp_pos = .1
    Kd_pos = 0.01
    imax = 3.0
    pos = np.zeros(device.nb_motors)

    for motor in range(device.nb_motors):
        pos[motor] = q_init[device.motorToUrdf[motor]] * device.gearRatioSigned[motor]
    
    i = threading.Thread(target=get_input)
    i.start()
    while not key_pressed:
        device.UpdateMeasurment()
        for motor in range(device.nb_motors):
            ref = Kp_pos*(pos[motor] - device.hardware.GetMotor(motor).GetPosition() -
                          Kd_pos*device.hardware.GetMotor(motor).GetVelocity())
            ref = min(imax, max(-imax, ref))
            device.hardware.GetMotor(motor).SetCurrentReference(ref)
        device.SendCommand(WaitEndOfCycle=True)

    print("Start the motion.")


def example_script(name_interface):
    device = Solo12(name_interface,dt=0.001)
    nb_motors = device.nb_motors
    
    device.Init(calibrateEncoders=True, q_init=q_traj[0])
    put_on_the_floor(device, q_traj[0])

    #CONTROL LOOP ***************************************************
    i_traj = 0
    initial_time = clock()

    while ((not device.hardware.IsTimeout()) and (clock() < 200)):
        device.UpdateMeasurment()

        # Get elements from trajectory 
        if clock()-initial_time > t_traj[i_traj] and i_traj+1<len(t_traj):
            i_traj += 1

        q_ref = q_traj[i_traj]
        v_ref = v_traj[i_traj]
        ff_ref = 0
        # Kp = kp_traj[i_traj]
        # Kd = kd_traj[i_traj]

        """
        # Compute torque
        q_err = q_ref - device.q_mes
        v_err = v_ref - device.v_mes
        tau = Kp*q_err + Kd*v_err + ff_ref
        device.SetDesiredJointTorque(tau)
        """
        device.SetDesiredJointPDgains([Kp]*12, [Kd]*12)
        device.SetDesiredJointPosition(q_ref)
        device.SetDesiredJointVelocity(v_ref)
        device.SetDesiredJointTorque([ff_ref] * 12)

        device.SendCommand(WaitEndOfCycle=True)

        if ((device.cpt % 100) == 0):
            device.Print()
            for i in range(12):
                print(device.hardware.GetMotor(i).GetPosition())
    #****************************************************************
    
    # Whatever happened we send 0 torques to the motors.
    device.SetDesiredJointTorque([0]*nb_motors)
    device.SendCommand(WaitEndOfCycle=True)

    if device.hardware.IsTimeout():
        print("Masterboard timeout detected.")
        print("Either the masterboard has been shut down or there has been a connection issue with the cable/wifi.")
    device.hardware.Stop()  # Shut down the interface between the computer and the master board

def main():
    parser = argparse.ArgumentParser(description='Example masterboard use in python.')
    parser.add_argument('-i',
                        '--interface',
                        required=True,
                        help='Name of the interface (use ifconfig in a terminal), for instance "enp1s0"')

    example_script(parser.parse_args().interface)


if __name__ == "__main__":
    main()

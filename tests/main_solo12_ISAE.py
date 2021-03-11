# coding: utf8
import numpy as np
import argparse
import math
from time import clock, sleep
from solo12_ISAE import Solo12


q_ref=np.zeros(12)
v_ref=np.zeros(12)
Kp = 0*0.1
Kd = 0*0.01


def example_script(name_interface):
    device = Solo12(name_interface,dt=0.001)
    nb_motors = device.nb_motors
    
    device.Init(calibrateEncoders=True, q_ref)


    #CONTROL LOOP ***************************************************
    while ((not device.hardware.IsTimeout()) and (clock() < 200)):
        device.UpdateMeasurment()

        q_err = q_ref - device.q_mes
        v_err = v_ref - device.v_mes
        tau = Kp*q_err + Kd*v_err
        device.SetDesiredJointTorque(tau)
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

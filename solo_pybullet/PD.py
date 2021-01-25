# coding: utf8

# Load modules
import numpy as np


def PD(qa_ref, qa_dot_ref, qa, qa_dot, dt, Kp=1, Kd=1, torques_sat=5*np.ones((12, 1)), torques_ref=np.zeros((12, 1))):
    # Output torques
    torques = Kp * (qa_ref - qa) + Kd * (qa_dot_ref - qa_dot) + torques_ref

    # Saturation to limit the maximal value that torques can have
    torques = np.maximum(np.minimum(torques, torques_sat), -torques_sat)

    if False:
        print("+------+-----------+-----------+-----------+")
        print("|  ID  | Reference |  Current  |   Torque  |")
        print("+------+-----------+-----------+-----------+")
        for i in range(len(qa_ref)):
            print("| j={0:2d} | ref={1:+3.2f} | cur={2:+3.2f} | tor={3:+3.2f} |".format(i, qa_ref[i][0],qa[i][0], torques[i][0]) )
        print("+------+-----------+-----------+-----------+\n")

    return torques

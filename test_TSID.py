import sys
sys.path.append('..')

import numpy as np
from numpy.linalg import norm
import matplotlib.pyplot as plt
import time
import pinocchio as pin
import tsid
#import gepetto.corbaserver
import subprocess
import os

from example_robot_data import loadSolo  # Functions to load the SOLO quadruped

# Load the robot for Pinocchio
solo12 = loadSolo(False)
path = "/opt/openrobots/share/example-robot-data/robots/solo_description/robots"
urdf = path + "/solo12.urdf"
vec = pin.StdVec_StdString()
vec.extend(item for item in path)
solo12_wrapper = tsid.RobotWrapper(urdf, vec, pin.JointModelFreeFlyer(), False)

formulation = tsid.InverseDynamicsFormulationAccForce("tsid", solo12_wrapper, False)
q0 = solo12.q0.copy()
v0 = np.zeros(solo12_wrapper.nv)
formulation.computeProblemData(0.0, q0, v0)

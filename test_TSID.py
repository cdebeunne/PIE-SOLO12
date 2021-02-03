import sys
sys.path.append('..')

import numpy as np
from numpy.linalg import norm
import matplotlib.pyplot as plt
import time
import pinocchio as pin
import tsid
import subprocess
import os
import solo_tsid 

from example_robot_data import loadSolo  # Functions to load the SOLO quadruped

# Load the robot for Pinocchio
solo12 = loadSolo(False)
solo12.initViewer(loadModel=True)

# initialize TSID

tsid = solo_tsid.solo_tsid()
tsid.setCOMRef(np.array([0.0,0.0,-0.05]).T, np.zeros(3), np.zeros(3))
tsid.setBaseRef(np.zeros(3))

# initalize trajectories

N_simu = 1000
tau    = np.full((tsid.solo12_wrapper.na, N_simu), np.nan)
q      = np.full((tsid.solo12_wrapper.nq, N_simu + 1), np.nan)
v      = np.full((tsid.solo12_wrapper.nv, N_simu + 1), np.nan)
dv     = np.full((tsid.solo12_wrapper.nv, N_simu + 1), np.nan)

# launch simu

t = 0.0
dt = 1e-3
q[:, 0], v[:, 0] = tsid.q0, tsid.v0

for i in range(N_simu):
    time_start = time.time()
	
    HQPData = tsid.formulation.computeProblemData(t, q[:,i], v[:,i])
    sol = tsid.solver.solve(HQPData)
    
    if sol.status != 0:
        print("Time %.3f QP problem could not be solved! Error code:" % t, sol.status)
        break
    
    tau[:,i] = tsid.formulation.getActuatorForces(sol)
    dv[:,i] = tsid.formulation.getAccelerations(sol)
    
    # numerical integration
    q[:,i + 1], v[:,i + 1] = tsid.integrate_dv(q[:,i], v[:,i], dv[:,i], dt)
    t += dt
    solo12.display(q[:,i])
    time.sleep(1e-3)

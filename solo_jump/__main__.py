from .TrajectoryGenerator import *
from .SecurityChecker import *
from example_robot_data import loadSolo  # Functions to load the SOLO quadruped

import numpy as np
import time

DISPLAY = False
SAVE = True
PLOT = True
file_name = 'trajectories/traj_7.npz'

# Create a instance of trajectory generator
traj_gen = TrajectoryGen_InvKin()

# Set a parameter of the generator
kwargs_trajec = {"traj_dx0":0, "traj_dx":0, "traj_dy":0, "traj_t0":1.5, "traj_t1":0.2, "traj_t2":0.2, "traj_z0":-0.31, "traj_dz":-0.18, "traj_zf":-0.25, "kps":[1.5, 4], "kds":[0.05, 0.1]}
traj_gen.setParameter('feet_traj_params', kwargs_trajec)

# Set multiple parameter of the genetor
kwargs_trajGen = {'debug':True, 'dt':0.01, 'init_reversed':True}
traj_gen.setParametersFromDict(**kwargs_trajGen)

# Display Informations about the generator
traj_gen.printInfo()

# Generate the trajectory
traj = traj_gen.generateTrajectory()

# Save the trajectory
if SAVE:
    traj.saveTrajectory(file_name)
    print("Trajectory saved as:", file_name)

# Plot the trajectory
if PLOT:
    traj.plotTrajectory(show_gains=True, show_all=True)

# Check security
secu = SecurityChecker()
secu.check_trajectory(traj)
secu.show_results(show_all=False)

# Display the generated trajectory in Gepetto-Gui
if DISPLAY:
    solo = loadSolo(False)
    solo.initViewer(loadModel=True)

    t0 = time.time()
    for i in range(traj.getSize()):
        q = np.concatenate((np.array([0, 0, 0.4, 0 ,0 ,0, 1]), traj.getElement('q', i)))
        solo.display(q)

        while(time.time()-t0<traj.getElement('t', i)):
            time.sleep(0.00001)

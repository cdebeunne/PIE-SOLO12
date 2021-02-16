from .TrajectoryGenerator import *
from .SecurityChecker import *
from example_robot_data import loadSolo  # Functions to load the SOLO quadruped

import numpy as np
import time

# Create a instance of trajectory generator
traj_gen = TrajectoryGen_InvKin()

# Set a parameter of the generator
kwargs_trajec = {"traj_dx0":0.05, "traj_t0":0.2, "traj_t1":0.25, "traj_z0":-0.05, "traj_zf":-0.25, "kps":[10, 2], "kds":[0.1, 0.08]}
traj_gen.setParameter('feet_traj_params', kwargs_trajec)

# Set multiple parameter of the genetor
kwargs_trajGen = {'debug':True, 'dt':0.01}
traj_gen.setParametersFromDict(**kwargs_trajGen)

# Display Informations about the generator
traj_gen.printInfo()

# Generate the trajectory
traj = traj_gen.generateTrajectory()

# Save the trajectory
traj.saveTrajectory('traj.npz')

# Plot the trajectory
traj.plotTrajectory(show_gains=True, show_all=True)

# Check security
secu = SecurityChecker()
secu.check_trajectory(traj)
secu.show_results(show_all=False)

# Display the generated trajectory in Gepetto-Gui
solo = loadSolo(False)
solo.initViewer(loadModel=True)

t0 = time.time()
for i in range(traj.getSize()):
    q = np.concatenate((np.array([0, 0, 0.4, 0 ,0 ,0, 1]), traj.getElement('q', i)))
    solo.display(q)

    while(time.time()-t0<traj.getElement('t', i)):
        time.sleep(0.00001)

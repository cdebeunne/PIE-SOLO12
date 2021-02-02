from __future__ import print_function

import os
import sys
import time

import example_robot_data
import numpy as np

import crocoddyl
import pinocchio
from crocoddyl.utils.quadruped import SimpleQuadrupedalGaitProblem, plotSolution

WITHDISPLAY = 'display' in sys.argv or 'CROCODDYL_DISPLAY' in os.environ
WITHPLOT = 'plot' in sys.argv or 'CROCODDYL_PLOT' in os.environ

# Loading the solo model
solo = example_robot_data.loadSolo()
robot_model = solo.model
lims = robot_model.effortLimit

# Setting up CoM problem
lfFoot, rfFoot, lhFoot, rhFoot = 'FL_FOOT', 'FR_FOOT', 'HL_FOOT', 'HR_FOOT'
gait = SimpleQuadrupedalGaitProblem(robot_model, lfFoot, rfFoot, lhFoot, rhFoot)

# Defining the initial state of the robot
q0 = robot_model.referenceConfigurations['standing'].copy()
v0 = pinocchio.utils.zero(robot_model.nv)
x0 = np.concatenate([q0, v0])

# Defining the CoM gait parameters
CoM_gait = {'comGoTo': 10, 'timeStep': 1e-2, 'numKnots': 25}

# Setting up the control-limited DDP solver
boxddp = crocoddyl.SolverBoxDDP(
    gait.createCoMGoalProblem(x0, CoM_gait['comGoTo'], CoM_gait['timeStep'],CoM_gait['numKnots']))

# Add the callback functions
print('*** SOLVE ***')
cameraTF = [2., 2.68, 0.84, 0.2, 0.62, 0.72, 0.22]
if WITHDISPLAY and WITHPLOT:
    display = crocoddyl.GepettoDisplay(solo, 4, 4, cameraTF, frameNames=[lfFoot, rfFoot, lhFoot, rhFoot])
    boxddp.setCallbacks([crocoddyl.CallbackLogger(), crocoddyl.CallbackVerbose(), crocoddyl.CallbackDisplay(display)])
elif WITHDISPLAY:
    display = crocoddyl.GepettoDisplay(solo, 4, 4, cameraTF, frameNames=[lfFoot, rfFoot, lhFoot, rhFoot])
    boxddp.setCallbacks([crocoddyl.CallbackVerbose(), crocoddyl.CallbackDisplay(display)])
elif WITHPLOT:
    boxddp.setCallbacks([
        crocoddyl.CallbackLogger(),
        crocoddyl.CallbackVerbose(),
    ])
else:
    boxddp.setCallbacks([crocoddyl.CallbackVerbose()])

xs = [robot_model.defaultState] * (boxddp.problem.T + 1)
us = boxddp.problem.quasiStatic([solo.model.defaultState] * boxddp.problem.T)

# Solve the DDP problem
boxddp.solve(xs, us, 100, False, 0.1)

# Plotting the entire motion
if WITHPLOT:
    # Plot control vs limits
    plotSolution(boxddp, bounds=True, figIndex=1, show=False)

    # Plot convergence
    log = boxddp.getCallbacks()[0]
    crocoddyl.plotConvergence(log.costs,
                              log.u_regs,
                              log.x_regs,
                              log.grads,
                              log.stops,
                              log.steps,
                              figIndex=3,
                              show=True)

# Display the entire motion
if WITHDISPLAY:
    display = crocoddyl.GepettoDisplay(solo, frameNames=[lfFoot, rfFoot, lhFoot, rhFoot])
    while True:
        display.displayFromSolver(boxddp)
        time.sleep(2.0)
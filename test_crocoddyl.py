from __future__ import print_function

import os
import sys
import time

import example_robot_data
import numpy as np

import crocoddyl
import pinocchio
from crocoddyl.utils.quadruped import SimpleQuadrupedalGaitProblem, plotSolution
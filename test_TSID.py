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
import TSID_traj

from example_robot_data import loadSolo  # Functions to load the SOLO quadruped

TSID_traj.TSID_traj(0.1, True)

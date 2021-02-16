import pybullet as p
import numpy as np

class PerformancesEvaluator:
    def __init__(self):
        #Maximum heigt reached by the Center of Mass of the robot
        self.maxHeightCoM = 0

    """
    Check the position of the CoM 
    and updates maxHeightCoM if it is at its max
    """
    def update_performance(self, q):
        CoMHeight = q[2]
        if CoMHeight > self.maxHeightCoM:
            self.maxHeightCoM = CoMHeight

    """
    Prints the performances of the jump
    """
    def show_results(self):
        print("Jump height : " + str(self.maxHeightCoM[0]) + "m")

# coding: utf8

import numpy as np  # Numpy library
import pybullet_data
from example_robot_data import loadSolo  # Functions to load the SOLO quadruped

import pybullet as p  # PyBullet simulator
import time

class SoloSimulation:
    def __init__(self, enableGepetto=False, enableGUI=False, enableGravity=True, dt=1e-3):
        self.solo = loadSolo(False)

        self.enableGepetto = enableGepetto
        if self.enableGepetto:
            self.solo.initDisplay(loadModel=True)

        # Start the client for PyBullet
        self.enableGUI = enableGUI
        if self.enableGUI:
            self.physicsClient = p.connect(p.GUI)
            p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
        else:
            self.physicsClient = p.connect(p.DIRECT)  # noqa
        
        # Set gravity (enabled by default)
        if enableGravity:
            p.setGravity(0, 0, -9.81)
        else:
            p.setGravity(0, 0, 0)
        
        # Load horizontal plane for PyBullet
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.loadURDF("plane.urdf")

        # Load the robot for PyBullet
        robotStartPos = [0, 0, 0.75]
        robotStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
        p.setAdditionalSearchPath("/opt/openrobots/share/example-robot-data/robots/solo_description/robots")
        self.robotId = p.loadURDF("solo12.urdf", robotStartPos, robotStartOrientation)

        # Set time step of the simulation
        self.dt = dt
        p.setTimeStep(self.dt)

        # Disable default motor control for revolute joints
        self.revoluteJointIndices = [0,1,2, 4,5,6, 8,9,10, 12,13,14]
        p.setJointMotorControlArray(self.robotId,
                                    jointIndices=self.revoluteJointIndices,
                                    controlMode=p.VELOCITY_CONTROL,
                                    targetVelocities=[0.0 for m in self.revoluteJointIndices],
                                    forces=[0.0 for m in self.revoluteJointIndices])
        
        # Enable torque control for revolute joints
        jointTorques = [0.0 for m in self.revoluteJointIndices]
        self.set_joint_torques(jointTorques)

        # Compute one step of simulation for initialization
        p.stepSimulation()
    
    """
    Sets the torques for the joints
    """
    def set_joint_torques(self, torques):
        if len(self.revoluteJointIndices) != torques.size:
            print("Wrong size of torques: {0}".format(torques.size))
        
        torques = torques.reshape((12,1))
        p.setJointMotorControlArray(self.robotId, self.revoluteJointIndices, controlMode=p.TORQUE_CONTROL, forces=torques)
    
    """
    Does one step of smulation with tempo if asked.
    """
    def step(self):
        p.stepSimulation()

        if self.enableGUI:
            # Display in gepettoGUI if requiered
            if self.enableGepetto:
                self.solo.display(self.get_q())

    """
    Returns true if the robot is touching the ground.
    """
    def is_on_ground(self):
        contacts = p.getContactPoints()
        return len(contacts) != 0

    """
    Returns q
    """
    def get_q(self):
        jointStates = p.getJointStates(self.robotId, self.revoluteJointIndices)  # State of all joints
        baseState = p.getBasePositionAndOrientation(self.robotId)  # Position of the free flying base

        # Reshaping data into q and qdot
        q = np.vstack((np.array([baseState[0]]).transpose(), np.array([baseState[1]]).transpose(),
                    np.array([[jointStates[i_joint][0] for i_joint in range(len(jointStates))]]).transpose()))

        return q

    """
    Returns qdot
    """
    def get_qdot(self):
        jointStates = p.getJointStates(self.robotId, self.revoluteJointIndices)  # State of all joints
        baseVel = p.getBaseVelocity(self.robotId)  # Velocity of the free flying base

        # Reshaping data into qdot
        qdot = np.vstack((np.array([baseVel[0]]).transpose(), np.array([baseVel[1]]).transpose(),
                        np.array([[jointStates[i_joint][1] for i_joint in range(len(jointStates))]]).transpose()))

        return qdot

    """
    Returns qa
    """
    def get_qa(self):
        jointStates = p.getJointStates(self.robotId, self.revoluteJointIndices)
        qa = np.array([[jointStates[i_joint][0] for i_joint in range(len(jointStates))]]).transpose()

        return qa
    
    """
    Returns qadot
    """
    def get_qadot(self):
        jointStates = p.getJointStates(self.robotId, self.revoluteJointIndices)
        qadot = np.array([[jointStates[i_joint][1] for i_joint in range(len(jointStates))]]).transpose()

        return qadot
    
    """
    Returns q and qdot
    """
    def get_state(self):
        return self.get_q(), self.get_qdot()

    """
    Returns qa and qadot
    """
    def get_state_a(self):
        return self.get_qa(), self.get_qadot()
    
    """
    Ends connection with pybullet.
    """
    def end(self):
        p.disconnect()
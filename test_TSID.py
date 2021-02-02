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

from example_robot_data import loadSolo  # Functions to load the SOLO quadruped

# Load the robot for Pinocchio
solo12 = loadSolo(False)
solo12.initViewer(loadModel=True)
path = "/opt/openrobots/share/example-robot-data/robots/solo_description/robots"
urdf = path + "/solo12.urdf"
srdf = "/opt/openrobots/share/example-robot-data/robots/solo_description/srdf/solo.srdf"
vec = pin.StdVec_StdString()
vec.extend(item for item in path)
solo12_wrapper = tsid.RobotWrapper(urdf, vec, pin.JointModelFreeFlyer(), False)
solo12_model = solo12_wrapper.model()
pin.loadReferenceConfigurations(solo12_model, srdf, False)

# get frame ID
ID_FL = solo12_model.getFrameId("FL_FOOT")
ID_FR = solo12_model.getFrameId("FR_FOOT")
ID_HL = solo12_model.getFrameId("HL_FOOT")
ID_HR = solo12_model.getFrameId("HR_FOOT")
ID_BASE = solo12_model.getFrameId("base_link")

# formulate the problem

formulation = tsid.InverseDynamicsFormulationAccForce("tsid", solo12_wrapper, False)
q0 = solo12_model.referenceConfigurations["standing"]
v0 = np.zeros(solo12_model.nv)
formulation.computeProblemData(0.0, q0, v0)
data = formulation.data()

# create the com task

kp_com = 10
w_com = 1
comTask = tsid.TaskComEquality("task-com", solo12_wrapper)
comTask.setKp(kp_com * np.ones(3).T)
comTask.setKd(2*np.sqrt(kp_com) * np.ones(3).T)
formulation.addMotionTask(comTask, w_com, 1, 0.0)
com_ref = solo12_wrapper.com(data)
trajCom = tsid.TrajectoryEuclidianConstant("traj-com", com_ref)

# create the SE3 task

kp_trunk = 50
w_trunk = 1
trunkTask = tsid.TaskSE3Equality("task-trunk", solo12_wrapper, 'base_link')
mask = np.matrix([1.0, 1.0, 0.0, 1.0, 1.0, 1.0]).T
trunkTask.setKp(np.matrix([0.0, 0.0, 0.0, kp_trunk, kp_trunk, 0.0]).T)
trunkTask.setKd(np.matrix([0.0, 0.0, 0.0, 2.0 * np.sqrt(kp_trunk), 2.0 * np.sqrt(kp_trunk), 0.0]).T)
trunkTask.useLocalFrame(False)
trunkTask.setMask(mask)
formulation.addMotionTask(trunkTask, w_trunk, 1, 0.0)
trunk_ref = solo12_wrapper.framePosition(data, ID_BASE)
trajTrunk = tsid.TrajectorySE3Constant("traj_base_link", trunk_ref)

# Create linear trajectory

N_simu = 1000
tau    = np.full((solo12_wrapper.na, N_simu), np.nan)
q      = np.full((solo12_wrapper.nq, N_simu + 1), np.nan)
v      = np.full((solo12_wrapper.nv, N_simu + 1), np.nan)
dv     = np.full((solo12_wrapper.nv, N_simu + 1), np.nan)
com_start = com_ref
com_end = com_ref - np.array([0,0,0.05])

sampleCom = trajCom.computeNext()
sampleCom.pos(com_end)
sampleCom.vel(np.zeros(3))
sampleCom.acc(np.zeros(3))
sampleTrunk = trajTrunk.computeNext()
sampleTrunk.pos(np.matrix([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]).T)
sampleTrunk.vel(np.matrix([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).T)
sampleTrunk.acc(np.matrix([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).T)
comTask.setReference(sampleCom)
trunkTask.setReference(sampleTrunk)

# create contact forces
contactNormal = np.array([0,0,1])
kp_contact = 30
mu = 0.5

contactFL = tsid.ContactPoint("contactFL", solo12_wrapper, "FL_FOOT", contactNormal, mu, 1, 1000)
contactFL.setKp(kp_contact * np.ones(6).T)
contactFL.setKd(2*np.sqrt(kp_contact) * np.ones(6).T)
fl_ref = solo12_wrapper.framePosition(data, ID_FL)
contactFL.setReference(fl_ref)
formulation.addRigidContact(contactFL, 10)

contactFR = tsid.ContactPoint("contactFR", solo12_wrapper, "FR_FOOT", contactNormal, mu, 1, 1000)
contactFR.setKp(kp_contact * np.ones(6).T)
contactFR.setKd(2*np.sqrt(kp_contact) * np.ones(6).T)
fr_ref = solo12_wrapper.framePosition(data, ID_FR)
contactFR.setReference(fr_ref)
formulation.addRigidContact(contactFR, 10)

contactHR = tsid.ContactPoint("contactHR", solo12_wrapper, "HR_FOOT", contactNormal, mu, 1, 1000)
contactHR.setKp(kp_contact * np.ones(6).T)
contactHR.setKd(2*np.sqrt(kp_contact) * np.ones(6).T)
hr_ref = solo12_wrapper.framePosition(data, ID_HR)
contactHR.setReference(hr_ref)
formulation.addRigidContact(contactHR, 10)

contactHL = tsid.ContactPoint("contactHL", solo12_wrapper, "HL_FOOT", contactNormal, mu, 1, 1000)
contactHL.setKp(kp_contact * np.ones(6).T)
contactHL.setKd(2*np.sqrt(kp_contact) * np.ones(6).T)
hl_ref = solo12_wrapper.framePosition(data, ID_HL)
contactHL.setReference(hl_ref)
formulation.addRigidContact(contactHL, 10)

# initialize solver

solver = tsid.SolverHQuadProgFast("qp solver")
solver.resize(formulation.nVar, formulation.nEq, formulation.nIn)

# launch simu

t = 0.0
dt = 1e-3
q[:, 0], v[:, 0] = q0, v0
p1 = False;

for i in range(N_simu):
    time_start = time.time()
    
    if i==500:
        kp_com = 100
        com_end = com_ref + np.array([0,0,0.05])
        sampleCom = trajCom.computeNext()
        sampleCom.pos(com_end)
        sampleCom.vel(np.matrix([0.0,0.0,1.0]).T)
        sampleCom.acc(np.zeros(3))
        comTask.setKp(kp_com * np.ones(3).T)
        comTask.setKd(2*np.sqrt(kp_com) * np.ones(3).T)
        comTask.setReference(sampleCom)

	
    HQPData = formulation.computeProblemData(t, q[:,i], v[:,i])
    sol = solver.solve(HQPData)
    
    if sol.status != 0:
        print("Time %.3f QP problem could not be solved! Error code:" % t, sol.status)
        break
    
    tau[:,i] = formulation.getActuatorForces(sol)
    dv[:,i] = formulation.getAccelerations(sol)
    
    # numerical integration
    v_mean = v[:,i] + 0.5*dt*dv[:,i]
    v[:,i + 1] = v[:,i] + dt * dv[:,i]
    q[:,i + 1] = pin.integrate(solo12_wrapper.model(), q[:,i], dt * v_mean)
    t += dt
    solo12.display(q[:,i])
    time.sleep(1e-3)

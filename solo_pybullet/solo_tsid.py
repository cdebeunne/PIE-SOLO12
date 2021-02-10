import os
import subprocess
import time

import numpy as np
import pinocchio as pin
import tsid


class SoloTSID:
    ''' Standard TSID formulation for a quadruped robot standing on its punctual feet.
        - Center of mass task
        - SE3 base task
        - contact force constraint for the feet
    '''
    def __init__(self):
		
		########################################################################
        #             Definition of the Model and TSID problem                 #
        ########################################################################
        
        # set the path
        path = "/opt/openrobots/share/example-robot-data/robots/solo_description/robots"
        urdf = path + "/solo12.urdf"
        srdf = "/opt/openrobots/share/example-robot-data/robots/solo_description/srdf/solo.srdf"
        vec = pin.StdVec_StdString()
        vec.extend(item for item in path)
		
		# get the solo12 wrapper
        self.solo12_wrapper = tsid.RobotWrapper(urdf, vec, pin.JointModelFreeFlyer(), False)
        self.solo12_model = self.solo12_wrapper.model()
        pin.loadReferenceConfigurations(self.solo12_model, srdf, False)
		
		# problem formulation
        self.formulation = tsid.InverseDynamicsFormulationAccForce("tsid", self.solo12_wrapper, False)
        self.q0 = self.solo12_model.referenceConfigurations["straight_standing"]
        self.v0 = np.zeros(self.solo12_model.nv)
        self.formulation.computeProblemData(0.0, self.q0, self.v0)
        self.data = self.formulation.data()
		
		# get frame ID
        self.ID_FL = self.solo12_model.getFrameId("FL_FOOT")
        self.ID_FR = self.solo12_model.getFrameId("FR_FOOT")
        self.ID_HL = self.solo12_model.getFrameId("HL_FOOT")
        self.ID_HR = self.solo12_model.getFrameId("HR_FOOT")
        self.ID_BASE = self.solo12_model.getFrameId("base_link")
		
		############
        # COM TASK #
        ############
        self.kp_com = 50
        self.w_com = 3
        self.comTask = tsid.TaskComEquality("task-com", self.solo12_wrapper)
        self.comTask.setKp(self.kp_com * np.ones(3).T)
        self.comTask.setKd(2*np.sqrt(self.kp_com) * np.ones(3).T)
        self.formulation.addMotionTask(self.comTask, self.w_com, 1, 0.0)
        self.com_ref = self.solo12_wrapper.com(self.data)
        self.trajCom = tsid.TrajectoryEuclidianConstant("traj-com", self.com_ref)
        
        #################
        # SE3 BASE TASK #
        #################
        self.kp_trunk = 50
        self.w_trunk = 1
        self.trunkTask = tsid.TaskSE3Equality("task-trunk", self.solo12_wrapper, 'base_link')
        mask = np.matrix([1.0, 1.0, 0.0, 1.0, 1.0, 1.0]).T
        self.trunkTask.setKp(np.matrix([self.kp_trunk, self.kp_trunk, 0.0, self.kp_trunk, self.kp_trunk, self.kp_trunk]).T)
        self.trunkTask.setKd(np.matrix([2.0 * np.sqrt(self.kp_trunk), 2.0 * np.sqrt(self.kp_trunk), 0.0, 2.0 * np.sqrt(self.kp_trunk), 2.0 * np.sqrt(self.kp_trunk), 2.0 * np.sqrt(self.kp_trunk)]).T)
        self.trunkTask.useLocalFrame(False)
        self.trunkTask.setMask(mask)
        self.formulation.addMotionTask(self.trunkTask, self.w_trunk, 1, 0.0)
        self.trunk_ref = self.solo12_wrapper.framePosition(self.data, self.ID_BASE)
        self.trajTrunk = tsid.TrajectorySE3Constant("traj_base_link", self.trunk_ref)
        
        ##################
        # CONTACT FORCES #
        ##################
        
        self.contactNormal = np.array([0,0,1])
        self.kp_contact = 30
        self.mu = 0.5

        self.contactFL = tsid.ContactPoint("contactFL", self.solo12_wrapper, "FL_FOOT", self.contactNormal, self.mu, 1, 1000)
        self.contactFL.setKp(self.kp_contact * np.ones(6).T)
        self.contactFL.setKd(2*np.sqrt(self.kp_contact) * np.ones(6).T)
        self.fl_ref = self.solo12_wrapper.framePosition(self.data, self.ID_FL)
        self.contactFL.setReference(self.fl_ref)
        self.formulation.addRigidContact(self.contactFL, 10)

        self.contactFR = tsid.ContactPoint("contactFR", self.solo12_wrapper, "FR_FOOT", self.contactNormal, self.mu, 1, 1000)
        self.contactFR.setKp(self.kp_contact * np.ones(6).T)
        self.contactFR.setKd(2*np.sqrt(self.kp_contact) * np.ones(6).T)
        self.fr_ref = self.solo12_wrapper.framePosition(self.data, self.ID_FR)
        self.contactFR.setReference(self.fr_ref)
        self.formulation.addRigidContact(self.contactFR, 10)

        self.contactHR = tsid.ContactPoint("contactHR", self.solo12_wrapper, "HR_FOOT", self.contactNormal, self.mu, 1, 1000)
        self.contactHR.setKp(self.kp_contact * np.ones(6).T)
        self.contactHR.setKd(2*np.sqrt(self.kp_contact) * np.ones(6).T)
        self.hr_ref = self.solo12_wrapper.framePosition(self.data, self.ID_HR)
        self.contactHR.setReference(self.hr_ref)
        self.formulation.addRigidContact(self.contactHR, 10)

        self.contactHL = tsid.ContactPoint("contactHL", self.solo12_wrapper, "HL_FOOT", self.contactNormal, self.mu, 1, 1000)
        self.contactHL.setKp(self.kp_contact * np.ones(6).T)
        self.contactHL.setKd(2*np.sqrt(self.kp_contact) * np.ones(6).T)
        self.hl_ref = self.solo12_wrapper.framePosition(self.data, self.ID_HL)
        self.contactHL.setReference(self.hl_ref)
        self.formulation.addRigidContact(self.contactHL, 10)
        
        ##########
        # SOLVER #
        ##########

        self.solver = tsid.SolverHQuadProgFast("qp solver")
        self.solver.resize(self.formulation.nVar, self.formulation.nEq, self.formulation.nIn)
    
    def getCOM(self):
        return self.solo12_wrapper.com(self.formulation.data())
        
    def setCOMRef(self, deltaPos, vel, acc):
        sampleCom = self.trajCom.computeNext()
        sampleCom.pos(self.com_ref + deltaPos)
        sampleCom.vel(vel)
        sampleCom.acc(acc)
        self.comTask.setReference(sampleCom)
        
    def setBaseRef(self):
        sampleTrunk = self.trajTrunk.computeNext()
        sampleTrunk.pos(np.matrix([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]).T)
        sampleTrunk.vel(np.matrix([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).T)
        sampleTrunk.acc(np.matrix([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).T)
        self.trunkTask.setReference(sampleTrunk)

    def integrate_dv(self, q, v, dv, dt):
        v_mean = v + 0.5 * dt * dv
        v += dt * dv
        q = pin.integrate(self.solo12_model, q, dt * v_mean)
        return q, v

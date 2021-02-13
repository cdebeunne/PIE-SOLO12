import pybullet as p
import numpy as np

class SecurityChecker:
    def __init__(self):
        # ID of frames allowed to have contact with ground
        self.frames_allowed = [3, 7, 11, 15]

        # Angle limits for actuators
        self.qa_lim = np.array([[-75, 140], [-180, 180], [0, 0]])
        self.qa_lim = np.deg2rad(self.qa_lim)

        # Speed limits for actuators
        self.qa_dot_lim = np.array([360, 720, 1080])
        self.qa_dot_lim = np.deg2rad(self.qa_dot_lim)

        # Torques limits for actuators
        self.torques_limit = np.array([3, 3, 3])

        # Security check elements
        self.flag_contact = False
        self.flag_limit = False
        self.flag_limit_dot = False
        self.flag_torque = False
        
        self.nb_contacts = 0
        self.minAngle = None
        self.maxAngle = None
        self.maxSpeed = np.zeros(12)
        self.maxTorques = np.zeros(12)

        # If true, prints informations while running
        self.verbose = False
    
    """
    Reset the security checker for another run.
    """
    def reset(self):
        self.flag_contact = False
        self.flag_limit = False
        self.flag_limit_dot = False
        self.flag_torque = False

        self.nb_contacts = 0
        self.minAngle = None
        self.maxAngle = None
        self.maxSpeed = np.zeros(12)
        self.maxTorques = np.zeros(12)
    
    """
    Checks if a contact occured between frames and ground.
    """
    def check_contact(self, solo):
        ret = False
        contacts = p.getContactPoints()

        for contact in contacts:
            # Get the one that is not ground
            if contact[1]==0:
                link = contact[4]
            elif contact[0]==0:
                link = contact[2]
            else:
                print("Something is wrong here")
            
            # Check if it is an allowed frame
            if not link in self.frames_allowed:
                if self.verbose:
                    print("Contact not allowed between {0} and ground ({1}).".format(link, self.frames_allowed))
                self.flag_contact = True
                self.nb_contacts += 1
                ret = True
        
        return ret
    
    """
    Checks if one of the joint went too far.
    :ret Returns true if a limit is overpassed
    """
    def check_limits(self, q):
        ret = False
        qa = q[7:].reshape((12))

        if self.maxAngle is None:
            self.maxAngle = qa
        if self.minAngle is None:
            self.minAngle = qa

        for leg in range(4):
            for art in range(3):
                q = qa[3*leg+art]

                # Store min value
                q_min = self.minAngle[3*leg+art]
                if q<q_min:
                    self.minAngle[3*leg+art] = q

                # Store max value
                q_max = self.maxAngle[3*leg+art]
                if q>q_max:
                    self.maxAngle[3*leg+art] = q
                
                # Check limit
                if (self.qa_lim[art, 0] and q<self.qa_lim[art, 0]) or (self.qa_lim[art, 1] and q>self.qa_lim[art, 1]):
                    if self.verbose:
                        print("Went to loin on leg {0} joint {1} (id={2}): {3}° ({4}°).".format(leg, i, 3*leg+art, np.rad2deg(q), np.rad2deg(self.qa_lim[art])))
                    self.flag_limit = True
                    ret = True
        
        return ret
    
    """
    Checks if one of the joint went too fast.
    :ret Returns true if a limit is overpassed
    """
    def check_speed(self, qdot):
        ret = False
        qa_dot = abs(qdot[6:].reshape((12)))

        for leg in range(4):
            for art in range(3):
                qdot = qa_dot[3*leg+art]

                # Update max speed
                qdot_max = self.maxSpeed[3*leg+art]
                if qdot > qdot_max:
                    self.maxSpeed[3*leg+art] = qdot
                
                # Check speed
                if (self.qa_dot_lim[art] and qdot>self.qa_dot_lim[art]):
                    if self.verbose:
                        print("Went to speed on leg {0} joint {1} (id={2}): {3} °/s ({4} °/s).".format(leg, art, 3*leg+art, np.rad2deg(qdot), np.rad2deg(self.qa_dot_lim[art])))
                    self.flag_limit_dot = True
                    ret = True
        
        return ret
    
    """
    Checks if one of the joint went too fast.
    :ret Returns true if a limit is overpassed
    """
    def check_torques(self, torques):
        ret = False
        torques = abs(torques.reshape((12)))

        # Checking limits
        for leg in range(4):
            for art in range(3):
                torque = torques[3*leg+art]

                # Update max torque
                torque_max = self.maxTorques[3*leg+art]
                if torque > torque_max:
                    self.maxTorques[3*leg+art] = torque
                
                if (self.torques_limit[art] and torque>self.torques_limit[art]):
                    if self.verbose:
                        print("Went to fort on leg {0} joint {1} (id={2}): {3} °/s ({4} °/s).".format(leg, art, 3*leg+art, torque, self.torques_limit[art]))
                    self.flag_torque = True
                    ret = True
        
        return ret
    
    """
    Checks everything.
    """
    def check_integrity(self, solo, q, qdot, torques):
        ret = False

        # Check contacts
        ret = True if self.check_contact(solo) else ret
        # Check angular limits
        ret = True if self.check_limits(q) else ret
        # Check speed limits
        ret = True if self.check_speed(qdot) else ret
        # Check torques limits
        ret = True if self.check_torques(torques) else ret
        
        return ret
    
    """
    Print results of the security check.
    """
    def show_results(self, show_all=False):
        print("\nSECURITY CHECK RESULTS:")

        # Contact Results
        print("\t-Contacts: ", "OCCURED" if self.flag_contact else "OK")
        if self.flag_contact:
            print("\t\t{0} contacts occured".format(self.nb_contacts))

        # Angular Limits Results
        if self.minAngle is not None:
            print("\t-Angular Limits: ", "OVERPASSED" if self.flag_limit else "OK")
            for leg in range(4):
                for art in range(3):
                    q_lim = np.rad2deg(self.qa_lim[art, :])
                    q_min = np.rad2deg(self.minAngle[3*leg+art])
                    q_max = np.rad2deg(self.maxAngle[3*leg+art])

                    is_bad = (q_lim[0] and q_min<q_lim[0]) or (q_lim[1] and q_max>q_lim[1])

                    if show_all or self.verbose or is_bad:
                        print("\t\t- Leg {0} joint {1} (id={2}):".format(leg, art, 3*leg+art))
                        
                        # Lower Bound
                        if q_lim[0]:
                            overpassed = "bad" if q_min<q_lim[0] else " ok"
                            print("\t\t\t-Lower: {0} ({1:+05.1f}°, {2:+05.1f}°)".format(overpassed, q_min, q_lim[0]))
                        else:
                            print("\t\t\t-Lower:  ok (not set)")
                        
                        # Upper Bound
                        if q_lim[1]:
                            overpassed = "bad" if q_max>q_lim[1] else " ok"

                            print("\t\t\t-Upper: {0} ({1:+05.1f}°, {2:+05.1f}°)".format(overpassed, q_max, q_lim[1]))
                        else:
                            print("\t\t\t-Upper:  ok (not set)")

        # Angular Speeds results
        print("\t-Angular Speeds: ", "OVERPASSED" if self.flag_limit_dot else "OK")
        for leg in range(4):
            for art in range(3):
                qdot = self.maxSpeed[3*leg+art]
                qdot_lim = self.qa_dot_lim[art]
                is_bad = (qdot_lim and abs(qdot)>qdot_lim)

                if show_all or self.verbose or is_bad:
                    overpassed = "bad" if is_bad else " ok"

                    print("\t\t- Leg {0} joint {1} (id={2:2d}): {3} ({4:.0f}°/s, {5:.0f}°/s).".format(leg, art, 3*leg+art, overpassed, np.rad2deg(qdot), np.rad2deg(qdot_lim)))

        # Torques results
        print("\t-Torques: ", "OVERPASSED" if self.flag_torque else "OK")
        for leg in range(4):
            for art in range(3):
                torque = self.maxTorques[3*leg+art]
                torque_lim = self.torques_limit[art]
                is_bad = (torque_lim and torque>torque_lim)

                if show_all or self.verbose or is_bad:
                    overpassed = "bad" if is_bad else " ok"

                    print("\t\t- Leg {0} joint {1} (id={2:2d}): {3} ({4:0.2f}N.m, {5:0.2f}N.m).".format(leg, art, 3*leg+art, overpassed, torque, torque_lim))

        print("")

        return self.flag_contact or self.flag_limit or self.flag_limit_dot or self.flag_torque


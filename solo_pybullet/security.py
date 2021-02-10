import pybullet as p
import numpy as np

def check_integrity(solo, q, q_dot):
    # Angle limits for actuators
    qa_lim = np.array([[-75, 140], [-180, 180], [0, 0]])
    qa_lim = np.deg2rad(qa_lim)
    # Speed limits for actuators
    qa_dot_lim = np.array([360, 720, 1080])
    qa_dot_lim = np.deg2rad(qa_dot_lim)
    
    contacts = p.getContactPoints()
    frames_allowed = [3, 7, 11, 15]

    qa = q[7:]
    qa_dot = q_dot[6:]

    # Checking contacts
    for contact in contacts:
        if contact[1]==0:
            link = contact[4]
        else:
            link = contact[2]
        
        if not link in frames_allowed:
            print("Contact not allowed between {0} and ground ({1}).".format(link, frames_allowed))
            check_integrity.flag_contact = True

    # Checking limits
    for leg in range(4):
        for i in range(3):
            q = qa[3*leg+i]
            q_max = check_integrity.maxAngle[3*leg+i]
            if q>q_max:
                check_integrity.maxAngle[3*leg+i] = q
            if (qa_lim[i, 0] and q<qa_lim[i, 0]) or (qa_lim[i, 1] and q>qa_lim[i, 1]):
                #print("Went to far on leg {0} joint {1} (id={2}): {3}° ({4}°).".format(leg, i, 3*leg+i, np.rad2deg(q), np.rad2deg(qa_lim[i])))
                check_integrity.flag_limit = True

            qdot = qa_dot[3*leg+i]
            qdot_max = check_integrity.maxSpeed[3*leg+i]
            if qdot > qdot_max:
                check_integrity.maxSpeed[3*leg+i] = qdot
            if (qa_dot_lim[i] and abs(qdot)>qa_dot_lim[i]):
                #print("Went to fast on leg {0} joint {1} (id={2}): {3} °/s ({4} °/s).".format(leg, i, 3*leg+i, np.rad2deg(qdot), np.rad2deg(qa_dot_lim[i])))
                check_integrity.flag_limit_dot = True
    
    if (check_integrity.flag_contact or check_integrity.flag_limit or check_integrity.flag_limit_dot):
        return True
    else:
        return False

check_integrity.flag_contact = False
check_integrity.flag_limit = False
check_integrity.flag_limit_dot = False
check_integrity.maxAngle = np.zeros(12)
check_integrity.maxSpeed = np.zeros(12)


def showIntegrity():
    # Angle limits for actuators
    qa_lim = np.array([[-75, 140], [-180, 180], [0, 0]])
    qa_lim = np.deg2rad(qa_lim)
    # Speed limits for actuators
    qa_dot_lim = np.array([360, 720, 1080])
    qa_dot_lim = np.deg2rad(qa_dot_lim)

    if check_integrity.flag_contact:
        print("A contact occured.")

    if check_integrity.flag_limit:
        print("Went too far occured.")
        for leg in range(4):
            for i in range(3):
                q = check_integrity.maxAngle[3*leg+i]
                if (qa_lim[i, 0] and q<qa_lim[i, 0]) or (qa_lim[i, 1] and q>qa_lim[i, 1]):
                    print("Went to far on leg {0} joint {1} (id={2}): {3}° ({4}°).".format(leg, i, 3*leg+i, np.rad2deg(q), np.rad2deg(qa_lim[i])))

    if check_integrity.flag_limit_dot:
        print("Joint was too speed:")
        for leg in range(4):
            for i in range(3):
                qdot = check_integrity.maxSpeed[3*leg+i]
                if (qa_dot_lim[i] and abs(qdot)>qa_dot_lim[i]):
                    print("  Went to fast on leg {0} joint {1} (id={2}): {3} °/s ({4} °/s).".format(leg, i, 3*leg+i, np.rad2deg(qdot), np.rad2deg(qa_dot_lim[i])))

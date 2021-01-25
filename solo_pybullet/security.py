import pybullet as p
import numpy as np

def check_integrity(solo, q, q_dot):
    if check_integrity.flag_contact or check_integrity.flag_limit:
        return True
    
    contacts = p.getContactPoints()
    frames_allowed = [3, 7, 11, 15]
    qa_lim = np.array([[-90, 90], [-175, 175], [0, 0]])
    qa_lim = np.deg2rad(qa_lim)
    qa = q[7:]

    for contact in contacts:
        if contact[1]==0:
            link = contact[4]
        else:
            link = contact[2]
        
        if not link in frames_allowed:
            print("Contact not allowed between {0} and ground ({1}).".format(link, frames_allowed))
            check_integrity.flag_contact = True

    for leg in range(4):
        for i in range(3):
            q = qa[3*leg+i]
            if (qa_lim[i, 0] and q<qa_lim[i, 0]) or (qa_lim[i, 1] and q>qa_lim[i, 1]):
                print("Went to far on leg {0} joint {1} (id={2}): {3} ({4}).".format(leg, i, 3*leg+i, q, qa_lim[i]))
                check_integrity.flag_limit = True
    
    return False

check_integrity.flag_contact = False
check_integrity.flag_limit = False
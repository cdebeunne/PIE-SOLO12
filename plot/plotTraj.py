from solo_pybullet.trajectory import *
import numpy as np
import matplotlib.pyplot as plt

dt = 0.0001
q_start = np.zeros((19,1))
t_crouch = 1.2
t_jump = 1.7
t_air = 3

qTraj, qdotTraj, gains = jumpTrajectory(q_start, t_crouch, t_jump, t_air, dt)
x = np.arange(0,t_air, dt)
plt.figure()
plt.plot(x, qTraj[1,:], 'g', x, qTraj[2,:], 'b')
plt.legend(['Shoulder', 'Knee'])
plt.axis([0, t_air, -5, 5])
plt.title('Cubic-spline trajectory of a front leg')
plt.show()

import numpy as np

class Logger:
    def __init__(self, size=1000):
        self.size = size
        self.current = 0

        self.t =            np.zeros(self.size)
        self.q =            np.zeros((self.size, 19))
        self.q_dot =        np.zeros((self.size, 18))
        self.qa_ref =       np.zeros((self.size, 12))
        self.qa_dot_ref =   np.zeros((self.size, 12))
        self.feedforward =  np.zeros((self.size, 12))
        self.torques =      np.zeros((self.size, 12))
        self.gains =        np.zeros((self.size, 2))
        
    def increase_size(self, extension_size=100):
        self.size += extension_size

        self.t =            np.resize(self.t,           self.size)
        self.q =            np.resize(self.q,           (self.size, 19))
        self.q_dot =        np.resize(self.q_dot,       (self.size, 18))
        self.qa_ref =       np.resize(self.qa_ref,      (self.size, 12))
        self.qa_dot_ref =   np.resize(self.qa_dot_ref,  (self.size, 12))
        self.feedforward =  np.resize(self.feedforward, (self.size, 12))
        self.torques =      np.resize(self.torques,     (self.size, 12))
        self.gains =        np.resize(self.gains,       (self.size, 2))

    def add_data(self, t, q, q_dot, qa_ref, qa_dot_ref, feedforward, torques, gains):
        self.t[self.current] =          t
        self.q[self.current] =          q.reshape(19)
        self.q_dot[self.current] =      q_dot.reshape(18)
        self.qa_ref[self.current] =     qa_ref
        self.qa_dot_ref[self.current] = qa_dot_ref
        self.feedforward[self.current] = feedforward
        self.torques[self.current] =    torques.reshape(12)
        self.gains[self.current] =      gains
        
        self.current += 1

        if self.current >= self.size:
            self.increase_size()
    
    def end(self):
        self.size = self.current
        self.increase_size(0)

        self.qa = self.q[:, 7:]
        self.qa_dot = self.q_dot[:, 6:]
    
    def plot_leg(self):
        import matplotlib.pyplot as plt

        fig, axes = plt.subplots(nrows=3, ncols=3, sharex=True, constrained_layout=True)
        fig.suptitle("Logger Leg Trajectory Recordings", fontsize=16)

        for i in range(3):
            axes[0, i].plot(self.t, np.rad2deg(self.qa[:, i]), label='real')
            axes[0, i].plot(self.t, np.rad2deg(self.qa_ref[:, i]), label='ref')
            axes[0, i].set_title("Position")
            axes[0, i].set_xlabel("Time (s)")
            axes[0, i].set_ylabel("Angular position (deg)")
            axes[0, i].grid()
            axes[0, i].legend()

            axes[1, i].plot(self.t, self.qa_dot[:, i], label='real')
            axes[1, i].plot(self.t, self.qa_dot_ref[:, i], label='ref')
            axes[1, i].set_title("Speed")
            axes[1, i].set_xlabel("Time (s)")
            axes[1, i].set_ylabel("Angular speed (rad/s)")
            axes[1, i].grid()
            axes[1, i].legend()

            axes[2, i].plot(self.t, self.torques[:, i], label='com')
            axes[2, i].plot(self.t, self.feedforward[:, i], label='ff')
            axes[2, i].set_title("Torques")
            axes[2, i].set_xlabel("Time (s)")
            axes[2, i].set_ylabel("Torque (N.m)")
            axes[2, i].grid()
            axes[2, i].legend()

        plt.show()

from matplotlib import pyplot as plt
import numpy as np

class Plotter:

    def __init__(self,logger):
        self.logger = logger

    def plot_position(self):
        plt.figure(figsize=(10,8))
        data = self.logger.get_numpy_data()
        qpos = data["qpos"]
        qdes = data["qdes"]
        timestamp = data["time"]
        dof = qpos.shape[1]
        for i in range(dof):
            if dof>1:
                plt.subplot(np.ceil(dof/2),2,i+1)
            plt.plot(timestamp,qpos[:,i],label="q_pos")
            plt.plot(timestamp,qdes[:,i],"--",label="q_des")
            plt.xlabel("time(s)")
            plt.ylabel(f"joint {i} angle (rad)")
            plt.legend()
            plt.grid()
        plt.show()
    
    def plot_velocity(self):
        plt.figure(figsize=(10,8))
        data = self.logger.get_numpy_data()
        qvel = data["qvel"]
        qvel_des = data["qvel_des"]
        timestamp = data["time"]
        dof = qvel.shape[1]
        for i in range(dof):
            if dof>1:
                plt.subplot(np.ceil(dof/2),2,i+1)
            plt.plot(timestamp,qvel[:,i],"--",label="qvel")
            plt.plot(timestamp,qvel_des[:,i],label="qvel_des")
            plt.xlabel("time(s)")
            plt.ylabel("joint angle velocity(rad/s)")
            plt.title(f"joint angle velocity")
            plt.legend()
            plt.grid()
        plt.show()
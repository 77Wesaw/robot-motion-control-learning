from matplotlib import pyplot as plt
import numpy as np

class Plotter:
    """
    绘图模块

    支持：
    - 七轴subplot
    - 单关节plot
    """
    def __init__(self,logger):
        self.logger = logger

    def plot_all(self):
        plt.figure(figsize=(10,8))
        data = self.logger.get_numpy_data()
        qpos = data["qpos"]
        q_des = data["q_des"]
        time = data["time"]
        for i in range(7):
            plt.subplot(4,2,i+1)
            plt.plot(time,qpos[:,i],label="q_pos")
            plt.plot(time,q_des[:,i],"--",label="q_des")
            plt.xlabel("time(s)")
            plt.ylabel(f"joint_angle {i} (rad)")
            plt.title(f"joint{i+1}")
            plt.legend()
            plt.grid()
        plt.show()
    
    def plot_single(self,joint_id):
        plt.figure(figsize=(8,10))
        data = self.logger.get_numpy_data()
        qpos = data["qpos"]
        qdes = data["qdes"]
        time = data["time"]
        plt.plot(time,qpos,label="qpos")
        plt.plot(time,qdes,"--",label="q_des")
        plt.xlabel("time(s)")
        plt.ylabel("joint_angle(rad)")
        plt.title(f"joint {joint_id} angle")
        plt.legend()
        plt.grid()
        plt.show()
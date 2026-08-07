from matplotlib import pyplot as plt

class Plotter:
    def __init__(self,logger):
        self.logger = logger
        self.data = logger.get_numpy_data()

    def plot_all(self):
        plt.figure(figsize=(10,8))
        for i in range(7):
            plt.subplot(4,2,i+1)
            time = self.data["time"]
            qpos = self.data["qpos"]
            qdes = self.data["qdes"]
            plt.plot(time,qpos[:,i],label="qpos")
            plt.plot(time,qdes[:,i],label="qdes")
            plt.xlabel("time(s)")
            plt.ylabel(f"joint {i+1} angle(rad)")
            plt.legend()
            plt.grid()
        plt.show()

    def plot_single(self,joint_id):
        plt.figure(figsize=(10,8))
        time = self.data["time"]
        qpos = self.data["qpos"]
        qdes = self.data["qdes"]
        plt.plot(time,qpos[:,joint_id],label="qpos")
        plt.plot(time,qdes[:,joint_id],label="qdes")
        plt.xlabel("time(s)")
        plt.ylabel(f"joint {joint_id} angle(rad)")
        plt.legend()
        plt.grid()
        plt.show()
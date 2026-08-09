import numpy as np

class DataLogger:
    def __init__(self):
        self.qpos = []
        self.qvel = []
        self.qdes = []
        self.qvel_des = []
        self.error = []
        self.ctrl = []
        self.timestamp = []

    def record(self,qpos,qvel,qdes,qvel_des,error,ctrl,timestamp):
        self.qpos.append(qpos.copy())
        self.qvel.append(qvel.copy())
        self.qdes.append(qdes.copy())
        self.qvel_des.append(qvel_des.copy())
        self.error.append(error.copy())
        self.ctrl.append(ctrl.copy())
        self.timestamp.append(timestamp)

    def get_numpy_data(self):
        return {
            "qpos":np.array(self.qpos),
            "qvel":np.array(self.qvel),
            "qdes":np.array(self.qdes),
            "qvel_des":np.array(self.qvel_des),
            "error":np.array(self.error),
            "ctrl":np.array(self.ctrl),
            "time":np.array(self.timestamp)
        }
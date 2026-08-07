import numpy as np
import time

class DataLogger:
    def __init__(self):
        self.qpos = []
        self.qvel = []
        self.qdes = []
        self.error = []
        self.ctrl = []
        self.time = []

    def record(self,qpos,qvel,qdes,error,ctrl,time):
        self.qpos.append(qpos.copy())
        self.qvel.append(qvel.copy())
        self.qdes.append(qdes.copy())
        self.error.append(error.copy())
        self.ctrl.append(ctrl.copy())
        self.time.append(time)

    def get_numpy_data(self):
        return {
            "qpos":np.array(self.qpos),
            "qvel":np.array(self.qvel),
            "qdes":np.array(self.qdes),
            "error":np.array(self.error),
            "ctrl":np.array(self.ctrl),
            "time":np.array(self.time)
        }
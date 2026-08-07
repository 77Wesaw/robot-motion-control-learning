
import numpy as np

class DataLogger():
    """
    数据记录器
    """
    def __init__(self):
        self.qpos = []
        self.qvel = []
        self.q_des = []
        self.error = []
        self.ctrl = []
        self.time = []

    def record(self,qpos,qvel,q_des,error,ctrl,time):
        self.qpos.append(qpos.copy())
        self.qvel.append(qvel.copy())
        self.q_des.append(q_des.copy())
        self.error.append(error.copy())
        self.ctrl.append(ctrl.copy())
        self.time.append(time)
    
    def get_numpy_data(self):
        """
        将list转换为numpy数组
        方便绘图
        """
        return {
                "qpos":np.array(self.qpos),
                "qvel":np.array(self.qvel),
                "q_des":np.array(self.q_des),
                "error":np.array(self.error),
                "ctrl":np.array(self.ctrl),
                "time":np.array(self.time)
        }
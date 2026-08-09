
import numpy as np

class PDController:
    def __init__(self,kp,kd):
        self.kp = np.atleast_1d(np.asarray(kp,dtype=float))
        self.kd = np.atleast_1d(np.asarray(kd,dtype=float))

    def compute(self,qpos,qvel,qdes,qvel_des):
        q_error = qdes - qpos
        qvel_error = qvel_des - qvel
        control = self.kp*q_error + self.kd*qvel_error
        return control,q_error
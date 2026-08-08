

class PDController:
    def __init__(self,kp,kd,):
        self.kp = kp
        self.kd = kd

    def compute(self,qpos,qvel,qdes):
        error = qdes - qpos
        control = self.kp*error - self.kd*qvel
        return control,error
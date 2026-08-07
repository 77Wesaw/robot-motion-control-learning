import numpy as np

class PDController:
    """
    关节空间PD控制器
    
    输入：
        qpos    当前关节角
        qvel    当前关节角速度
        q_des   期望关节角
    
    输出：
        control 控制量
    """

    def __init__(self,kp,kd):
        self.kp = np.array(kp)
        self.kd = np.array(kd)

    def compute(self,qpos,qvel,q_des):
        error = q_des - qpos
        control = (self.kp*error - self.kd*qvel)
        return control,error
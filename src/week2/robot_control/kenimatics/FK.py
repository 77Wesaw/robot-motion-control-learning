import mujoco
import numpy as np

class MujocoFK:
    def __init__(self,robot,ee_body_id = 8):
        """
        ee_body_id --- 机器人末端执行器的关节id，一般设为7
        """

        self.robot = robot
        self.ee_body_id = ee_body_id

    def compute(self,q=None):
        if q is not None:
            q = np.asarray(q,dtype=float).copy()
            self.robot.data.qpos[:7] = q
            mujoco.mj_forward(self.robot.model,self.robot.data)
            
        position = self.robot.data.xpos[self.ee_body_id].copy()
        rotation = self.robot.data.xmat[self.ee_body_id].reshape(3,3).copy()

        return position,rotation
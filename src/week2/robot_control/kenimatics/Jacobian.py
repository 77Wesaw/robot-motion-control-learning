import numpy as np
import mujoco

class MujocoJacobian:
    def __init__(self,robot,ee_body_id = 8):
        self.robot = robot
        self.ee_body_id = ee_body_id

    def compute(self,q = None):
        """
        Compute end-effector Jacobian


        Returns:

            J:
                (6,n)

                first 3 rows:
                    linear velocity

                last 3 rows:
                    angular velocity
        """
        model = self.robot.model
        data = self.robot.data
        
        if q is not None:
            q = np.asarray(q,dtype = float).copy()
            data.qpos[:7] = q
            mujoco.mj_forward(model,data)

        dof = model.nv

        J_pos = np.zeros((3,dof))
        J_rot = np.zeros((3,dof))

        mujoco.mj_jacBody(model,data,J_pos,J_rot,self.ee_body_id)

        J = np.concatenate((J_pos,J_rot))

        return J

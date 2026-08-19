import numpy as np

class JacobianIK:
    def __init__(self,fk,jacobian):
        self.fk = fk
        self.jacobian = jacobian

    def DLSSolve(self,target_position,target_rotation,q_init,damping = 0.01,max_iter = 100,torlerance=1e-3,step_size=1.0):
        target_position = np.asarray(target_position,dtype=float).copy()
        target_rotation = np.asarray(target_rotation,dtype=float).copy()
        q = np.asarray(q_init,dtype = float).copy()
        
        for i in range(max_iter):
            # 1.当前对应的末端位置
            # 2.位置误差
            # 3.判断是否收敛
            # 4.计算雅可比矩阵
            # 5.求伪逆
            # 6.计算关节修正量
            # 7.更新关节角

            current_position,current_rotation = self.fk(q)
            position_error = target_position - current_position
            rotation_error = self.orentation_error(current_rotation,target_rotation)
            if (np.linalg.norm(position_error) <= torlerance) and (np.linalg.norm(rotation_error) <= torlerance):
                break
            error = np.concatenate([position_error,rotation_error])
            # 阻尼最小二乘法
            J = self.jacobian(q)    #(6,7)
            A = J @ J.T + damping**2 * np.eye(J.shape[0])
            x = np.linalg.solve(A,error)
            dq = J.T @ x
            q += dq * step_size
        return q
    
    def NullSpaceSolve(self,target_position,target_rotation,q_init,q_ref,k_null,max_iter=100,torlerance=1e-3,step_size=1.0):
        q = np.asarray(q_init,dtype=float).copy()
        q_ref = np.asarray(q_ref,dtype = float).copy()
        target_position = np.asarray(target_position,dtype=float).copy()
        target_rotation = np.asarray(target_rotation,dtype=float).copy()
        for i in range(max_iter):
            current_position,current_rotation = self.fk(q)
            position_error = target_position - current_position
            rotation_error = self.orentation_error(current_rotation,target_rotation)
            error = np.concatenate([position_error,rotation_error])
            J = self.jacobian(q)
            J_pinv = np.linalg.pinv(J)
            dq_task = J_pinv @ error
            dq_null = k_null * (np.eye(J.T.shape[0]) - J_pinv @ J) @ (q_ref - q)
            task_flag = ( (np.linalg.norm(position_error)<=torlerance) and (np.linalg.norm(rotation_error)<=torlerance) )
            null_flag = ( np.linalg.norm(dq_null) <= torlerance )
            if task_flag and null_flag :
                break
            dq = (dq_task + dq_null) * step_size
            q += dq
        return q


    
    def orentation_error(self,R,Rd):
        """
        计算当前姿态到目标姿态的误差

        输入：
        R:当前末端姿态旋转矩阵:(3,3)
        Rd:期望末端姿态旋转矩阵：(3,3)
        """

        # 求姿态误差的旋转矩阵
        Re = Rd @ R.T

        # 从旋转矩阵中计算旋转角
        trace = np.trace(Re)
        cos_theta = (trace - 1)/2
        cos_theta = np.clip(cos_theta,-1,1)
        theta = np.arccos(cos_theta)

        if theta < 1e-6:
            return np.zeros(3)
        
        # 从旋转矩阵中计算旋转轴
        axis = np.array([
            Re[2,1] - Re[1,2],
            Re[0,2] - Re[2,0],
            Re[1,0] - Re[0,1]
        ]) / (2*np.sin(theta))

        return theta * axis



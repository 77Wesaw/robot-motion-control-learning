import numpy as np

class LinearTrajectory:
    """
    线性插值轨迹生成器

    q(t) = q_start + s*(q_goal - q_start)
    
    输入：
        起始关节角
        目标关节角
        运动时间
    """

    def __init__(self,q_start,q_goal,duration):
        self.q_start = np.array(q_start)
        self.q_goal = np.array(q_goal)
        self.duration = duration

    def get_position(self,t):
        """
        根据时间返回期望位置
        """
        if t>= self.duration:
            return self.q_goal
        else:
            q_des = self.q_start + t/self.duration*(self.q_goal-self.q_start)
            return q_des

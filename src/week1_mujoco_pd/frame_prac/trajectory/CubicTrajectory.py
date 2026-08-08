import numpy as np

class CubicTrajectory:
    """
    三次多项式轨迹
    q(t) = a0+a1*t+a2*t^2+a3*t^3
    """
    
    def __init__(self,qstart,qgoal,duration):
        self.qstart = qstart
        self.qgoal = qgoal
        self.duration = duration
        delta_q = self.qgoal - self.qstart
        self.a0 = qstart
        self.a1 = np.zeros_like(self.qstart)
        self.a2 = 3*delta_q/self.duration**2
        self.a3 = -2*delta_q/self.duration**3

    def get_position(self,t):
        if t>= self.duration:
            qdes = self.qgoal
        else:
            qdes = self.a0 + self.a1*t + self.a2*t**2 + self.a3*t**3
        return qdes
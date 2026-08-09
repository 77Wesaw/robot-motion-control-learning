import numpy as np

class CubicTrajectory:
    """
    三次多项式轨迹
    q(t) = a0+a1*t+a2*t^2+a3*t^3
    """
    
    def __init__(self,qstart,qgoal,duration):
        self.qstart = np.atleast_1d(np.asarray(qstart,dtype = float))
        self.qgoal = np.atleast_1d(np.asarray(qgoal,dtype = float))
        self.duration = float(duration)
        delta_q = self.qgoal - self.qstart
        self.a0 = self.qstart
        self.a1 = np.zeros_like(self.qstart)
        self.a2 = 3*delta_q/self.duration**2
        self.a3 = -2*delta_q/self.duration**3

    def get_position(self,t):
        if t<0:
            qdes = self.qstart
        elif t>= self.duration:
            qdes = self.qgoal
        else:
            qdes = self.a0 + self.a1*t + self.a2*t**2 + self.a3*t**3
        return qdes
    
    def get_velocity(self,t):
        if (t>= self.duration or t<0):
            qvel = np.zeros_like(self.qstart)
        else:
            qvel = self.a1 + 2*self.a2*t + 3*self.a3*t**2
        return qvel
    
    def get_acceleration(self,t):
        if (t >= self.duration or t<0):
            qacc = np.zeros_like(self.qstart)
        else:
            qacc = 2*self.a2 + 6*self.a3*t
        return qacc
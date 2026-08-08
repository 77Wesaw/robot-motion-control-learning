

class LinearTrajectory:
    def __init__(self,qstart,qgoal,duration):
        self.qstart = qstart
        self.qgoal = qgoal
        self.duration = duration

    def get_position(self,t):
        if t >= self.duration:
            return self.qgoal
        else:
            return self.qstart + t/self.duration * (self.qgoal - self.qstart)
    
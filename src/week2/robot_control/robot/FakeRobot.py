from robot_control.robot.RobotBase import RobotBase
import numpy as np

class FakeRobot(RobotBase):
    def __init__(self):
        self.qpos = np.zeros(7)
        self.qvel = np.zeros(7)
        self.control  =  np.zeros(7)
        self.time = 0.0

    def get_state(self):
        qpos = self.qpos.copy()
        qvel = self.qvel.copy()
        return qpos,qvel

    def reset(self):
        self.qpos = np.zeros(7)
        self.qvel = np.zeros(7)
        self.control  =  np.zeros(7)
        self.time = 0.0

    def send_control(self,control):
        self.control = control

    def step(self):
        """
        模拟积分，并非真实动力学
        """
        dt = 0.001
        self.qvel = self.control
        self.qpos += self.qvel*dt
        self.time += dt

    def get_time(self):
        return self.time
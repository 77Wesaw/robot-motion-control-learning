import numpy as np

class MujocoDynamics:
    def __init__(self,robot):
        self.robot = robot

    def get_gravity_torque(self):
        gravity = self.robot.data.qfrc_bias[:7].copy()
        return gravity
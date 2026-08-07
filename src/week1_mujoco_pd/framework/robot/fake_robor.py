import numpy as np
from framework.robot.robot_base import RobotBase

class FakeRobot(RobotBase):
    """
    模拟真实机器人接口
    
    只模拟：
    - 状态读取
    - 控制输入
    """

    def __init__(self):
        #当前关节角
        self.qpos = np.zeros(7)

        #当前关节速度
        self.qvel = np.zeros(7)

        #控制输入
        self.control = np.zeros(7)

        #模拟时间
        self.time = 0.0

    def reset(self):
        self.qpos = np.zeros(7)
        self.qvel = np.zeros(7)
        self.control = np.zeros(7)
        self.time = 0.0
    
    def get_state(self):
        """
        模拟读取机器人状态
        真实机器人这里可能是：
        robot.receive()
        """
        return (self.qpos.copy(),self.qvel.copy())
    
    def send_control(self, control):
        """
        模拟发送控制命令
        真实机器人这里可能是：
        send_joint_torque()
        """
        self.control = np.array(control)

    def step(self):
        """
        模拟真实机器人运行一步
        注意：
        只是简单积分
        不是动力学模型
        """
        dt = 0.001
        self.qvel = self.control
        self.qpos += self.qvel * dt
        self.time += dt
    
    def get_time(self):
        return self.time
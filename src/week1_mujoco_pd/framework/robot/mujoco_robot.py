import mujoco
from framework.robot.robot_base import RobotBase

class MujocoRobot(RobotBase):
    """
    mujoco仿真机器人实现

    负责：
    - 加载模型
    - 获取状态
    - 接收控制
    - 推进仿真
    """

    def __init__(self,xml_path):
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)

    def reset(self):
        """
        重置机器人状态
        """
        mujoco.mj_resetData(self.model,self.data)
        mujoco.mj_forward(self.model,self.data)

    def get_state(self):
        """
        获取机器人当前状态
        """
        qpos = self.data.qpos[:7].copy()
        qvel = self.data.qvel[:7].copy()
        return qpos,qvel
    
    def send_control(self,control):
        """
        发送控制量
        """
        self.data.ctrl[:7] = control
    
    def step(self):
        """
        仿真推进
        """
        mujoco.mj_step(self.model,self.data)

    def get_time(self):
        return self.data.time

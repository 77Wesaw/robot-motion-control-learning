import mujoco
from frame_prac.robot.RobotBase import RobotBase

class MujocoRobot(RobotBase):
    def  __init__(self,xml):
        self.model = mujoco.MjModel.from_xml_path(xml)
        self.data = mujoco.MjData(self.model)
        self.viewer = None

    def get_state(self):
        qpos = self.data.q[:7].copy()
        qvel= self.data.qvel[:7].copy()
        return qpos,qvel
    
    def reset(self):
        mujoco.mj_resetData(self.model,self.data)
        mujoco.forward(self.model,self.data)

    def send_control(self,control):
        self.data.ctrl[:7] = control

    def step(self):
        mujoco.step(self.model,self.data)

    def get_time(self):
        time = self.data.time
        return time
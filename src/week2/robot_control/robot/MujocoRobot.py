import mujoco
from robot_control.robot.RobotBase import RobotBase

class MujocoRobot(RobotBase):
    def  __init__(self,xml,joint_id=None):
        self.model = mujoco.MjModel.from_xml_path(xml)
        self.data = mujoco.MjData(self.model)
        self.joint_id = joint_id

    def get_state(self):
        if self.joint_id is None:
            qpos = self.data.qpos[:7].copy()
            qvel= self.data.qvel[:7].copy()
            return qpos,qvel
        else:
            qpos = self.data.qpos[self.joint_id : self.joint_id+1].copy()
            qvel= self.data.qvel[self.joint_id : self.joint_id+1].copy()
            return qpos,qvel
    
    def reset(self):
        mujoco.mj_resetData(self.model,self.data)
        mujoco.mj_forward(self.model,self.data)

    def send_control(self,control):
        if self.joint_id is None:
            self.data.ctrl[:7] = control
        else:
            self.data.ctrl[self.joint_id] = control[0]

    def step(self):
        mujoco.mj_step(self.model,self.data)

    def get_time(self):
        time = self.data.time
        return time
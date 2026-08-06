import mujoco
import mujoco.viewer
import time
import os
import matplotlib.pyplot as plt

class PDController():
    """
    PD控制器
    """
    def __init__(self,target,kp,kd):
        self.target = target
        self.kp = kp
        self.kd = kd
    
    def compute(self,q,vel):
        error = self.target - q
        ctrl = (self.kp * error - self.kd * vel)
        return ctrl
    
class DataLogger:
    """
    数据记录
    """
    def __init__(self):
        self.time = []
        self.q = []
        self.target = []
        self.error = []
        self.ctrl = []

    def record(self,time,q,target,error,ctrl):
        self.time.append(time)
        self.q.append(q)
        self.target.append(target)
        self.error.append(error)
        self.ctrl.append(ctrl)

class MujocoSimulator:
    """
    Mujoco部署仿真
    """
    def __init__(self,xml_path,controller,logger,render=False):
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)
        self.controller = controller
        self.logger = logger
        self.render = render
    
    def reset(self):
        mujoco.mj_resetDataKeyframe(self.model,self.data,0)
        mujoco.mj_forward(self.model,self.data)
    
    def step(self):
        q = self.data.qpos[0]
        qvel = self.data.qvel[0]
        ctrl = self.controller.compute(q,qvel)
        self.data.ctrl[0] = ctrl
        mujoco.mj_step(self.model,self.data)
        self.logger.record(self.data.time,q,self.controller.target,self.controller.target-q,ctrl)
    
    def run_with_viewer(self,sim_time):
        with mujoco.viewer.launch_passive(self.model,self.data) as viewer:
            viewer.cam.lookat[:] = [0,0,0.5]
            viewer.cam.distance = 2.5
            while (viewer.is_running() and self.data.time <= sim_time):
                self.step()
                viewer.sync()
                time.sleep(0.01)

    def run_without_viewer(self,sim_time):
        while self.data.time <= sim_time:
            self.step()
    
    def run(self,sim_time):
        if self.render:
            self.run_with_viewer(sim_time)
        else:
            self.run_without_viewer(sim_time)

class Plot:
    def __init__(self,logger):
        self.logger = logger

    def plot(self):
        plt.figure(figsize=(8,6))
        plt.plot(self.logger.time,self.logger.q,label="q_pos")
        plt.plot(self.logger.time,self.logger.target,"--")
        plt.xlabel("time(s)")
        plt.ylabel("q_pos(rad)")
        plt.legend()
        plt.grid()
        plt.show()

if __name__ == "__main__":
    
    current_dir = os.path.dirname(os.path.abspath(__file__))
    xml_path = os.path.join(current_dir,"../../models/franka_panda/franka_emika_panda/panda.xml")

    target = 2.0
    kp = 50
    kd = 5
    sim_time = 2.0
    controller = PDController(target=target,kp=kp,kd=kd)
    logger = DataLogger()
    plot = Plot(logger=logger)
    simulator = MujocoSimulator(xml_path=xml_path,controller=controller,logger=logger,render=True)

    simulator.reset()
    simulator.run(sim_time=sim_time)
    plot.plot()
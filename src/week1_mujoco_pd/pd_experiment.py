import mujoco
import mujoco.viewer
import time
import os
import matplotlib.pyplot as plt
import numpy as np

class PDController():
    """
    PD控制器
    """
    def __init__(self,target,kp,kd):
        self.target = np.array(target)
        self.kp = np.array(kp)
        self.kd = np.array(kd)
    
    def compute(self,q,vel):
        error = self.target - q
        ctrl = (self.kp * error - self.kd * vel)
        return ctrl,error
    
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
        self.q.append(q.copy())
        self.target.append(target.copy())
        self.error.append(error.copy())
        self.ctrl.append(ctrl.copy())

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
        q = self.data.qpos[:7].copy()
        qvel = self.data.qvel[:7].copy()
        ctrl,error = self.controller.compute(q,qvel)
        self.data.ctrl[:7] = ctrl
        mujoco.mj_step(self.model,self.data)
        self.logger.record(self.data.time,q,self.controller.target,error,ctrl)
    
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

class Plotter:
    def __init__(self,logger):
        self.logger = logger

    def plot(self):
        plt.figure(figsize=(10,8))
        q = np.array(self.logger.q)
        target = np.array(self.logger.target)
        for i in range(7):
            plt.subplot(4,2,i+1)
            plt.plot(self.logger.time,q[:,i],label="q_pos")
            plt.plot(self.logger.time,target[:,i],"--")
            plt.xlabel("time(s)")
            plt.ylabel("q_pos(rad)")
            plt.legend()
            plt.grid()
        plt.show()

if __name__ == "__main__":

    current_dir = os.path.dirname(os.path.abspath(__file__))
    xml_path = os.path.join(current_dir,"../../models/franka_panda/franka_emika_panda/panda.xml")

    target = np.array([0.5,-0.3,0.2,-1.2,0.3,1.0,0.0])
    kp = np.array([50,50,50,50,50,50,50])
    kd = np.array([5,5,5,5,5,5,5])
    sim_time = 2.0
    controller = PDController(target=target,kp=kp,kd=kd)
    logger = DataLogger()
    plot = Plotter(logger=logger)
    simulator = MujocoSimulator(xml_path=xml_path,controller=controller,logger=logger,render=True)

    simulator.reset()
    simulator.run(sim_time=sim_time)
    plot.plot()
import mujoco
import mujoco.viewer
import time
import os
import matplotlib.pyplot as plt
import numpy as np

class TrajectoryGenerator:
    """
    轨迹规划初始
    """
    def __init__(self,start,goal,duration):
        self.start = start
        self.goal = goal
        self.duration = duration
    
    def get_position(self,t):
        if t>= self.duration:
            return self.goal
        q_des = self.start + t/self.duration * (self.goal - self.start)
        return q_des

class PDController():
    """
    PD控制器
    """
    def __init__(self,kp,kd):
        # self.target = np.array(target)
        # self.kp = np.array(kp)
        # self.kd = np.array(kd)
        self.kp = kp
        self.kd = kd
    
    def compute(self,q,vel,q_des):
        error = q_des - q
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
        q = self.data.qpos[0].copy()
        qvel = self.data.qvel[0].copy()
        ctrl,error = self.controller.compute(q,qvel)
        self.data.ctrl[0] = ctrl
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
        # for i in range(7):
        #     plt.subplot(4,2,i+1)
        #     plt.plot(self.logger.time,q[:,i],label="q_pos")
        #     plt.plot(self.logger.time,target[:,i],"--")
        #     plt.xlabel("time(s)")
        #     plt.ylabel("q_pos(rad)")
        #     plt.legend()
        #     plt.grid()

        plt.plot(self.logger.time,q,label="q_pos")
        plt.plot(self.logger.time,target,"--")
        plt.xlabel("time(s)")
        plt.ylabel("q_pos(rad)")
        plt.legend()
        plt.grid()        
        plt.show()

if __name__ == "__main__":

    current_dir = os.path.dirname(os.path.abspath(__file__))
    xml_path = os.path.join(current_dir,"../../models/franka_panda/franka_emika_panda/panda.xml")

    # target = np.array([0.5,-0.3,0.2,-1.2,0.3,1.0,0.0])
    # kp = np.array([50,50,50,50,50,50,50])
    # kd = np.array([5,5,5,5,5,5,5])
    kp = 50
    kd = 5
    sim_time = 3.0
    target = 2.5
    start = 0.0
    traj = TrajectoryGenerator(start,target,sim_time)
    q_des = traj.get_position()
    controller = PDController(kp=kp,kd=kd)
    logger = DataLogger()
    plot = Plotter(logger=logger)
    simulator = MujocoSimulator(xml_path=xml_path,controller=controller,logger=logger,render=False)

    simulator.reset()
    simulator.run(sim_time=sim_time)
    plot.plot()
    
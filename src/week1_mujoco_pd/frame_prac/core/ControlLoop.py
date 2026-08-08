import time
import numpy as np

class ControlLoop():
    def __init__(self,trajectory,controller,robot,logger,frequency,viewer = None):
        self.trajectory = trajectory
        self.controller = controller
        self.robot = robot
        self.logger = logger
        self.frequency = frequency
        self.viewer = viewer
        
    def run(self,sim_time):
        self.robot.reset()
        start_time = self.robot.get_time()
        while self.robot.get_time()-start_time <= sim_time:
            qdes = np.array(self.trajectory.get_position(self.robot.get_time()))
            qpos,qvel= self.robot.get_state()
            control,error = self.controller.compute(qpos,qvel,qdes)
            self.robot.send_control(control)
            self.robot.step()
            if self.viewer:
                self.viewer.sync()
            self.logger.record(qpos,qvel,qdes,error,control,self.robot.get_time())
            time.sleep(1/self.frequency)
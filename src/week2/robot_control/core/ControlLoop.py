import time

class ControlLoop():
    def __init__(self,trajectory,dynamics,controller,robot,logger,frequency,viewer = None):
        self.trajectory = trajectory
        self.dynamics = dynamics
        self.controller = controller
        self.robot = robot
        self.logger = logger
        self.frequency = frequency
        self.viewer = viewer
        self.dt = 1/frequency
        
    def run(self,sim_time):
        self.robot.reset()
        start_time = self.robot.get_time()
        while self.robot.get_time()-start_time <= sim_time:
            current_time = self.robot.get_time()
            qdes = self.trajectory.get_position(current_time)
            qvel_des = self.trajectory.get_velocity(current_time)
            qpos,qvel= self.robot.get_state()
            print(self.dynamics.get_gravity_torque())
            control,error = self.controller.compute(qpos,qvel,qdes,qvel_des)
            self.robot.send_control(control)
            self.robot.step()
            if self.viewer:
                self.viewer.sync()
            self.logger.record(qpos,qvel,qdes,qvel_des,error,control,current_time)
            time.sleep(self.dt)
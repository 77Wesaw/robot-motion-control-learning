import time

class ControlLoop:
    """
    机器人控制循环

    负责连接：
    - Trajectory
            ↓
    - Controller
            ↓
    - Robot
            ↓
    - Logger
            
    """

    def __init__(self,robot,controller,trajectory,logger,frequency,viewer = None):
        self.robot = robot
        self.controller = controller
        self.trajectory = trajectory
        self.logger = logger
        self.dt = 1.0 / frequency
        self.viewer = viewer

    def  run(self,duration):
        """
        控制循环
        """
        self.robot.reset()
        start_time = self.robot.get_time()
        while self.robot.get_time() - start_time<= duration:
            # 1.获取机器人状态
            # 2.获取目标轨迹
            # 3.控制器计算
            # 4.发送控制
            # 5.推进机器人
            # 6.数据记录
            qpos,qvel = self.robot.get_state()
            q_des= self.trajectory.get_position(self.robot.get_time())
            control,error = self.controller.compute(qpos,qvel,q_des)
            self.robot.send_control(control)
            self.robot.step()
            if self.viewer:
                self.viewer.sync()
            self.logger.record(qpos,qvel,q_des,error,control,self.robot.get_time())
            time.sleep(self.dt)

        
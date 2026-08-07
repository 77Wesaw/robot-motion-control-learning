import mujoco.viewer

class MujocoViewer:
    """
    Mujoco可视化模块

    负责：
    - 打开Mujoco窗口
    - 同步画面
    """

    def __init__(self,robot):
        self.robot = robot
        self.viewer = None
    
    def launch(self):
        """
        打开iewer
        """
        self.viewer = mujoco.viewer.launch_passive(self.robot.model,self.robot.data)
        self.viewer.cam.lookat[:] = [0,0,0.5]
        self.viewer.cam.distance = 2.5
    
    def sync(self):
        """
        更新画面
        """
        if self.viewer:
            self.viewer.sync()
    
    def close(self):
        if self.viewer:
            self.viewer.close()
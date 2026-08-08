import mujoco.viewer

class MujocoViewer:
    def __init__(self,robot):
        self.robot = robot
        self.viewer = mujoco.viewer
    
    def launch(self):
        self.viewer = mujoco.viewer.launch_passive(self.robot.model,self.robot.data)
        self.viewer.cam.lookat[:] = [0,0,0.5]
        self.viewer.cam.distance = 2.5

    def sync(self):
            self.viewer.sync()
        
    def close(self):
            self.viewer.close()
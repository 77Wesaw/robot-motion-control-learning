

class MujocoFK:
    def __init__(self,robot,ee_body_id = 7):
        """
        ee_body_id --- 机器人末端执行器的关节id，一般设为7
        """

        self.robot = robot
        self.ee_body_id = ee_body_id

    def compute(self):
        position = self.robot.data.body_xpos(self.ee_body_id).copy()
        rotation = self.robot.data.body_xmat[self.ee_body_id].reshape(3,3).copy()

        return position
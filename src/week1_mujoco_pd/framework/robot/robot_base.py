from abc import ABC,abstractmethod

class RobotBase(ABC):
    """
    机器人抽象接口
    """
    @abstractmethod
    def get_state(self):
        """
        获取机器人状态
        """
        pass

    @abstractmethod
    def send_control(self,control):
        """
        发送控制指令
        """
        pass

    @abstractmethod
    def step(self):
        """
        推进一步
        """
        pass
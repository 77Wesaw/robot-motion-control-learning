from abc import ABC,abstractmethod

class RobotBase():
    @abstractmethod
    def get_state(self):
        pass

    @abstractmethod
    def reset(self):
        pass

    @abstractmethod
    def send_control(self):
        pass

    @abstractmethod
    def step():
        pass

    @abstractmethod
    def get_time(self):
        pass
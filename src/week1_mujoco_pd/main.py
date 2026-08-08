import os
import numpy as np

from frame_prac.core.ControlLoop import ControlLoop

# Trajectory
from frame_prac.trajectory.LinearTrajectory import LinearTrajectory

#controller
from frame_prac.controller.PDController import PDController

#robot
from frame_prac.robot.RobotBase import RobotBase
from frame_prac.robot.MujocoRobot import MujocoRobot

#simulation
from frame_prac.simulation.MujocoViewer import MujocoViewer

#utils
from frame_prac.utils.DataLogger import DataLogger
from frame_prac.utils.Plotter import Plotter

current_dir = os.path.dirname(os.path.abspath(__file__))
xml_path = os.path.join(current_dir,"../../models/franka_panda/franka_emika_panda/panda.xml")
sim_time = 2.0
qstart = np.array([0.0]*7)
qgoal = np.array([0.5]*7)
duration = 0.5
kp= np.array([50]*7)
kd = np.array([5]*7)
frequency = 1000

def main():
    robot = MujocoRobot(xml_path)
    viewer = MujocoViewer(robot)
    trajectory = LinearTrajectory(qstart,qgoal,duration)
    controller = PDController(kp,kd)
    logger = DataLogger()
    plotter = Plotter(logger)
    loop = ControlLoop(trajectory,controller,robot,logger,frequency,viewer)
    viewer.launch()
    loop.run(sim_time)
    viewer.close()
    plotter.plot_all()

if __name__ == "__main__":
    main()
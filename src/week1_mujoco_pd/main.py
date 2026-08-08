import os
import numpy as np

from frame_prac.core.ControlLoop import ControlLoop

# Trajectory
from frame_prac.trajectory.LinearTrajectory import LinearTrajectory
from frame_prac.trajectory.CubicTrajectory import CubicTrajectory

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

#单关节和多关节模式切换改这里
mode = "single"

if mode == "all":
    joint_id = None
    qstart = np.array([0.0]*7)
    qgoal = np.array([0.5]*7)
    kp= np.array([50]*7)
    kd = np.array([5]*7)

elif mode == "single":
    joint_id = 5
    qstart = 0.0
    qgoal = 0.5
    kp = 50
    kd = 5
    
current_dir = os.path.dirname(os.path.abspath(__file__))
xml_path = os.path.join(current_dir,"../../models/franka_panda/franka_emika_panda/panda_nohand.xml")
sim_time = 5.0
duration = 2.0
frequency = 1000


def main():
    robot = MujocoRobot(xml_path,joint_id)
    viewer = MujocoViewer(robot)
    # trajectory = LinearTrajectory(qstart,qgoal,duration)
    trajectory = CubicTrajectory(qstart,qgoal,duration)
    controller = PDController(kp,kd)
    logger = DataLogger()
    plotter = Plotter(logger)
    loop = ControlLoop(trajectory,controller,robot,logger,frequency,viewer)
    viewer.launch()
    loop.run(sim_time)
    viewer.close()
    if mode == "all":
        plotter.plot_all()
    elif mode == "single":
        plotter.plot_single(joint_id)

if __name__ == "__main__":
    main()
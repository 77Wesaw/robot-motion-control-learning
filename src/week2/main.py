import os
import numpy as np

from robot_control.core.ControlLoop import ControlLoop

# Trajectory
from robot_control.trajectory.LinearTrajectory import LinearTrajectory
from robot_control.trajectory.CubicTrajectory import CubicTrajectory

#controller
from robot_control.controller.PDController import PDController

#Dynamics
from robot_control.dynamics.MujocoDynamics import MujocoDynamics

#robot
from robot_control.robot.RobotBase import RobotBase
from robot_control.robot.MujocoRobot import MujocoRobot

#simulation
from robot_control.simulation.MujocoViewer import MujocoViewer

#utils
from robot_control.utils.DataLogger import DataLogger
from robot_control.utils.Plotter import Plotter

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
    dynamics = MujocoDynamics(robot)
    logger = DataLogger()
    plotter = Plotter(logger)
    loop = ControlLoop(trajectory,dynamics,controller,robot,logger,frequency,viewer)
    viewer.launch()
    loop.run(sim_time)
    viewer.close()
    plotter.plot_position()
    plotter.plot_velocity()

if __name__ == "__main__":
    main()
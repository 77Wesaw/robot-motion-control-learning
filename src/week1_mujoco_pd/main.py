import os
import numpy as np
import time

# trajectory
from framework.trajectory.JointTrajectory import LinearTrajectory

# controllerr
from framework.controller.pd_controller import PDController

# robot
from framework.robot.mujoco_robot import MujocoRobot

# utils
from framework.utils.DataLogger import DataLogger
from framework.utils.Plotter import Plotter

# simulator
from framework.simulation.mujoco_viewer import MujocoViewer

# core
from framework.core.control_loop import ControlLoop

def main():
    current_dir = os.path.dirname(os.path.abspath(__file__))
    xml = os.path.join(current_dir,"../../models/franka_panda/franka_emika_panda/panda.xml")
    
    robot = MujocoRobot(xml)
    controller = PDController(kp=np.array([50]*7),kd=np.array([5]*7))
    viewer= MujocoViewer(robot)
    viewer.launch()
    q_start = np.zeros(7)
    q_goal = np.array([0.5,-0.3,0.2,-1.2,0.3,1.0,0.0])
    trajectory = LinearTrajectory(q_start,q_goal,duration=3.0)
    logger = DataLogger()

    loop = ControlLoop(robot,controller,trajectory,logger,1000,viewer)
    start = time.time()
    loop.run(3.0)
    viewer.close()
    end = time.time()
    print(f"simulation cost: {end-start:.2f} s")

    plotter = Plotter(logger)
    plotter.plot_all()

if __name__ == "__main__":
    main()
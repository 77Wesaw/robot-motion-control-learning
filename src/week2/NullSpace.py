import numpy as np
import os
import time

from robot_control.robot.MujocoRobot import MujocoRobot
from robot_control.kenimatics.IK import JacobianIK
from robot_control.kenimatics.Jacobian import MujocoJacobian
from robot_control.kenimatics.FK import MujocoFK
from robot_control.simulation.MujocoViewer import MujocoViewer


current_path = os.path.dirname(os.path.abspath(__file__))
xml = os.path.join(current_path, "../../models/franka_panda/franka_emika_panda/panda_nohand.xml")

def main():
    robot = MujocoRobot(xml)
    fk = MujocoFK(robot)
    jacobian = MujocoJacobian(robot)
    viewer = MujocoViewer(robot)
    ik = JacobianIK(fk.compute, jacobian.compute)
    q_init = np.array([0.0, -0.2, 0.5, -1.0, 0.3, 0.8, 0.0])
    target_position, target_rotation = fk.compute(q_init)
    J = jacobian.compute(q_init)
    # svd分解求解零空间方向
    _, _, Vt = np.linalg.svd(J)
    null_direction = Vt[-1]
    q = q_init.copy()
    t = 0.0
    viewer.launch()
    for i in range(400):
        q_ref = q_init + 10*np.sin(t)*null_direction
        q = ik.NullSpaceSolve(target_position, target_rotation, q, q_ref,k_null=1.0, max_iter=1, torlerance=1e-8, step_size=0.05)
        robot.data.qpos[:7] = q
        viewer.sync()
        time.sleep(0.03)
        t += 0.03
    viewer.close()

if __name__ == "__main__":
    main()
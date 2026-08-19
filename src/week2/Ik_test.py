import numpy as np
import os
from robot_control.robot.MujocoRobot import MujocoRobot
from robot_control.kenimatics.IK import JacobianIK
from robot_control.kenimatics.Jacobian import MujocoJacobian
from robot_control.kenimatics.FK import MujocoFK


current_path = os.path.dirname(os.path.abspath(__file__))
xml = os.path.join(current_path,"../../models/franka_panda/franka_emika_panda/panda_nohand.xml")

def main():
    robot = MujocoRobot(xml)
    mujocoFk = MujocoFK(robot)
    mujocoJacobian = MujocoJacobian(robot)

    q_target = np.array([
    0.2,
    -0.4,
    0.3,
    -1.2,
    0.2,
    1.0,
    0.4
    ])
    target_position,target_rotation = mujocoFk.compute(q_target)
    q_init = np.array([
    0.0,
    -0.2,
    0.1,
    -1.0,
    0.0,
    0.8,
    0.0
])
    ik = JacobianIK(mujocoFk.compute,mujocoJacobian.compute)
    q_solution = ik.DLSSolve(target_position,target_rotation,q_init,step_size=1)
    print(f"q_solution: {q_solution}")
    position_solution,rotation_solution = mujocoFk.compute(q_solution)
    rotation_error = ik.orentation_error(rotation_solution,target_rotation)
    print(f"position_error: {np.linalg.norm(target_position-position_solution)}")
    print(f"rotation_error: {np.linalg.norm(rotation_error)}")

if __name__ == "__main__":
    main()
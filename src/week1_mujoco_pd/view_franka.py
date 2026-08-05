import mujoco
import mujoco.viewer
import time
import os

#Franka Panda路径
current_dir = os.path.dirname(os.path.abspath(__file__))
xml_path = os.path.join(
    current_dir,
    "../../models/franka_panda/franka_emika_panda/panda.xml")

#加载模型
model = mujoco.MjModel.from_xml_path(xml_path)

#创建仿真数据
data = mujoco.MjData(model)


print("模型加载成功！")
print("关节数量：",model.njnt)

with mujoco.viewer.launch_passive(model,data) as viewer:

    viewer.cam.azimuth = 120
    viewer.cam.elevation = -20
    viewer.cam.distance = 2
    viewer.cam.lookat[:] = [0,0,0.5]

    while viewer.is_running():
        mujoco.mj_step(model,data)
        time.sleep(0.01)
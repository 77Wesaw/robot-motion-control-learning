import mujoco
import mujoco.viewer
import time

xml = """
<mujoco>
    <visual>
        <global offwidth="800" offheight="600"/>
    </visual>

    <asset>
        <material name="ground_mat"
                  rgba="0.5 0.5 0.5 1"/>
        <material name="box_mat"
                  rgba="0.1 0.5 0.9 1"/>
    </asset>

    <worldbody>

        <!-- 光源 -->
        <light name="top" pos="0 0 5" dir="0 0 -1" directional="true"/>

        <!-- 相机 -->
        <camera name="cam"
                mode="fixed"
                pos="90 90 60"
                xyaxes="1 0 0 0 0 1"/>

        <geom type="plane" size="2 5 0.1" material="ground_mat"/>

        <body name="box" pos="0 0 1">
            <freejoint/>
            <geom type="box" size="0.2 0.2 0.2" material="box_mat"/>
        </body>

    </worldbody>
</mujoco>
"""

xml2 = """
<mujoco>

    <worldbody>
        <geom type="box" size="1 1 1" pos="1 1 1 "/>
        <body name="name" pos="1 1 1"/>

    </worldbody>

</mujoco>
"""

model = mujoco.MjModel.from_xml_string(xml2)
data = mujoco.MjData(model)

with mujoco.viewer.launch_passive(model,data) as viewer:
    while viewer.is_running():
        mujoco.mj_step(model,data)
        viewer.sync()
        time.sleep(0.01)
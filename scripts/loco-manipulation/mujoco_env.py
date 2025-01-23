import mujoco
import os

model_path = '../../widowGo1/urdf/widowGo1.xml'
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

print(data.qpos)
mujoco.mj_step(model, data)
print(data.qpos)
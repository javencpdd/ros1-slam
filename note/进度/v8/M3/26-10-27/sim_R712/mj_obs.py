import numpy as np, mujoco, json
XML='''<mujoco><worldbody>
<geom name="ground" type="box" pos="0 0 -0.05" size="2 2 0.05" rgba=".5 .5 .5 1"/>
<geom name="step1" type="box" pos="2.25 0 0.13" size="0.75 0.6 0.05" rgba=".7 .5 .3 1"/>
<geom name="landing" type="box" pos="3.25 0 0.31" size="0.75 0.6 0.05" rgba=".3 .6 .3 1"/>
<camera name="top" pos="1.6 0 3" euler="0 0 0" fovy="60"/>
</worldbody></mujoco>'''
m=mujoco.MjModel.from_xml_string(XML); d=mujoco.MjData(m); mujoco.mj_forward(m,d)
r=mujoco.Renderer(m, 240, 320); r.enable_depth_rendering(); r.update_scene(d, camera='top')
dep=r.render().copy()
v=dep[np.isfinite(dep)]
print("DEPTH_MINMAX", round(float(v.min()),3), round(float(v.max()),3))
u,c=np.unique(np.round(v,2), return_counts=True)
idx=np.argsort(-c)[:6]
print("TOPDEPTH", [(float(u[i]), int(c[i])) for i in idx])

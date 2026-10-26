import numpy as np, mujoco, json, math, sys
sys.path.insert(0,'/work/repo/note/计划书v8/planner'); sys.path.insert(0,'/work/repo/note/计划书v8/contracts')
import validate, support_planner
XML='''<mujoco><worldbody>
<geom name="ground" type="box" pos="0 0 -0.05" size="2 2 0.05"/>
<geom name="step1" type="box" pos="2.25 0 0.13" size="0.75 0.6 0.05"/>
<geom name="landing" type="box" pos="3.25 0 0.31" size="0.75 0.6 0.05"/>
<camera name="top" pos="1.6 0 3" euler="0 0 0" fovy="60"/>
</worldbody></mujoco>'''
m=mujoco.MjModel.from_xml_string(XML); d=mujoco.MjData(m); mujoco.mj_forward(m,d)
r=mujoco.Renderer(m, 240, 320); r.enable_depth_rendering(); r.update_scene(d, camera='top'); dep=r.render().copy()
CAM=(1.6,0.0,3.0); H,W=dep.shape
fy=H/(2*math.tan(math.radians(30))); fx=fy; cx=W/2; cy=H/2
us,vs=np.meshgrid(np.arange(W),np.arange(H))
ok=np.isfinite(dep)&(dep>0.1)&(dep<10)
xs=CAM[0]+(us-cx)*dep/fx; ys=CAM[1]-(vs-cy)*dep/fy; zs=CAM[2]-dep
sel=ok&(np.abs(xs)<5)&(np.abs(ys)<3)&(zs>-0.2)&(zs<1.0)
X,Y,Z=xs[sel],ys[sel],zs[sel]
bands={}
for x,y,z in zip(X,Y,Z): bands.setdefault(round(float(z),2),[]).append((float(x),float(y)))
pick=[(z,v) for z,v in sorted(bands.items(), key=lambda kv:-len(kv[1]))[:3]]
pick.sort(key=lambda t:t[0])
print("BANDS", [(z,len(v)) for z,v in pick])
surfaces=[]
for i,(z,xy) in enumerate(pick):
    ax=[p[0] for p in xy]; ay=[p[1] for p in xy]
    surfaces.append({"surface_id":"obs_%d"%i,"z_reference":float(z),"thickness_m":0.1,
      "representation":"elevation_grid","data_uri":"observation:mujoco-depth",
      "traversability":"supported","clearance_min_m":0.4,
      "footprint":{"min_x":round(min(ax),2),"min_y":round(min(ay),2),"max_x":round(max(ax),2),"max_y":round(max(ay),2)},
      "edges":[]})
for i in range(len(surfaces)-1):
    dlt=abs(surfaces[i+1]["z_reference"]-surfaces[i]["z_reference"])
    e={"to":surfaces[i+1]["surface_id"],"kind":"step","max_step_height_m":round(dlt+0.02,2),"bidirectional":True}
    surfaces[i]["edges"].append(e)
    surfaces[i+1]["edges"].append({"to":surfaces[i]["surface_id"],"kind":"step","max_step_height_m":e["max_step_height_m"],"bidirectional":True})
value={"schema_version":1,"map_id":"stairs_obs","map_version":1,"frame":"map","kind":"support_map",
 "resolution":0.1,"origin":[-2.0,-2.0,0.0],"extent":{"width":80,"height":40},
 "cell_states":["free","occupied","unknown"],"unknown_policy":"unknown_not_traversable",
 "multiple_surfaces_per_xy":True,
 "source":{"uri":"observation:mujoco-depth","sha256":"%064x"%int(abs(np.round(dep[np.isfinite(dep)].sum()*1000)))},
 "surfaces":surfaces}
validate.validate("map", value)
route=support_planner.plan(value,(0.0,0.0,0.0),(3.5,0.0,0.36))
print(json.dumps({"surfaces":[s["surface_id"] for s in value["surfaces"]],
 "z":[s["z_reference"] for s in value["surfaces"]],
 "points":len(route["path"]),"segments":len(route["segments"]),"transitions":len(route["transitions"])}))
open('/work/obs_map.json','w').write(json.dumps(value,indent=1))

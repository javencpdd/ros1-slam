import json, math
import numpy as np
from scipy.spatial import cKDTree

def read_pcd(path):
    f=open(path,'rb')
    while True:
        line=f.readline().decode('ascii','ignore').strip()
        if line.startswith('FIELDS'): fields=line.split()[1:]
        if line.startswith('POINTS'): n=int(line.split()[1])
        if line.startswith('DATA'): break
    dt=np.dtype([(name,'<f4') for name in fields])
    a=np.frombuffer(f.read(n*dt.itemsize), dtype=dt, count=n)
    return np.stack([a['x'],a['y'],a['z']],axis=1).astype(np.float64)

P=math.radians(15); C,S=math.cos(P),math.sin(P)
R=np.array([[C,0,S],[0,1,0],[-S,0,C]]); T=np.array([0,0,0.34])
mp=read_pcd('/work/maps/m1_world_synth_v1.pcd'); tree=cKDTree(mp)
data=json.load(open('/work/map_audit_scans.json'))
for item in data:
    x,y,yaw=item['pose']
    Rz=np.array([[math.cos(yaw),-math.sin(yaw),0],[math.sin(yaw),math.cos(yaw),0],[0,0,1]])
    ds=[]
    for s in item['scans']:
        p=np.array(s)
        if p.shape[0]<200: continue
        pw=(p@R.T+T)@Rz.T+np.array([x,y,0.11])
        d,_=tree.query(pw,k=1); ds.append(d)
    d=np.concatenate(ds)
    print('pose',item['pose'],'n',d.size,'p50 %.4f p95 %.4f max %.4f'%(np.percentile(d,50),np.percentile(d,95),d.max()))
# 自由空间假占据：走廊中线 z=0.5
pts=np.array([[xx,0.0,0.5] for xx in np.arange(0,4.21,0.1)])
d,_=tree.query(pts,k=1)
print('free-space corridor z=0.5: min %.4f p50 %.4f frac<0.2 %.3f'%(d.min(),np.percentile(d,50),(d<0.2).mean()))

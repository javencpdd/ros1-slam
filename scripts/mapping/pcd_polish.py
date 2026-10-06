#!/usr/bin/env python3
# pcd_polish.py — PCD 抛光三件套：①z 轴裁剪（去地板/房顶，实验性多层楼梯检测）②自适应体素降采样 ③SOR 离群去除
# 参数（KEY=VAL 形式，均可缺省）：
#   ZMIN/ZMAX   显式 z 保留区间（设置后跳过自动楼层检测）
#   FLOOR_KEEP  地板上方保留起点（默认 0.10m——刮掉地面层）
#   ROOF_TRIM   房顶下方裁剪量（默认 0.15m——刮掉天花板层）
#   FLOORS      auto=自动检测多层楼梯（按 z 密度峰，实验性）；缺省=单层自动
#   STORY       层高（多层模式顶层无上一峰时用，默认 3.2m）
#   MIN_FLOOR_GAP 最小层间距（默认 2.5m，小于它的峰算同一层）
#   VOXEL       显式体素（缺省=自适应：从 0.05 起步直到点数≤TARGET）
#   TARGET      自适应目标点数（默认 400000）
#   SOR_MIN     SOR 邻居数阈值（默认 5；0=关闭 SOR）
#   SOR_RADIUS  SOR 邻域半径（默认 0.25m）
import sys, os, re, numpy as np

def load(path):
    raw=open(path,"rb").read()
    he=raw.find(b"DATA binary\n")+len(b"DATA binary\n")
    hdr=raw[:he].decode(errors="replace")
    n=int(re.search(r"POINTS (\d+)",hdr).group(1))
    m=re.search(r"POINT_STEP (\d+)",hdr)
    if m: step=int(m.group(1))
    else:
        sizes=[int(s) for s in re.search(r"SIZE (.+)",hdr).group(1).split()]
        cs=re.search(r"COUNT (.+)",hdr)
        counts=[int(c) for c in cs.group(1).split()] if cs else [1]*len(sizes)
        step=sum(s*c for s,c in zip(sizes,counts))
    arr=np.frombuffer(raw[he:he+n*step],dtype=np.uint8).reshape(n,step)
    return arr,step

def save(path,arr):
    N=len(arr)
    hdr=(f"VERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n"
         f"WIDTH {N}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {N}\nDATA binary\n")
    open(path,"wb").write(hdr.encode()+arr[:,:16].tobytes())

def xyz(arr,step):
    x=arr[:,0:4].copy().view("<f4").ravel(); y=arr[:,4:8].copy().view("<f4").ravel(); z=arr[:,8:12].copy().view("<f4").ravel()
    return x,y,z

def detect_floors(z, gap):
    """z 密度峰检测楼层（0.2m 分箱+平滑，峰高>10% 最大值，峰间距≥gap）"""
    lo,hi=float(z.min()),float(z.max())
    bins=np.arange(lo,hi+0.2,0.2)
    cnt,_=np.histogram(z,bins=bins)
    k=max(1,len(cnt)//10)
    sm=np.convolve(cnt,np.ones(k)/k,mode="same")
    thr=0.10*sm.max()
    peaks=[]
    for i in range(1,len(sm)-1):
        if sm[i]>=thr and sm[i]>=sm[i-1] and sm[i]>=sm[i+1]:
            zc=bins[i]+0.1
            if not peaks or zc-peaks[-1]>=gap:
                peaks.append(zc)
    return peaks

args={a.split("=",1)[0]:a.split("=",1)[1] for a in sys.argv[2:] if "=" in a}
g=lambda k,d: args.get(k,str(d))
src=sys.argv[1]
dst=args.get("OUT",re.sub(r"\.pcd$","_polished.pcd",src))
arr,step=load(src)
x,y,z=xyz(arr,step)
fin=np.isfinite(x)&np.isfinite(y)&np.isfinite(z)
arr=arr[fin]; x,y,z=x[fin],y[fin],z[fin]
print(f"[polish] 载入 {len(z)} 点")

# ① z 轴裁剪
if "ZMIN" in args or "ZMAX" in args:
    zmin=float(g("ZMIN",z.min())); zmax=float(g("ZMAX",z.max()))
    keep=(z>=zmin)&(z<=zmax); note=f"显式区间 [{zmin},{zmax}]"
elif g("FLOORS","1")=="auto":
    fk,rt,st,gap=float(g("FLOOR_KEEP",0.10)),float(g("ROOF_TRIM",0.15)),float(g("STORY",3.2)),float(g("MIN_FLOOR_GAP",2.5))
    peaks=detect_floors(z,gap)
    if len(peaks)<=1:
        fl,cp=float(np.percentile(z,0.5)),float(np.percentile(z,99.5))
        keep=(z>=fl+fk)&(z<=cp-rt); note=f"单层自动 [{fl+fk:.2f},{cp-rt:.2f}]"
    else:
        keep=np.zeros(len(z),bool); segs=[]
        for i,p in enumerate(peaks):
            top=(peaks[i+1]-0.30) if i+1<len(peaks) else p+st
            m_=(z>=p+fk)&(z<=top-rt); keep|=m_; segs.append(f"L{i+1}[{p+fk:.2f},{top-rt:.2f})")
        note="多层 "+ " ".join(segs)
else:
    fl,cp=float(np.percentile(z,0.5)),float(np.percentile(z,99.5))
    fk,rt=float(g("FLOOR_KEEP",0.10)),float(g("ROOF_TRIM",0.15))
    keep=(z>=fl+fk)&(z<=cp-rt); note=f"单层自动 [{fl+fk:.2f},{cp-rt:.2f}]"
arr,x,y,z=arr[keep],x[keep],y[keep],z[keep]
print(f"[polish] ①z 裁剪（{note}）: 余 {len(z)}")

# ② 体素降采样（自适应）
target=int(g("TARGET",400000))
v=args.get("VOXEL")
if v:
    v=float(v)
    key=np.floor(np.stack([x,y,z],1)/v).astype(np.int64)
    _,ui=np.unique(key,axis=0,return_index=True)
    sel=np.sort(ui)
    arr,x,y,z=arr[sel],x[sel],y[sel],z[sel]
    print(f"[polish] ②体素 {v}m（显式）: 余 {len(z)}")
else:
    # 二分法搜体素：精确命中 target（±10%）
    lo_v,hi_v=0.03,1.0
    best=None
    while hi_v-lo_v>0.005:
        mid=round((lo_v+hi_v)/2,3)
        key=np.floor(np.stack([x,y,z],1)/mid).astype(np.int64)
        cnt=len(np.unique(key,axis=0))
        if cnt>target: lo_v=mid
        else: hi_v=mid; best=(mid,cnt)
        if abs(cnt-target)<=target*0.10: best=(mid,cnt); break
    v,cnt=best if best else (hi_v,None)
    key=np.floor(np.stack([x,y,z],1)/v).astype(np.int64)
    _,ui=np.unique(key,axis=0,return_index=True)
    sel=np.sort(ui)
    arr,x,y,z=arr[sel],x[sel],y[sel],z[sel]
    print(f"[polish] ②体素 {v}m（二分自适应）: 余 {len(z)}（目标 {target}±10%）")

# ③ SOR 离群去除
sor_min=int(g("SOR_MIN",5))
if sor_min>0:
    r=float(g("SOR_RADIUS",0.25))
    cell={}
    gidx=np.floor(np.stack([x,y,z],1)/r).astype(np.int64)
    for i,k in enumerate(map(tuple,gidx)): cell[k]=cell.get(k,0)+1
    cnt=np.array([sum(cell.get((a,b,c),0) for a in (k[0]-1,k[0],k[0]+1)
                                    for b in (k[1]-1,k[1],k[1]+1)
                                    for c in (k[2]-1,k[2],k[2]+1)) for k in map(tuple,gidx)])
    keep=cnt>=sor_min
    arr=arr[keep]
    print(f"[polish] ③SOR(r={r},≥{sor_min}邻): 余 {len(arr)}（剔 {(~keep).sum()}）")

save(dst,arr)
print(f"[polish] 完成 → {dst}（{len(arr)} 点, {len(arr)*16/1e6:.1f}MB）")

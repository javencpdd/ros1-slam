#!/usr/bin/env python3
# pcd_slice.py — 按 Z 厚度把 PCD 切成水平断面片（输出 4 字段小 pcd，CloudCompare/Foxglove 均可开）
import sys, os, re, numpy as np
path, thick = sys.argv[1], float(sys.argv[2])
zmin_req = sys.argv[3] if len(sys.argv) > 3 and sys.argv[3] else None
zmax_req = sys.argv[4] if len(sys.argv) > 4 and sys.argv[4] else None
outdir   = sys.argv[5] if len(sys.argv) > 5 else "slices"
tag      = sys.argv[6] if len(sys.argv) > 6 else "cloud"
raw = open(path, "rb").read()
hdr_end = raw.find(b"DATA binary\n") + len(b"DATA binary\n")
hdr = raw[:hdr_end].decode(errors="replace")
n  = int(re.search(r"POINTS (\d+)", hdr).group(1))
step = int(re.search(r"POINT_STEP (\d+)", hdr).group(1)) if re.search(r"POINT_STEP (\d+)", hdr) else 32
arr = np.frombuffer(raw[hdr_end:hdr_end+n*step], dtype=np.uint8).reshape(n, step)
x = arr[:,0:4].copy().view("<f4").ravel(); y = arr[:,4:8].copy().view("<f4").ravel(); z = arr[:,8:12].copy().view("<f4").ravel()
fin = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
z = z[fin]
lo = float(zmin_req) if zmin_req else float(np.floor(z.min())) - 0.5
hi = float(zmax_req) if zmax_req else float(np.ceil(z.max())) + 0.5
os.makedirs(outdir, exist_ok=True)
base = os.path.splitext(os.path.basename(path))[0]
made = 0
band = lo
while band < hi:
    b2 = round(band + thick, 3)
    m = fin & (z >= band) & (z < b2)
    if m.sum() < 10:
        band = b2; continue
    sub = arr[fin][m]
    N = len(sub)
    hdr_out = (f"# .pcd v0.7 - z-slice [{band:.2f},{b2:.2f}) of {base}\n"
               f"VERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n"
               f"WIDTH {N}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {N}\nPOINT_STEP 16\nDATA binary\n")
    out = os.path.join(outdir, f"{base}_z{band:+.2f}_{b2:+.2f}.pcd")
    open(out, "wb").write(hdr_out.encode() + sub[:, :16].tobytes())
    print(f"  z∈[{band:+.2f},{b2:+.2f}): {N:7d} 点 → {os.path.basename(out)}")
    made += 1
    band = b2
print(f"[m_slice] 共 {made} 片 → {outdir}/（scp 或 m_view 逐片查看）")

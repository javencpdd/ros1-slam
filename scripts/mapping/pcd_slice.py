#!/usr/bin/env python3
# pcd_slice.py — 按 Z 厚度把 PCD 切成水平断面片（输出 4 字段小 pcd，CloudCompare/Foxglove 均可开）
# v2：文件名去 '+'（正负用 p/m 前缀）、头部去 '#' 注释行——两者均为 CloudCompare PCD 解析器不兼容项
import sys, os, re, numpy as np

def load(path):
    raw = open(path, "rb").read()
    he = raw.find(b"DATA binary\n") + len(b"DATA binary\n")
    hdr = raw[:he].decode(errors="replace")
    n = int(re.search(r"POINTS (\d+)", hdr).group(1))
    m = re.search(r"POINT_STEP (\d+)", hdr)
    step = int(m.group(1)) if m else 32
    arr = np.frombuffer(raw[he:he+n*step], dtype=np.uint8).reshape(n, step)
    return arr, step

def save(path, arr):
    N = len(arr)
    hdr = (f"VERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n"
           f"WIDTH {N}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {N}\nPOINT_STEP 16\nDATA binary\n")
    open(path, "wb").write(hdr.encode() + arr[:, :16].tobytes())

def pn(v):
    """+1.5 → p1.50 / -1.5 → m1.50（文件名中不用 + 号：CloudCompare 兼容性差）"""
    return ("p" if v >= 0 else "m") + f"{abs(v):.2f}"

def main():
    path = sys.argv[1]
    thick = float(sys.argv[2])
    zmin_req = sys.argv[3] if len(sys.argv) > 3 and sys.argv[3] else None
    zmax_req = sys.argv[4] if len(sys.argv) > 4 and sys.argv[4] else None
    outdir = sys.argv[5] if len(sys.argv) > 5 else "slices"
    base = sys.argv[6] if len(sys.argv) > 6 else "cloud"
    arr, step = load(path)
    x = arr[:,0:4].copy().view("<f4").ravel()
    y = arr[:,4:8].copy().view("<f4").ravel()
    z = arr[:,8:12].copy().view("<f4").ravel()
    fin = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
    z = z[fin]
    lo = float(zmin_req) if zmin_req else float(np.floor(z.min())) - 0.5
    hi = float(zmax_req) if zmax_req else float(np.ceil(z.max())) + 0.5
    os.makedirs(outdir, exist_ok=True)
    made = 0
    band = lo
    while band < hi:
        b2 = round(band + thick, 3)
        m = fin & (z >= band) & (z < b2)
        if m.sum() >= 10:
            sub = arr[fin][m]
            N = len(sub)
            out = os.path.join(outdir, f"{base}_z{pn(band)}_{pn(b2)}.pcd")
            save(out, sub)
            print(f"  z∈[{band:+.2f},{b2:+.2f}): {N:7d} 点 → {os.path.basename(out)}")
            made += 1
        band = b2
    print(f"[m_slice] 共 {made} 片 → {outdir}/（scp 或 m_view 逐片查看）")

if __name__ == "__main__":
    main()

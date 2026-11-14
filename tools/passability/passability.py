#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tools/passability/passability.py —— 楼梯「通道 vs 障碍」静态标注的写/校验/生成/变更摘要

为何存在：H 稿 C-5 实证「同几何楼梯＋中段一根立柱」在 2 传感臂 × 3 分辨率的八格里**全部不可分**
⇒ 几何通道原则上答不了「这级梯此刻能不能用」，需要一层语义标注（I 稿＝方向四先行子集）。
本工具只做三件事：**让人写的标注自洽**、**把标注变成机器可读栅格**、**改动后给出可粘进台账的摘要**。

性质：纯离线。零 ROS 节点、零在册端口、不接任何 launch、**不改任何门值/判据**；
生成的 mask 是给「我方合并代价层」的中间物，**不是** map_server 的占用栅格，也**不是** costmap 的第二个
StaticLayer（X2 已实测：两个 StaticLayer 不能叠加）。真正读入要等 #46／B7 裁定。

调用姿势自证：`python3 tools/passability/passability.py --selftest`（零 ROS 依赖，不需 env 三步）。
rc：0=通过；5=校验有红（含自检里"该红的必须红"未红）。

红线（写死在代码里，不靠自觉）：
  ①未标注＝unknown(0)，**任何路径都不塌陷成"可走"**；
  ②同一格被两个不同语义覆盖＝冲突，直接红，**不实现"后写的赢"**；
  ③生成物与源 sha 不符＝有人手改了生成物，红（生成物只读，改源重建）。
"""
import argparse
import hashlib
import io
import json
import math
import os
import subprocess
import sys
import tempfile

import numpy as np
import yaml
from PIL import Image, ImageDraw

UNKNOWN, BLOCKED, PASSABLE = 0, 1, 2
USE2VAL = {"blocked_stair": BLOCKED, "blocked": BLOCKED, "passable_stair": PASSABLE, "corridor": PASSABLE}
USES = sorted(USE2VAL.keys()) + ["unknown"]
ROOT = "/home/ros1-slam"
DEF_SRC = os.path.join(ROOT, "ws/src/95_simulation/config/passability/passability.yaml")
DEF_MAP = os.path.join(ROOT, "data/maps/lite3.yaml")
DEF_OUT = os.path.join(ROOT, "ws/src/95_simulation/config/passability/generated")
MASK_MAGIC_NOTE = ("# PASSABILITY MASK — 不是 map_server 占用栅格，勿直接加载；"
                   "值 0=unknown 1=blocked 2=passable；溯源见同目录 passability_mask.json\n")


def sha_of(path):
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        h.update(fh.read())
    return h.hexdigest()


def load_map(map_yaml):
    m = yaml.safe_load(io.open(map_yaml, encoding="utf-8").read())
    img = m["image"]
    if not os.path.isabs(img):
        img = os.path.join(os.path.dirname(os.path.abspath(map_yaml)), img)
    return {"resolution": float(m["resolution"]), "origin": [float(v) for v in m["origin"]],
            "image": img}


def grid_shape(map_yaml):
    im = Image.open(map_yaml["image"])
    return int(im.size[0]), int(im.size[1])          # (nx, ny) 列数/行数


# --- 几何 ---------------------------------------------------------------------
def _seg_int(p1, p2, p3, p4):
    def cr(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    d1, d2 = cr(p3, p4, p1), cr(p3, p4, p2)
    d3, d4 = cr(p1, p2, p3), cr(p1, p2, p4)
    return ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0))


def self_intersect(poly):
    n = len(poly)
    for i in range(n):
        for j in range(i + 1, n):
            if j == i or (j + 1) % n == i or (i + 1) % n == j:
                continue                              # 相邻边共享顶点，不算自交
            if _seg_int(poly[i], poly[(i + 1) % n], poly[j], poly[(j + 1) % n]):
                return True
    return False


def world_to_cell(poly, mp, nx, ny):
    r = mp["resolution"]
    return [((px - mp["origin"][0]) / r, (py - mp["origin"][1]) / r) for px, py in poly]


def raster(poly_cell, nx, ny):
    m = Image.new("L", (nx, ny), 0)
    ImageDraw.Draw(m).polygon([tuple(pt) for pt in poly_cell], fill=255)
    return np.array(m) > 0


# --- 校验 ---------------------------------------------------------------------
def check(src, map_yaml_path, verbose=True):
    errs, warns = [], []
    doc = yaml.safe_load(io.open(src, encoding="utf-8").read()) or {}
    mp = load_map(map_yaml_path)
    nx, ny = grid_shape(mp)
    regions = doc.get("regions") or []
    if doc.get("frame_id") != "map":
        warns.append("frame_id 不是 map（当前 %r）——mask 与底图同框架，跨框架引用要显式换算" % (doc.get("frame_id"),))
    acc = np.zeros((ny, nx), dtype=np.int8)
    for i, reg in enumerate(regions):
        nm = reg.get("name") or ("region#%d" % i)
        use = reg.get("use")
        poly = reg.get("polygon")
        if use not in USE2VAL:
            errs.append("%s：use=%r 不在枚举 %s 内 ⇒ 按纪律只能落 unknown，不得默认成可走" % (nm, use, USES))
            continue
        if not poly or len(poly) < 3:
            errs.append("%s：polygon 少于 3 点，无法成面" % nm)
            continue
        if any(len(pt) < 2 for pt in poly):
            errs.append("%s：polygon 有点缺坐标" % nm)
            continue
        pts = [(float(p[0]), float(p[1])) for p in poly]
        if self_intersect(pts):
            errs.append("%s：多边形自交（顶点顺序错了？自交面 rasterize 结果不可信）" % nm)
            continue
        cell = raster(world_to_cell(pts, mp, nx, ny), nx, ny)
        if not cell.any():
            warns.append("%s：整落在底图之外，mask 里没有任何格" % nm)
        val = USE2VAL[use]
        clash = np.argwhere(cell & (acc != 0) & (acc != val))
        if len(clash):
            other = sorted(set(int(acc[y, x]) for y, x in clash))
            errs.append("%s：与已标注区在 %d 格上语义冲突（本区=%d，已有=%s）⇒ 不实现「后写的赢」，请分块或改界"
                        % (nm, len(clash), val, other))
        acc[cell] = val
    cov = {int(v): int((acc == v).sum()) for v in (UNKNOWN, BLOCKED, PASSABLE)}
    tot = nx * ny
    if verbose:
        for e in errs:
            print("  [红] %s" % e)
        for w in warns:
            print("  [警] %s" % w)
        print("  栅格 %dx%d（分辨率 %.3f、原点 [%.2f, %.2f]）｜区数 %d｜unknown %.1f%% blocked %.1f%% passable %.1f%%"
              % (nx, ny, mp["resolution"], mp["origin"][0], mp["origin"][1], len(regions),
                 100.0 * cov[UNKNOWN] / tot, 100.0 * cov[BLOCKED] / tot, 100.0 * cov[PASSABLE] / tot))
    return errs, warns, acc, mp, nx, ny


# --- 生成 ---------------------------------------------------------------------
def _disp(p):
    return os.path.relpath(p, ROOT) if os.path.abspath(p).startswith(ROOT + os.sep) else p


def _read_pgm_header(raw):
    """按 PGM 规范取头：magic nx ny maxval，允许 # 注释行；返回体起始偏移。"""
    i = [0]

    def tok():
        n = len(raw)
        while i[0] < n:
            c = raw[i[0]:i[0] + 1]
            if c in (b" ", b"\t", b"\n", b"\r"):
                i[0] += 1
                continue
            if c == b"#":
                i[0] = raw.index(b"\n", i[0]) + 1
                continue
            j = i[0]
            while j < n and raw[j:j + 1] not in (b" ", b"\t", b"\n", b"\r"):
                j += 1
            t = raw[i[0]:j].decode("ascii")
            i[0] = j
            return t
        return None

    magic, nx, ny, maxv = tok(), tok(), tok(), tok()
    i[0] += 1                                        # maxval 后恰有一个空白字符收尾
    return magic, int(nx), int(ny), int(maxv), i[0]


def build(src, map_yaml_path, outdir):
    errs, warns, acc, mp, nx, ny = check(src, map_yaml_path)
    if errs:
        print("BUILD 拒绝：校验有 %d 条红，不产出（宁可不出，不出过期物）" % len(errs))
        return 5
    if not os.path.isdir(outdir):
        os.makedirs(outdir)
    mask_path = os.path.join(outdir, "passability_mask.pgm")
    with open(mask_path, "wb") as fh:
        fh.write(("P5\n%s%d %d\n255\n" % (MASK_MAGIC_NOTE, nx, ny)).encode("utf-8"))
        fh.write(acc.astype(np.uint8).tobytes())
    doc = yaml.safe_load(io.open(src, encoding="utf-8").read()) or {}
    side = {"source": _disp(src), "source_sha256": sha_of(src),
            "map_ref": _disp(os.path.abspath(map_yaml_path)),
            "map_image_sha256": sha_of(mp["image"]), "resolution": mp["resolution"],
            "origin": mp["origin"], "grid": [nx, ny], "values": {"0": "unknown", "1": "blocked", "2": "passable"},
            "regions": [{"name": r.get("name"), "use": r.get("use"),
                         "poly_sha256": hashlib.sha256(json.dumps(r.get("polygon")).encode()).hexdigest()[:12]}
                        for r in (doc.get("regions") or [])]}
    with io.open(os.path.join(outdir, "passability_mask.json"), "w", encoding="utf-8") as fh:
        fh.write(json.dumps(side, ensure_ascii=False, indent=2) + "\n")
    png = preview(src, map_yaml_path, os.path.join(outdir, "passability_preview.png"))
    print("BUILD OK → %s（%dx%d）、%s、%s"
          % (_disp(mask_path), nx, ny, _disp(os.path.join(outdir, "passability_mask.json")), _disp(png)))
    return 0


def preview(src, map_yaml_path, outpng):
    errs, warns, acc, mp, nx, ny = check(src, map_yaml_path, verbose=False)
    base = Image.open(mp["image"]).convert("RGB")
    ov = Image.new("RGBA", base.size, (0, 0, 0, 0))
    d = ImageDraw.Draw(ov)
    for y, x in zip(*np.where(acc == PASSABLE)):
        d.point((int(x), ny - 1 - int(y)), fill=(0, 220, 0, 160))
    for y, x in zip(*np.where(acc == BLOCKED)):
        d.point((int(x), ny - 1 - int(y)), fill=(230, 0, 0, 160))
    out = Image.alpha_composite(base.convert("RGBA"), ov).convert("RGB")
    out.save(outpng)
    return outpng


# --- 变更摘要（甲→乙自动升级那三列） --------------------------------------------
def diff(src, map_yaml_path, outdir):
    side_p = os.path.join(outdir, "passability_mask.json")
    if not os.path.exists(side_p):
        print("无已生成物可比（先 build）：%s 不存在" % side_p)
        return 5
    old = json.load(io.open(side_p, encoding="utf-8"))
    new_doc = yaml.safe_load(io.open(src, encoding="utf-8").read()) or {}
    new = {r.get("name"): r.get("use") for r in (new_doc.get("regions") or [])}
    olds = {r["name"]: r["use"] for r in old.get("regions", [])}
    print("| 区域 | 改前 → 改后 | 影响的判据面 |")
    print("|---|---|---|")
    n = 0
    for k in sorted(set(list(new) + list(olds))):
        a, b = olds.get(k, "（无）"), new.get(k, "（删除）")
        if a == b:
            continue
        n += 1
        eff = ("R-b 硬闸新增/变更" if b in USE2VAL and USE2VAL.get(b) == BLOCKED else
               ("R-a 许可新增/变更" if b in USE2VAL and USE2VAL.get(b) == PASSABLE else "R-c 落 unknown"))
        print("| %s | %s → %s | %s |" % (k, a, b, eff))
    if old.get("source_sha256") != sha_of(src):
        print("（源已变：sha %s… → %s…，生成物过期，须重 build）"
              % (old.get("source_sha256", "")[:12], sha_of(src)[:12]))
        n += 1
    print("变更条目 %d 条；以上可直接粘进 note/04_配置改动diff.md" % n)
    return 0


def verify_built(src, outdir):
    """红线③：生成物是否对得上当前源（防手改生成物／防过期）。"""
    side_p = os.path.join(outdir, "passability_mask.json")
    if not os.path.exists(side_p):
        print("[红] 无生成物（未 build）")
        return 5
    side = json.load(io.open(side_p, encoding="utf-8"))
    bad = []
    if side.get("source_sha256") != sha_of(src):
        bad.append("源已改而生成物未重建（源 %s… ≠ 登记 %s…）" % (sha_of(src)[:12], str(side.get("source_sha256"))[:12]))
    mp = load_map(DEF_MAP)
    if side.get("map_image_sha256") != sha_of(mp["image"]):
        bad.append("底图换了 ⇒ mask 坐标系可能已错位，须重 build 并目视 preview")
    mask_p = os.path.join(outdir, "passability_mask.pgm")
    raw = open(mask_p, "rb").read()
    nx, ny = side.get("grid", [0, 0])
    magic, hnx, hny, maxv, off = _read_pgm_header(raw)
    body = raw[off:]
    if magic != "P5" or maxv != 255:
        bad.append("mask 不是预期的 P5/255 灰度图（实为 %r/%d）" % (magic, maxv))
    if (hnx, hny) != (nx, ny):
        bad.append("mask 头尺寸 %dx%d ≠ json 登记 %dx%d ⇒ 生成物被手改" % (hnx, hny, nx, ny))
    if len(body) != nx * ny:
        bad.append("mask 体长度 %d ≠ 登记栅格 %dx%d ⇒ 生成物被手改或截断" % (len(body), nx, ny))
    if body and set(np.unique(np.frombuffer(body, dtype=np.uint8))) - {UNKNOWN, BLOCKED, PASSABLE}:
        bad.append("mask 出现枚举外的值 ⇒ 不是本工具产物")
    for x in bad:
        print("  [红] %s" % x)
    if not bad:
        print("[绿] 生成物与源、底图、登记尺寸三者一致（可引用）")
    return 5 if bad else 0


# --- 自检：含"该红的必须红"（05 纪律：新门必须先真红一次） -----------------------
def selftest():
    bad = 0

    def ck(name, cond, detail):
        nonlocal bad
        print("[%s] %s ｜ %s" % ("PASS" if cond else "FAIL", name, detail))
        if not cond:
            bad += 1

    tmp = tempfile.mkdtemp(prefix="passability_selftest_")
    src = os.path.join(tmp, "p.yaml")
    out = os.path.join(tmp, "gen")

    def write(regions):
        with io.open(src, "w", encoding="utf-8") as fh:
            fh.write(yaml.safe_dump({"frame_id": "map", "regions": regions}, allow_unicode=True))

    ck("S0 底图可读且与工具同尺寸", grid_shape(load_map(DEF_MAP)) == (299, 290), str(grid_shape(load_map(DEF_MAP))))
    write([])
    e, w, acc, mp, nx, ny = check(src, DEF_MAP, verbose=False)
    ck("S1 空标注＝全 unknown（不得默认成可走）", not e and int((acc == UNKNOWN).sum()) == nx * ny,
       "unknown=%d/%d" % (int((acc == UNKNOWN).sum()), nx * ny))
    write([{"name": "A", "use": "passable_stair", "polygon": [[-3.1, 1.2], [-2.2, 1.2], [-2.2, 2.4], [-3.1, 2.4]]}])
    e, w, acc, mp, nx, ny = check(src, DEF_MAP, verbose=False)
    ck("S2 合法单区可栅格化", not e and 0 < int((acc == PASSABLE).sum()) < 5000, "passable 格=%d" % int((acc == PASSABLE).sum()))
    rc = build(src, DEF_MAP, out)
    ck("S3 build 产出三件且可被 verify_built 认", rc == 0 and os.path.exists(os.path.join(out, "passability_mask.pgm"))
       and verify_built(src, out) == 0, "mask+json+png 在位且自证一致")
    write([{"name": "A", "use": "blocked_stair", "polygon": [[-3.1, 1.2], [-2.2, 1.2], [-2.2, 2.4], [-3.1, 2.4]]}])
    ck("S4 改源后生成物必须判过期（红线③）", verify_built(src, out) == 5, "源 sha 变了而 mask 未重建 ⇒ 必红")
    write([{"name": "A", "use": "passable_stair", "polygon": [[-3.1, 1.2], [-2.2, 1.2], [-2.2, 2.4], [-3.1, 2.4]]},
           {"name": "B", "use": "blocked_stair", "polygon": [[-3.0, 1.3], [-2.1, 1.3], [-2.1, 2.3], [-3.0, 2.3]]}])
    e, w, acc, mp, nx, ny = check(src, DEF_MAP, verbose=False)
    ck("S5 语义冲突必红且不「后写赢」", len([x for x in e if "冲突" in x]) == 1,
       "只红一次（B 撞 A 的累加格）：%s" % (e[0][:64] if e else "无"))
    ck("S5b 冲突时 build 拒绝产出", build(src, DEF_MAP, os.path.join(tmp, "g5")) == 5, "宁可不产，也不出静默覆盖后的物")
    write([{"name": "C", "use": "escalator", "polygon": [[-3.1, 1.2], [-2.2, 1.2], [-2.2, 2.4], [-3.1, 2.4]]}])
    e, w, acc, mp, nx, ny = check(src, DEF_MAP, verbose=False)
    ck("S6 枚举外 use 必红（不得当成可走）", len(e) == 1 and "escalator" in e[0], e[0][:60] if e else "未红")
    write([{"name": "D", "use": "passable_stair", "polygon": [[-3.1, 1.2], [-2.2, 2.4], [-2.2, 1.2], [-3.1, 2.4]]}])
    e, w, acc, mp, nx, ny = check(src, DEF_MAP, verbose=False)
    ck("S7 自交多边形必红", len(e) == 1 and "自交" in e[0], e[0][:60] if e else "未红")
    write([{"name": "E", "use": "passable_stair", "polygon": [[120.0, 120.0], [121.0, 120.0], [121.0, 121.0]]}])
    e, w, acc, mp, nx, ny = check(src, DEF_MAP, verbose=False)
    ck("S8 完全落在图外要报警（静默空 mask 比红更坏）", not e and len(w) == 1, (w[0][:60] if w else "未警"))
    write([{"name": "A", "use": "passable_stair", "polygon": [[-3.1, 1.2], [-2.2, 1.2], [-2.2, 2.4], [-3.1, 2.4]]}])
    o1, o2 = os.path.join(tmp, "g1"), os.path.join(tmp, "g2")
    build(src, DEF_MAP, o1)
    build(src, DEF_MAP, o2)
    ck("S9 同输入两次 build 产物逐字节相同（确定性）",
       sha_of(os.path.join(o1, "passability_mask.pgm")) == sha_of(os.path.join(o2, "passability_mask.pgm")),
       "mask sha 全等")
    print("PASSABILITY SELFTEST: FAIL=%d ｜ 临时件 %s" % (bad, tmp))
    return 0 if bad == 0 else 5


def main():
    ap = argparse.ArgumentParser(description="楼梯可通行性静态标注工具（离线，不改门值）")
    ap.add_argument("cmd", choices=["check", "build", "preview", "diff", "verify-built", "selftest"])
    ap.add_argument("--src", default=DEF_SRC)
    ap.add_argument("--map", default=DEF_MAP)
    ap.add_argument("--out", default=DEF_OUT)
    a = ap.parse_args()
    if a.cmd == "selftest":
        return selftest()
    if a.cmd == "check":
        e, w, *_ = check(a.src, a.map)
        print("CHECK: 红 %d ／ 警 %d" % (len(e), len(w)))
        return 5 if e else 0
    if a.cmd == "build":
        return build(a.src, a.map, a.out)
    if a.cmd == "preview":
        print("PREVIEW →", preview(a.src, a.map, os.path.join(a.out, "passability_preview.png")))
        return 0
    if a.cmd == "diff":
        return diff(a.src, a.map, a.out)
    return verify_built(a.src, a.out)


if __name__ == "__main__":
    sys.exit(main())

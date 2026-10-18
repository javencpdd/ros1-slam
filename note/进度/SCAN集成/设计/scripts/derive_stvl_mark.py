#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""stvl_标注列**只读后处理**推导件（G-7 到签＝路径 A；★零窗、★不改任何在册件、★不产归因）

定形真身＝`note/进度/SCAN集成/证据/log/raw/r356_x86联调批/acceptance_verdict/stvl_标注列定形_x86小美.md`
  · §2.1 字段与取值域（顶档 `有效／无痕／N3` ＋ 必带子键；缺任一必带子键⇒顶档即 `N3`）
  · §2.2 判据式四前置 P1—P4（★本件取 cost 面＋cloud log，**不取 node log**，见 §2.5 第 1 条）
  · §2.3 判据足印常量 `rows 18—21 × cols 29—32`（16 格）；★`x2_pub_cloud.py` 的 `build()` 一变即须重算
  · §2.4 顶档判定表；§2.5 八条"不得判"
  · ③ 生产路径表 A 行＝"能产到 `层内痕迹列`＋全部原值子键；★顶档只能落到 `N3`（缺归因）"

用法：
  python3 derive_stvl_mark.py --selftest
  python3 derive_stvl_mark.py --dir <X2 窗留底目录> [--segs x2_g1,x2_g2,...]
rc：0＝全部被检段 P1—P3 成立（★顶档 N3 属路径 A 的设计使然、★不是失败）｜5＝任一段 P1／P3 不成立｜9＝前置缺失（目录或原始件不在位）
"""
import argparse
import os
import re
import sys

W = 40                       # 图幅宽（40×40＝1600 格；与探针 _parse.py 同索引法 g(r,c)=第 r*W+c 个整数）
FACE = W * W
FOOT_ROWS = (18, 21)         # §2.3 判据足印（含端点）
FOOT_COLS = (29, 32)
MARK_ROWS = 10               # §2.2 P3 判别格值所在行
MARK_COLS = (10, 14)
TRACE_MAX_DOMAIN = (0, 99, 100)


def parse_face(path):
    """从 `cost_*.txt` 的 `data:` 段取整数面；返回 (值列表或 None, data_行数)。"""
    txt = open(path, 'r', errors='replace').read()
    lines = [ln for ln in txt.split('\n') if ln.strip().startswith('data:')]
    if not lines:
        return None, 0
    nums = []
    for ln in lines:
        nums.extend(int(v) for v in re.findall(r'-?\d+', ln.split(':', 1)[1]))
    return nums, len(lines)


def cloud_ok(path):
    """§2.2 P2：含 `X2_CLOUD first sent pts=42` 且其后无 Traceback／AttributeError。缺该 log 文件＝False。"""
    if not os.path.isfile(path):
        return False
    txt = open(path, 'r', errors='replace').read()
    m = txt.find('X2_CLOUD first sent pts=42')
    if m < 0:
        return False
    return not any(t in txt[m:] for t in ('Traceback', 'AttributeError'))


def derive(dirpath, seg):
    """单段推导；★缺归因⇒顶档一律 N3（路径 A 的设计边界）。"""
    cost = os.path.join(dirpath, 'cost_%s.txt' % seg)
    cloud = os.path.join(dirpath, 'cloud_%s.txt' % seg)
    if not os.path.isfile(cloud):
        cloud = os.path.join(dirpath, 'cloud_%s.log' % seg)
    out = {'段': seg}
    if not os.path.isfile(cost):
        out['前置'] = 'P1＝前置缺失（cost 件不在位）'
        out['层内痕迹列'] = out['顶档'] = 'N3'
        out['必带子键齐'] = False
        out['归因'] = '未定源'
        out['原因'] = 'cost_%s.txt 不在位' % seg
        return out, False
    nums, dlines = parse_face(cost)
    p1 = (dlines == 1) and (nums is not None) and (len(nums) == FACE)
    if not p1:
        out.update({'前置': 'P1 不成立（data_行数=%d 解析值数=%s）' % (dlines, 'None' if nums is None else len(nums)),
                    '层内痕迹列': 'N3', '顶档': 'N3', '必带子键齐': False, '归因': '未定源',
                    '原因': '原始件不合格（P1）⇒ 该段不判被试'})
        return out, False
    g = lambda r, c: nums[r * W + c]
    marks = [g(MARK_ROWS, c) for c in range(MARK_COLS[0], MARK_COLS[1] + 1)]
    p3 = (marks == [100] * 5)
    foot = [(r, c) for r in range(FOOT_ROWS[0], FOOT_ROWS[1] + 1) for c in range(FOOT_COLS[0], FOOT_COLS[1] + 1)]
    fv = [g(r, c) for r, c in foot]
    trace_n = sum(1 for v in fv if v != 0)
    trace_max = max(fv) if fv else 0
    out.update({
        '痕迹_格数': trace_n,
        '痕迹_最大值': trace_max,
        '254_格数': sum(1 for v in nums if v == 254),
        '足印': 'rows[%d,%d]xcols[%d,%d]' % (FOOT_ROWS[0], FOOT_ROWS[1], FOOT_COLS[0], FOOT_COLS[1]),
        '判别格值': marks,
        'cloud_first42': cloud_ok(cloud),
    })
    p4_col = '有效' if (trace_n >= 1 and trace_max in (99, 100)) else ('无痕' if trace_n == 0 else 'N3')
    p2 = out['cloud_first42']
    out['层内痕迹列'] = p4_col
    out['归因'] = '未定源'                       # ★路径 A 不产归因（③ 表 A 行原文）
    out['必带子键齐'] = all(k in out for k in
                        ('痕迹_格数', '痕迹_最大值', '254_格数', '足印', '判别格值', 'cloud_first42', '归因', '层内痕迹列'))
    if not p3 or not p2:
        out['前置'] = 'P3%s、P2%s 须两前置齐' % ('成立' if p3 else '不成立', '成立' if p2 else '不成立')
        out['顶档'] = 'N3'
        out['原因'] = 'P2／P3 不成立 ⇒ 该段不判被试（§2.2）'
    else:
        out['前置'] = 'P1 成立、P2 成立、P3 成立、P4 已计'
        out['顶档'] = 'N3'
        out['原因'] = '★路径 A 缺归因（`归因=未定源`）⇒ 顶档按 §2.4 末行＝`N3`；★不得读作"数据缺口"，归因面须另起 M3 旁证窗（路径 C 或 E）'
    return out, (p3 and p2)


def emit(out):
    keys = ('段', '顶档', '层内痕迹列', '归因', '痕迹_格数', '痕迹_最大值', '254_格数', '足印', '判别格值',
            'cloud_first42', '必带子键齐', '前置', '原因')
    return ' ｜ '.join('%s=%s' % (k, out.get(k, '—')) for k in keys if k in out)


def selftest():
    ok = fail = 0
    lines = []

    def check(tag, cond, detail=''):
        nonlocal ok, fail
        if cond:
            ok += 1
            lines.append('  [PASS] %s %s' % (tag, detail))
        else:
            fail += 1
            lines.append('  [FAIL] %s %s' % (tag, detail))

    import tempfile
    d = tempfile.mkdtemp(prefix='derive_stvl_selftest_')
    def write_face(seg, foot_vals, mark_val=100, cloud='X2_CLOUD first sent pts=42\n'):
        vals = [0] * FACE
        for c in range(MARK_COLS[0], MARK_COLS[1] + 1):
            vals[MARK_ROWS * W + c] = mark_val
        for i, (r, c) in enumerate([(rr, cc) for rr in range(FOOT_ROWS[0], FOOT_ROWS[1] + 1)
                                    for cc in range(FOOT_COLS[0], FOOT_COLS[1] + 1)]):
            vals[r * W + c] = foot_vals[i]
        with open(os.path.join(d, 'cost_%s.txt' % seg), 'w') as f:
            f.write('header: x\nwidth: %d\nheight: %d\ndata: %s\n' % (W, W, ' '.join(str(v) for v in vals)))
        if cloud is not None:
            with open(os.path.join(d, 'cloud_%s.log' % seg), 'w') as f:
                f.write(cloud)

    # A1 有效面（足印内有 100）⇒ 层内痕迹列=有效、顶档仍 N3（缺归因）
    write_face('s_ok', [0] * 15 + [100])
    o, good = derive(d, 's_ok')
    check('A1 层内痕迹列=有效（足印内 1 格 100）', o['层内痕迹列'] == '有效' and o['痕迹_格数'] == 1 and o['痕迹_最大值'] == 100, emit(o))
    check('A1 ★顶档=N3 且原因写明"缺归因"（路径 A 设计边界）',
          o['顶档'] == 'N3' and o['归因'] == '未定源' and '缺归因' in o['原因'], '顶档=%s 归因=%s' % (o['顶档'], o['归因']))
    check('A1 必带子键齐', o['必带子键齐'] is True)

    # A2 无痕面（足印全 0）⇒ 层内痕迹列=无痕（★不得塞进 N3，见 §2.4 那句"把负读数伪装成数据缺口"）
    write_face('s_none', [0] * 16)
    o2, _ = derive(d, 's_none')
    check('A2 层内痕迹列=无痕（足印全 0、非数据缺口）', o2['层内痕迹列'] == '无痕' and o2['痕迹_格数'] == 0, emit(o2))

    # A3 P1 阴性对照：原始件残（data 段 2 行）⇒ 该段不判被试
    with open(os.path.join(d, 'cost_s_bad.txt'), 'w') as f:
        f.write('data: 1 2 3\ndata: 4 5 6\n')
    o3, good3 = derive(d, 's_bad')
    check('A3 P1 不成立⇒顶档 N3 且 rc 面记该段不合格', o3['顶档'] == 'N3' and good3 is False, o3['原因'])

    # A4 阳性对照（★证 A1 那条有牙）：把 100 挪到足印**外**⇒ 层内痕迹列必须变无痕
    write_face('s_out', [0] * 16, mark_val=100)
    with open(os.path.join(d, 'cost_s_out.txt')) as f:
        pass
    o4, _ = derive(d, 's_out')
    check('A4 足印内无值⇒无痕（★同一式子、痕迹位置决定档位＝断言有牙）', o4['层内痕迹列'] == '无痕', emit(o4))

    # A5 P2 阴性对照：cloud log 首帧后又出 Traceback ⇒ 该段不判被试
    write_face('s_tb', [0] * 15 + [100], cloud='X2_CLOUD first sent pts=42\nTraceback (most recent call last):\n')
    o5, good5 = derive(d, 's_tb')
    check('A5 P2 不成立（首帧后有 traceback）⇒顶档 N3、该段判不合格', o5['顶档'] == 'N3' and good5 is False, o5['前置'])

    # A6 前置缺失：段名不存在 ⇒ N3 且写明不在位
    o6, good6 = derive(d, 's_missing')
    check('A6 件不在位⇒顶档 N3（★不静默产空面）', o6['顶档'] == 'N3' and good6 is False, o6['原因'])

    for f in os.listdir(d):
        os.remove(os.path.join(d, f))
    os.rmdir(d)

    lines.append('  合计 PASS=%d FAIL=%d' % (ok, fail))
    return ok, fail, lines


def main(argv=None):
    ap = argparse.ArgumentParser(description='stvl_标注列只读后处理推导（路径 A；★零窗、★不改在册件、★不产归因）')
    ap.add_argument('--selftest', action='store_true')
    ap.add_argument('--dir', help='X2 窗留底目录（含 cost_*.txt 与 cloud_*.log）')
    # ★缺省只取 x2_g6：判据足印由合成点云 build() 推得（§2.3），只有四插件段才有点云发布侧；
    #   其余五段＝static2 构成、本无 cloud 发布侧 ⇒ 对其套 P2 只会造出"假 N3"，★不是被试失败。
    ap.add_argument('--segs', default='x2_g6')
    ap.add_argument('--expect-zero', action='store_true', help='预留：与在册期望值并报（本件不自判）')
    a = ap.parse_args(argv)
    rc = 0
    if a.selftest:
        ok, fail, lines = selftest()
        for ln in lines:
            print(ln)
        print('DERIVE_SELFTEST PASS=%d FAIL=%d rc=%d' % (ok, fail, 0 if fail == 0 else 1))
        rc = 0 if fail == 0 else 1
    if a.dir:
        if not os.path.isdir(a.dir):
            print('DERIVE 前置缺失：目录不在位 %s' % a.dir)
            return 9
        bad = 0
        for seg in [s.strip() for s in a.segs.split(',') if s.strip()]:
            o, good = derive(a.dir, seg)
            print('STVL_MARK ' + emit(o))
            if not good:
                bad += 1
        print('DERIVE_SUMMARY 被检段=%d P1—P3 全不成立段=%d｜★顶档全为 N3 属路径 A 设计边界（缺归因）、★不是失败、★不得读作数据缺口'
              % (len([s for s in a.segs.split(',') if s.strip()]), bad))
        print('DERIVE_GRADE 桶名＝甲-离（只读后处理既有落盘件；★不顶替甲-活、★不进分子）')
        rc = 0 if bad == 0 else 5
    if not a.selftest and not a.dir:
        print('用法：--selftest 或 --dir <X2 留底目录>（两者皆不传 ⇒ rc=9 前置缺失）')
        rc = 9
    return rc


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))

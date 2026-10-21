#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ★批173（R517）活体窗套件生成器★＝从 r514 全套复制改名成 r517 ＋ 六枚件内改形（B1／B4／B5／A4／A3／A2）
#   六枚改形的字面来源＝任务卡「批173 现场状态·补正本」；★逐枚 assert count==1 再落盘★
#   ★批170 教训＝改名替换串不得覆盖目标名里的同族子串（r511→r514 把 r511q 一起改成 r514q）★
#     ⇒ 本批先断言复制件内 r514q 字面＝0，再整体替换 r514→r517，替换后现读「件内 R= 路径＝盘上路径」
#   ★批172 教训＝同一个文件既当读者又当写手必须分两语句（先读进内存、再开写）★
import io, os, re, ast, sys, shutil, subprocess, hashlib

RAW = "/home/jack/ysc-code/ros1-slam/note/进度/SCAN集成/证据/log/raw"
SRC_NAME = "r514_活体窗按r511q回文重排八枚_余量小数差与基线检值与让路函数真跑并读rc与窗内SWAP与前后两枚sha与回潮写进件里与打印面标签修净"
DST_NAME = "r517_活体窗按r514q缺席清单落六枚件内修_真调让路函数并报分支命中枚数与自测挪到roscore之前与9档拆名与sha随句带行数与栏形新名与源面从句"
SRC = os.path.join(RAW, SRC_NAME)
DST = os.path.join(RAW, DST_NAME)

assert os.path.isdir(SRC), "源目录不在位"
if os.path.exists(DST):
    # ★只允许重建「本批尚未开窗的半成品」★：无 run/、无 00_window_log.txt、无清单 ⇒ 才可清空重来
    junk = os.listdir(DST)
    assert "run" not in junk and "00_window_log.txt" not in junk and "_sha256_manifest.txt" not in junk, \
        "目标目录已存在且含产物（%d 件），不允许重建" % len(junk)
    shutil.rmtree(DST)
    print("清空上一轮半成品目录（%d 件、无产物）后重建" % len(junk))

# ---------- 1. 复制（只复制链接件；产物／清单／历史现场不复制） ----------
KEEP = [
    "00_r514_window.sh", "01_复读件_r514.py", "02_make_copies_r514.py", "03_sub_elev_count.py",
    "_cmp_r514.py", "pub_rect_probe_r514.py", "sub_costmap_dump_r514.py", "sim_nav_smoke_r514.sh",
    "global_costmap_params_A_r514.yaml", "global_costmap_params_C_r514.yaml",
    "mbf_navigation_r514.launch", "mode_nav_sim_r514.launch",
]
for f in KEEP:
    assert os.path.isfile(os.path.join(SRC, f)), "源件缺失：" + f
os.makedirs(DST)
for f in KEEP:
    dstf = f.replace("r514", "r517")
    shutil.copyfile(os.path.join(SRC, f), os.path.join(DST, dstf))
print("复制 %d 件 -> %s" % (len(KEEP), DST_NAME))

def rd(p):
    with io.open(p, "r", encoding="utf-8") as fh:
        return fh.read()

def wr(p, s):
    with io.open(p, "w", encoding="utf-8") as fh:
        fh.write(s)

def sub1(s, old, new, tag):
    """★单枚替换：断言恰好命中 1 次才落★"""
    n = s.count(old)
    assert n == 1, "改形[%s] 命中 %d 次（应为 1）｜目标串前 80 字＝%s" % (tag, n, old[:80])
    return s.replace(old, new)

W = os.path.join(DST, "00_r517_window.sh")
P1 = os.path.join(DST, "01_复读件_r517.py")
PB = os.path.join(DST, "pub_rect_probe_r517.py")

# ---------- 2. 改名（先断言 r514q 不在复制件里，防批170 那枚同族子串被咬） ----------
#   ★批170 教训的补强★＝件内既有小写 r514 也有大写 R514（注释里的「R514 改形⑥」与环境变量 R514_YAML），
#   ★两种大小写都要替★，否则注释字面留在文件里、下一次按字面定位会扑空（本批首跑就栽在这一格）。
for fn in sorted(os.listdir(DST)):
    p = os.path.join(DST, fn)
    if os.path.isfile(p):
        t = rd(p)
        assert "r514q" not in t and "R514q" not in t, "复制件 %s 内含 r514q/R514q（替换链会咬到同族子串）" % fn
        t = t.replace("r514q", "@@KEEPQ@@").replace("R514q", "@@KEEPQ@@")
        t = t.replace("r514", "r517").replace("R514", "R517")
        t = t.replace("@@KEEPQ@@", "r514q")
        wr(p, t)
print("改名：文件名与内容 r514/R514 -> r517/R517（r514q 字面在复制件内 0 枚＝已断言）")

# ---------- 2b. 件内路径串 → 本批新目录名（★顺序要紧★＝先做完上面的通用替换、再把整串旧目录名换成新目录名） ----------
#   ★本批新目录名里含 r514q★ ⇒ 若先落新目录名再跑通用替换，新名里的 r514 会被一起改成 r517（＝批170 那枚「替换链咬到目标名里的同族子串」）★
OLD_DIRNAME = SRC_NAME.replace("r514", "r517")
for fn in sorted(os.listdir(DST)):
    p = os.path.join(DST, fn)
    if os.path.isfile(p):
        t = rd(p)
        n = t.count(OLD_DIRNAME)
        if n:
            t = t.replace(OLD_DIRNAME, DST_NAME)
            wr(p, t)
            print("  路径串改写：%s 内 %d 处旧目录名 -> 新目录名" % (fn, n))
assert DST_NAME.count("r514q") == 1, "新目录名应含 r514q 一枚（与回执册第 2 字段同串）"

# ---------- 3. 六枚件内改形 ----------
s = rd(W)

# ① B1-a＝删掉件:112—115 那套镜像重演（含其两行注释）＝CLRC 硬赋值＋( exit 6 ) 从未被 $? 读
OLD_CLASH = (
    "# ★R517 改形⑥＝同秒撞名让路要有「窗内」载体★（R499 那枚自测件在窗外、r508 目录 .prev_=0 枚＝无从证）：\n"
    "#   这里在 /tmp 临时目录里用★同一枚 STAMP★把让路那段判定再走一遍，要求它当场撞名并给出 rc=6（★只在 /tmp、本窗盘面不被二次改名★）。\n"
    "CLASH=$(mktemp -d /tmp/r517_clash_XXXXXX)\n"
    "mkdir -p \"$CLASH\" && touch \"$CLASH/.prev_${STAMP}_run\"\n"
    "if [ -e \"$CLASH/.prev_${STAMP}_run\" ]; then echo \"撞名\" >&2; ( exit 6 ); CLRC=6; else CLRC=0; fi\n"
    "rm -rf \"$CLASH\"; say \"  ★改形⑥窗内自证＝临时目录用同一枚 STAMP（$STAMP）造撞名 ⇒ 让路判定走到「已在位、不静默覆盖」分支、子壳 rc=$CLRC（★6＝会停、不是静默覆盖；★本窗盘面无二次改名★）\"\n"
)
s = sub1(s, OLD_CLASH, "", "B1-a 删镜像重演")

# ① B1-b＝第二圈前把源文件造回来（撞名态真出现）＋第二圈捕获 stderr、rc 从变量读回、计数分支命中枚数
OLD_Y2 = (
    "mkdir -p \"$SCR/run\"\n"
    "yield_prev \"$SCR\" \"$STAMP\"; Y1=$?\n"
    "Y1L=\"$YIELDED\"\n"
    "yield_prev \"$SCR\" \"$STAMP\"; Y2=$?\n"
    "rm -rf \"$SCR\"\n"
)
NEW_Y2 = (
    "mkdir -p \"$SCR/run\"\n"
    "yield_prev \"$SCR\" \"$STAMP\"; Y1=$?\n"
    "Y1L=\"$YIELDED\"\n"
    "# ★B1（r514q 缺席清单 B1）＝r514 那版第二圈进函数时源文件已被第一圈改名搬走 ⇒ 判定式根本不进入 ⇒ rc=0 是「没走到分支」不是「分支放行」\n"
    "#   ⇒ 本批★第二圈前把源文件造回位★（touch）⇒ 撞名态真出现；★第二圈的 rc 紧跟调用现取★、★stderr 用命令替换捕获★（分支 echo 走 >&2、say 只 tee stdout ⇒ 不捕获就进不了志）\n"
    "touch \"$SCR/run\"\n"
    "Y2ERR=$(yield_prev \"$SCR\" \"$STAMP\" 2>&1 >/dev/null); Y2=$?\n"
    "Y2HITS=$(printf '%s\\n' \"$Y2ERR\" | grep -c '撞名即停')\n"
    "rm -rf \"$SCR\"\n"
)
s = sub1(s, OLD_Y2, NEW_Y2, "B1-b 第二圈造回源文件＋捕获 stderr")

# ① B1-c＝自证句改写（含★分支命中枚数★与★stderr 原文亲手搬进窗志★；$? 不再出现在双引号串里）
OLD_SAY = "say \"  ★改形③自证（★同一份函数真跑两圈、rc 从 $? 读回★）＝scratch 第一圈 rc=$Y1（让开＝$Y1L） ‖ 同秒第二圈 rc=$Y2（★6＝会停、不是静默覆盖；不是 R511 那版的镜像重演★） ‖ 临时目录已删\"\n"
NEW_SAY = (
    "say \"  ★改形③＋B1 自证（★同一份函数真跑两圈、两圈的 rc 都紧跟调用现取★）＝scratch 第一圈 rc=$Y1（让开＝$Y1L） ‖ ★第二圈前已把源文件造回位★ ⇒ 同秒第二圈 rc=$Y2（★期望 6＝会停、不是静默覆盖★） ‖ ★分支命中枚数＝$Y2HITS（对 stderr 捕获串 grep '撞名即停'）★ ‖ ★stderr 原文（亲手搬进窗志）＝$Y2ERR★ ‖ ★该分支字面本走 stderr、不经搬运进不了窗志 ⇒ 志里数不到≠分支没执行★ ‖ 临时目录已删\"\n"
    "say \"  ★B1 判读尺★＝第二圈 rc=6 且分支命中枚数=1 ⇒ ★让路真停第一次有窗内载体★；rc=0 或命中枚数=0 ⇒ 本批仍没喂到那条分支、按在册形只报不认\"\n"
)
s = sub1(s, OLD_SAY, NEW_SAY, "B1-c 自证句")

# ② B4＝把闸式自测整块挪到 roscore 之前（原在 r514 那版排在 roscore 之后 ⇒「开窗前」名实不符）＋标题改＋rc≠0 不起 master
OLD_DT = (
    "# ★R517 新增＝★开窗前必须真跑那件闸式自测、并把 rc 与「不合枚数」随申报行一起打★★（追加九十八① 的触发面字面由 r511q 补齐）\n"
    "DT=\"$R/../../../_prewindow_drytest.sh\"\n"
    "if [ -f \"$DT\" ]; then\n"
    "  DTOUT=$(bash \"$DT\" \"$R/_cmp_r517.py\" 2>&1); DTRC=$?\n"
    "  DTSAM=$(printf '%s\\n' \"$DTOUT\" | grep -oE '不合枚数＝[0-9]+／共 [0-9]+ 枚样本' | tail -1)\n"
    "  say \"== 步1a2 ★开窗前闸式自测★＝本件 rc=$DTRC ‖ $DTSAM（★0／N＝式子能区分真值行与反例行；rc≠0 就不该开窗；被试件 sha 由该件自带★）==\"\n"
    "else\n"
    "  say \"== 步1a2 ★开窗前闸式自测＝采集件不在位（$DT）⇒ 本枚记★未证★、不假装跑过==\"\n"
    "fi\n"
)
assert s.count(OLD_DT) == 1, "B4 源块命中 %d 次（应为 1）" % s.count(OLD_DT)
s = s.replace(OLD_DT, "")   # ★先摘下来★

NEW_DT = (
    "# ★B4（r514q 缺席清单 B4）＝r514 那版「步1a2 开窗前闸式自测」排在 roscore 之后（见本件下方 roscore 那行）⇒「开窗前」相对 master 不成立\n"
    "#   ⇒ 本批整块★前移到 roscore 之前★、标题改「步0a2 四态首跑之前、且 roscore 尚未起」、★rc≠0 则不起 master★\n"
    "DT=\"$R/../../../_prewindow_drytest.sh\"\n"
    "if [ -f \"$DT\" ]; then\n"
    "  DTOUT=$(bash \"$DT\" \"$R/_cmp_r517.py\" 2>&1); DTRC=$?\n"
    "  DTSAM=$(printf '%s\\n' \"$DTOUT\" | grep -oE '不合枚数＝[0-9]+／共 [0-9]+ 枚样本' | tail -1)\n"
    "  say \"== 步0a2 ★闸式自测＝四态首跑之前、且 roscore 尚未起★＝本件 rc=$DTRC ‖ $DTSAM（★0／N＝式子能区分真值行与反例行；被试件 sha 由该件自带★）==\"\n"
    "else\n"
    "  say \"== 步0a2 ★闸式自测＝采集件不在位（$DT）⇒ 本枚记★未证★、不假装跑过==\"\n"
    "  DTRC=9\n"
    "fi\n"
    "if [ \"${DTRC:-9}\" != \"0\" ]; then say \"★闸式自测未过（rc=$DTRC）⇒ 不起 master、不起窗★\"; exit 7; fi\n"
)
ROSCORE = "roscore -p 11313 > \"$RUN/roscore.log\" 2>&1 &\n"
assert s.count(ROSCORE) == 1, "roscore 起行命中 %d 次（应为 1）" % s.count(ROSCORE)
s = s.replace(ROSCORE, NEW_DT + ROSCORE)
print("B4：闸式自测块已前移到 roscore 之前")

# ④ A4＝两枚窗志 sha 之间真插一次 say 写志（r514 那版两读之间无写＝同一瞬间两读恒真）＋随句带行数＋报两枚是否不同
OLD_A = "LOGSHA_BEFORE=$(sha256sum \"$R/00_window_log.txt\" 2>/dev/null | cut -c1-16)\n"
NEW_A = (
    "LOGSHA_BEFORE=$(sha256sum \"$R/00_window_log.txt\" 2>/dev/null | cut -c1-16)\n"
    "LOGLN_BEFORE=$(wc -l < \"$R/00_window_log.txt\")\n"
    "# ★A4（r514q 缺席清单 A4）＝r514 那版两枚 sha 之间★没有任何写窗志的动作★ ⇒ 同一瞬间对同一份字节的两次读、同值恒真 ⇒ 标签「跑前／跑后」名实不符\n"
    "#   ⇒ 本批在两读之间★真写一次窗志★，使两枚分处两次写入的两端；并同句带★该件此刻行数★（r511q:86 点名那枚）\n"
    "say \"  ★A4 插桩＝两枚窗志 sha 之间★真写一次窗志★（写入前窗志行数＝$LOGLN_BEFORE）＝★不插这一句则两读落在同一瞬间★\"\n"
)
s = sub1(s, OLD_A, NEW_A, "A4 插桩")

OLD_S5 = "say \"  ★改形⑤窗志 sha 两枚＝跑前 sha16=$LOGSHA_BEFORE ‖ 跑后 sha16=$INP_SHA（★两枚都只表示各自那一刻、事后不可复现；凡引用它必须带这句★）\"\n"
NEW_S5 = (
    "LOGLN_AFTER=$(wc -l < \"$R/00_window_log.txt\")\n"
    "say \"  ★改形⑤＋A4 窗志 sha 两枚＝跑前 sha16=$LOGSHA_BEFORE（该刻行数＝$LOGLN_BEFORE） ‖ 跑后 sha16=$INP_SHA（该刻行数＝$LOGLN_AFTER） ‖ ★两枚不同＝$([ \\\"$LOGSHA_BEFORE\\\" != \\\"$INP_SHA\\\" ] && echo True || echo False)★ ‖ ★随句带行数＝r511q:86 点名的那枚★（★两枚都只表示各自那一刻、事后不可复现；凡引用它必须带这句★）\"\n"
)
s = sub1(s, OLD_S5, NEW_S5, "A4 报两枚是否不同")

wr(W, s)
print("窗脚本改形落盘：B1-a/b/c、B4、A4 共 5 处")

# ③ B5＝01 件那句拆名（两支共用「rc_复读=5」而正测随后 exit 9 ⇒ 名字＝口径不符）
p = rd(P1)
OLD_B5 = "print(\"   被引件不在位＝%s ⇒ rc_复读=5（★替换自测分支期望的就是这个红★）\" % cited)\n"
NEW_B5 = (
    "if SWAP:\n"
    "        print(\"   被引件不在位＝%s ⇒ rc_复读=5（★5＝替换自测分支＝设计来翻红的那支★）\" % cited)\n"
    "    else:\n"
    "        print(\"   被引件不在位＝%s ⇒ rc_复读=9（★9＝正测前置缺失档★；★B5/R517＝不再与负例共用 5★）\" % cited)\n"
)
p = sub1(p, OLD_B5, NEW_B5, "B5 拆名")

# ⑤ A3＝07 栏形声明不再写旧名「差值枚」（批172 现读 9 枚命中）
OLD_A3 = "栏形＝留底值 ‖ 留底出处窗号＋行号 ‖ 活体复读值 ‖ 差值枚 ‖ 复读时刻"
NEW_A3 = "栏形＝留底值 ‖ 留底出处窗号＋行号 ‖ 活体复读值 ‖ ★值差·不合枚★ ‖ 复读时刻（★A3/R517＝旧名「差值枚」退役★）"
p = sub1(p, OLD_A3, NEW_A3, "A3 栏形声明")
OLD_A3B = "（★R517 改形④＝旧名「差值枚」把★值差★报成★枚数★，两枚并报才分得开）"
NEW_A3B = "（★R517 改形④＋A3/R517＝旧名「差值枚」把★值差★报成★枚数★，两枚并报才分开；本批起栏形声明已改用新名「值差·不合枚」★）"
p = sub1(p, OLD_A3B, NEW_A3B, "A3 说明句")
wr(P1, p)
print("复读件改形落盘：B5、A3（2 处）")

# ⑥ A2＝源面注释补辖域从句
b = rd(PB)
OLD_A2 = "lethal_recount = sum(1 for v in data if v == 100)   # ★独立来源二＝对已发布数组逐格重数★\n"
NEW_A2 = ("lethal_recount = sum(1 for v in data if v == 100)   # ★独立来源二＝对已发布数组逐格重数★"
          "（★A2/R517 辖域从句★＝本件进程内、data 序列化前那一刻的 T 段格集；★不构成对 ROS 传输／订阅面的第二证★）\n")
b = sub1(b, OLD_A2, NEW_A2, "A2 辖域从句")
wr(PB, b)
print("发布器改形落盘：A2")

# ---------- 4. 校验（bash -n ／ ast.parse ／ 件内路径＝盘上路径） ----------
sh = subprocess.run(["bash", "-n", W], capture_output=True, text=True)
assert sh.returncode == 0, "bash -n 未过：\n" + sh.stderr
for py in sorted(os.listdir(DST)):
    if py.endswith(".py"):
        ast.parse(rd(os.path.join(DST, py)), filename=py)
print("bash -n 通过 ‖ ast.parse 通过（%d 枚 py）" % len([f for f in os.listdir(DST) if f.endswith('.py')]))

sw = rd(W)
m = re.search(r'^R=(/opt/ros1-slam/\S+)$', sw, re.M)
assert m, "窗脚本内未找到 R= 路径行"
inner = m.group(1)
host_p = inner.replace("/opt/ros1-slam", "/home/jack/ysc-code/ros1-slam", 1)
assert os.path.isdir(host_p), "件内 R= 路径在宿主面不存在：" + host_p
assert host_p == DST, "件内 R=%s 与盘上目录 %s 不一致" % (host_p, DST)
print("件内 R= 路径＝盘上路径 ✔ ＝%s" % DST_NAME)
assert re.search(r'CLRC\s*=', sw) is None, "镜像重演的 CLRC 硬赋值应已删净（注释里的历史描述不算）"
assert "CLASH" not in sw, "镜像重演的 CLASH 临时目录应已删净"
assert sw.count("Y2HITS") == 2, "Y2HITS 应出现 2 次（赋值＋打印），实为 %d" % sw.count("Y2HITS")
assert 'say "== 步1a2' not in sw, "旧步号 步1a2 的打印行应已改为 步0a2（注释里的历史描述不算）"
assert sw.count("步0a2") >= 1
print("OK 套件建成 · %d 件" % len(os.listdir(DST)))

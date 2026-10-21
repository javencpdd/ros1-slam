#!/usr/bin/env python3
# 批186 第二支＝把★F3／F5／F6／F7／F8／F9★补到★已改名★的 r530 件上（★每枚替换点计数须＝1★）
import io, os, re, shutil, subprocess

RAW = "/home/jack/ysc-code/ros1-slam/note/进度/SCAN集成/证据/log/raw"
DST = os.path.join(RAW, [d for d in os.listdir(RAW) if d.startswith("r530_")][0])
WSH = "00_r530_window.sh"
CMP = "_cmp_r530.py"
RECHECK = "01_复读件_r530.py"


def repl(fn, reps, tag):
    p = os.path.join(DST, fn)
    s = io.open(p, encoding="utf-8").read()
    for old, new in reps:
        n = s.count(old)
        assert n == 1, ("★%s★ 替换点计数＝%d（须＝1）‖ 锚＝%s" % (tag, n, old[:80]))
        s = s.replace(old, new)
    io.open(p, "w", encoding="utf-8").write(s)
    print("  ✔ %s ‖ 替换 %d 处" % (tag, len(reps)))


# ── F3＝对照件：条件句必须挂同一枚条件（r525q S28）──
repl(CMP, [(
 'print("   ★本窗不变量★未行使★（当 INV_DONE=0 时）＝发放形永久带这句★ ‖ 历窗行使面另行全扫、★不得跨窗引用★")',
 'if INV_DONE == 0:\n'
 '    print("   ★本窗不变量★未行使★（INV_DONE=0）＝发放形永久带这句★ ‖ 历窗行使面另行全扫、★不得跨窗引用★")\n'
 'else:\n'
 '    print("   ★本窗不变量★已行使 %d 对且全成立★（INV_DONE＞0 ⇒ ★不得★再打「未行使」＝r525q S28 那枚无条件照打的更正形）‖ 历窗面须现扫、★不得跨窗引用★" % INV_DONE)')], "F3 条件句挂条件")

# ── 窗脚本＝F5／F6／F7a／F7b／F8／F9 ──
repl(WSH, [
 # F6＝两件守卫黑名单同构（r525q S31：件:123 只列 3／6 名 ⇒ 三例逃逸）
 ('case " $YIELD_LIST " in *" run "*|*" 05_cmp.txt "*|*" 00_window_log.txt "*) echo "★S1 守卫不过＝让路清单里还有承重件＝当场中止★" >&2; exit 8;; esac',
  'case " $YIELD_LIST " in *" run "*|*" 05_cmp.txt "*|*" 00_window_log.txt "*|*" 01_preflight.txt "*|*" 03_delta.txt "*|*" 07_复读.txt "*) echo "★S1 守卫不过＝让路清单里还有承重件＝当场中止★" >&2; exit 8;; esac   # ★F6/R530＝与步98 那件★黑名单同构★（r525 只列 3／6 名）★'),
 # F5＝排除式覆盖到★形★＋★元字符正控★（r525 那枚只覆盖一种形、且 + 未转义＝0 命中＝死码）
 ('TOK_TIME="本件时刻＝[0-9-]{10} [0-9:]{8}（★容器面 UTC；宿主 CST＝+8★）"',
  'TOK_TIME="本件时刻＝[0-9]{4}-[0-9]{2}-[0-9]{2} [0-9]{2}:[0-9]{2}:[0-9]{2}（[^）]*）"   # ★F5/R530＝覆盖到★形★＝两种字面形都要命中（r525q S18／S24）★\n'
  'N_APPEAR=$(grep -c "本件时刻＝" "$R/05_cmp.txt"); N_HIT=$(grep -cE "$TOK_TIME" "$R/05_cmp.txt")\n'
  '[ "$N_HIT" -ge 1 ] && [ "$N_HIT" = "$N_APPEAR" ] || echo "★F5 元字符正控不过＝排除式命中 $N_HIT／出现 $N_APPEAR ⇒ 这枚式子★没行使★、下面那个差值不可信★" >&2'),
 ('say "  对照件第一遍 rc=$CRC ‖ ★离窗复跑第二遍 rc=$CRC2★ ‖ 两遍逐行差＝$DIFFN 行（行位口径＝diff 两侧各计一行） ‖ ★S18/R530＝除★时刻 token★ 外逐行差＝$DIFFX 行★',
  'say "  对照件第一遍 rc=$CRC ‖ ★离窗复跑第二遍 rc=$CRC2★ ‖ 两遍逐行差＝$DIFFN 行（行位口径＝diff 两侧各计一行） ‖ ★S18/R530＝除★时刻 token★ 外逐行差＝$DIFFX 行★ ‖ ★F5/R530 三数并报＝出现 $N_APPEAR ‖ 命中 $N_HIT ‖ 排除后差 $DIFFX★（★命中＝出现只是必要条件、还必须与★差集所在行★对表＝r525q S23★）★'),
 # F7a＝$? 连读两枚变量＝第二枚恒 0 ⇒ 改取★变量本身★（r525q S15 问③a）
 ('SRC_FROM=$?',
  'SRC_FROM=$SRC   # ★F7a/R530＝★任何赋值都把 $? 清 0★（两面实证）⇒ 第二枚取★变量本身★、不再取 $?★'),
 ('say "  ★SWAP 遍 进程 rc 回读＝$SRC（★这枚数是从★同一枚★变量 SRC_FROM＝$SRC_FROM 读回来的、不是打印串里的自陈★）',
  'say "  ★SWAP 遍 进程 rc 回读＝$SRC（★F7a/R530＝该数取自★被试命令★的 $?（上一行）、随句带★同一枚变量回读＝$SRC_FROM★；★r525 那句「从 SRC_FROM＝0 读回来」＝恒 0 的自陈＝本批更正、旧志不回改★）'),
 # F9＋F7b＝锚由★拼接★造出（标签不自带被检字面）＋★当场值与事后值并报★
 ('say "  ★S15 形检★＝全志里『从 ★0★ 读回来』字样枚数＝$(grep -c "从 ★0★ 读回来" "$R/00_window_log.txt")（期望 ★0★；★记号含★星号★＝本行不含被检字面＝防自打自证＝追加一百零四① 同族★）"',
  'F7B="SRC""_FROM＝"   # ★F9/R530＝锚串由★拼接★造出 ⇒ 本行字面里★没有★被检串＝「防自打自证」★真的成立★（r525 那版标签自带被检字面＝事后 grep 得 1＝S25）\n'
  'HIT_NOW=$(grep -c "$F7B" "$R/00_window_log.txt" || true)\n'
  'say "  ★F7b／F9/R530 志面语义锚★＝全志含那枚★变量赋值记号★的行数＝$HIT_NOW（期望 ★0★＝件:328 已改取变量本身；★旧版锚的是字面句＝换写法即逃逸★）"\n'
  'HIT_AFTER=$(grep -c "$F7B" "$R/00_window_log.txt" || true)\n'
  'say "  ★F9/R530＝同一枚检查★事后复现★＝当场值 $HIT_NOW ‖ 落盘后复现值 $HIT_AFTER★（★两值并报＝时点读数不许写成状态；★只报不拦★）"'),
 # F8＋S16＝守卫枚数★现取★（道数≠码位、式子随句、锚用拼接）
 ('say "  ★S1 两道守卫★＝①让路清单临时只含★一枚★探针件（不止一枚⇒exit 8）②★跑完后窗志与 run/ 仍在位★（被动走⇒exit 8）＝走到下面这一行＝★两枚守卫都过★"',
  'E8C="exit"" 8"; N8=$(grep -c "$E8C" "$0" || true)\n'
  'say "  ★S1 守卫★＝语义★道数 2★（①让路清单临时只含★一枚★探针件 ②跑完后窗志与 run/ 仍在位）‖ ★F8/R530＝本件 exit-8 ★码位★枚数现取＝$N8★（式子＝grep -c 拼接锚、标签不自带被检串；★道数≠码位★＝r525q 问⑥d 三数并报那格；其中一枚是★非让路族★的 master 未起）‖ 走到这一行＝两枚★道★都过"'),
 # F6 自测＋F8 辖域句
 ('say "  ★步98 判读尺★＝rc=6 且 前后指纹相同 ⇒ ★承重名上的撞名让路第一次有窗内载体、且未损坏留底★',
  'BL_TEST=0; for nm in 01_preflight.txt 03_delta.txt 07_复读.txt; do case " $nm " in *" run "*|*" 05_cmp.txt "*|*" 00_window_log.txt "*|*" 01_preflight.txt "*|*" 03_delta.txt "*|*" 07_复读.txt "*) BL_TEST=$((BL_TEST+1));; esac; done\n'
  'say "  ★F6/R530 黑名单同构★自测★＝把★上一版会逃逸★的三枚名逐个喂进★同构后★的 case ⇒ 拦住枚数＝$BL_TEST（期望 ★3★；★壳层自测、不改任何 rc 语义★）"\n'
  'say "  ★F8/R530 步98 辖域句★＝真目录载体只★一枚★承重名（05_cmp.txt）的★停支★；另两枚承重名在真目录撞过＝★0 枚★、mv 支在真目录＝★0 枚★（★r525q 问⑥b 裁＝★不为凑数在真目录跑三遍★；镜像另证且★镜像≠正本★）★；★且★本步那枚 .prev_ 是本步自造的预置件★ ⇒「真目录历史遗留引发的撞名」＝0 枚＝S32★"\n'
  'say "  ★步98 判读尺★＝rc=6 且 前后指纹相同 ⇒ ★承重名上的撞名让路第一次有窗内载体、且未损坏留底★'),
], "窗脚本 F5／F6／F7／F8／F9")

# ── F4＝判读面★查不到★要有红档（新建一枚离窗判读件；不留崩／None 上路的形）──
# ── 套件本体换成★本批两支★的副本（_kit 自述＝生成件而非旧批那枚）──
os.remove(os.path.join(DST, "_kit_r530.py"))
shutil.copyfile("/tmp/r530_kit.py", os.path.join(DST, "_kit_r530.py"))
shutil.copyfile("/tmp/r530_fix.py", os.path.join(DST, "_kit_r530_fix.py"))

# ── 四段自证③④＝语法＋锚点全命中＋新自测件单跑（正控＋反控，只在 /tmp）──
sh = os.path.join(DST, WSH)
r = subprocess.run(["bash", "-n", sh], capture_output=True, text=True)
assert r.returncode == 0, ("bash -n 不过", r.stderr[:300])
for f in os.listdir(DST):
    if f.endswith(".py"):
        import ast
        ast.parse(io.open(os.path.join(DST, f), encoding="utf-8", errors="replace").read())
print("  ✔ bash -n rc=0 ‖ 全部 .py ast.parse 过 ‖ 窗脚本行数＝", io.open(sh, encoding="utf-8").read().count("\n"))
T = "/tmp/r530_selftest"
os.makedirs(T, exist_ok=True)
# 正控 1＝F5 新式必须★两种形都命中★
with io.open(os.path.join(T, "two_forms.txt"), "w", encoding="utf-8") as f:
    f.write("甲 本件时刻＝2026-10-20 22:23:15（★容器面 UTC；宿主 CST＝+8★）\n乙 本件时刻＝2026-10-20 22:23:48（★容器面 UTC★）\n")
tok = '本件时刻＝[0-9]{4}-[0-9]{2}-[0-9]{2} [0-9]{2}:[0-9]{2}:[0-9]{2}（[^）]*）'
hit = subprocess.run(["bash", "-c", 'grep -cE "%s" %s' % (tok, os.path.join(T, "two_forms.txt"))],
                     capture_output=True, text=True).stdout.strip()
old_hit = subprocess.run(["bash", "-c", 'grep -cE "本件时刻＝[0-9-]{10} [0-9:]{8}（★容器面 UTC；宿主 CST＝\\+8★）" %s || true' % os.path.join(T, "two_forms.txt")],
                         capture_output=True, text=True).stdout.strip()
print("  ✔ F5 正控＝两种形都命中枚数 %s（期望 2）‖ ★反控＝r525 那枚（含 ★\\+ 也仍只）命中 %s（期望 1）★" % (hit, old_hit))
assert hit == "2", "★F5 新式没覆盖到形★"
# 正控 2＝F7a／F7b 语义锚必须抓到★旧形★、放过★新形★
anc = subprocess.run(["bash", "-c", r'''printf 'SRC=$?\nSRC_FROM=$?\n' > /tmp/r530_selftest/old_form.sh; printf 'SRC=$?\nSRC_FROM=$SRC\n' > /tmp/r530_selftest/new_form.sh; awk 'p && /^[[:space:]]*[A-Za-z_]+=\?[[:space:]]*$/{c++} /^[[:space:]]*[A-Za-z_]+=\$\?[[:space:]]*$/{p=1} END{print c+0}' /tmp/r530_selftest/old_form.sh; awk 'p && /^[[:space:]]*[A-Za-z_]+=\?[[:space:]]*$/{c++} /^[[:space:]]*[A-Za-z_]+=\$\?[[:space:]]*$/{p=1} END{print c+0}' /tmp/r530_selftest/new_form.sh'''],
                     capture_output=True, text=True).stdout.split()
print("  ✔ F7a 语义锚正控（旧形）＝命中 %s（期望 ≥1）‖ 反控（新形）＝命中 %s（期望 0）" % (anc[0], anc[1]))
assert int(anc[0]) >= 1 and int(anc[1]) == 0, "★F7a 语义锚没牙或恒红★"
# 正控 3＝F9 拼接锚：标签行★不★含被检串
lab = 'F7B="SRC""_FROM＝"; printf "   ★志面语义锚★＝全志含那枚变量赋值记号的行数＝$(grep -c \\"$F7B\\" x)（期望 0）\n" | grep -c "$F7B" || true'
got = subprocess.run(["bash", "-c", lab], capture_output=True, text=True).stdout.strip()
print("  ✔ F9 正控＝拼接锚对★自己的标签行★命中枚数 %s（期望 0＝标签不自带被检字面）" % got)
assert got == "0", "★F9 标签仍自带被检串★"
# 正控 4＝F6 同构后的 case 必须拦住三枚名
bl = subprocess.run(["bash", "-c", '''c=0; for nm in 01_preflight.txt 03_delta.txt 07_复读.txt; do case " $nm " in *" run "*|*" 05_cmp.txt "*|*" 00_window_log.txt "*|*" 01_preflight.txt "*|*" 03_delta.txt "*|*" 07_复读.txt "*) c=$((c+1));; esac; done; echo $c'''],
                    capture_output=True, text=True).stdout.strip()
print("  ✔ F6 正控＝同构后 case 拦住枚数 %s（期望 3）" % bl)
assert bl == "3", "★F6 黑名单仍不同构★"
shutil.rmtree(T)
print("新目录＝", DST)
for f in sorted(os.listdir(DST)):
    fp = os.path.join(DST, f)
    print("     %-34s ‖ %d 字节 ‖ %d 行" % (f, os.path.getsize(fp), io.open(fp, encoding="utf-8", errors="replace").read().count("\n")))

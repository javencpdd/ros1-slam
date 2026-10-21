#!/usr/bin/env bash
# _prewindow_drytest.sh —— ★开窗前的「闸式」两组合样自测件★（施工侧自有件、★不是闸★：不进任何在册件的 rc、不入门禁串）
# 起意＝R511 批167 改合取五时，第一版（按｜分 token、每 token 只认一键）把★真窗志退净行判成 False★＝一枚施工侧自造的约束，
#         我是在★开窗之前★手工拿六行样本跑出来才拦住的（在册姿势＝追加九十八①「改完一枚闸先拿真值行＋反例行各跑一遍、时机在开窗之前」）；
#         手工一次＝下次还会漏 ⇒ 本件把它做成常驻可跑的东西。
# 姿势＝★不抄字面★：被试正则★从件里现读★（抓 `g5 = dict(re.findall(r"..."))` 那一行的原始串），件改了式子、本件跟着变；
#       样本按「行＋期望」写死成表，★真值行必须判成立、反例行必须判不成立★，任一不合即本件 rc=5。
# 用法＝bash _prewindow_drytest.sh [被试件路径] [--break]
#   --break ＝★常驻负例分支★：故意改用旧的无边界式子喂同一批样本，必须当场报出「不合」枚数＞0 且本件 rc=5
#             （＝证明这件自己有牙，不是只会一路绿灯的过场）。
set -u
ROOT=$(cd "$(dirname "$0")/../../../.." && pwd)
CMP="$ROOT/note/进度/SCAN集成/证据/log/raw/r511_活体窗八枚件内改形_合取五带边界解析与死赋值删与正向红写进注释与差值改名与SWAP窗内载体与撞名窗内自证与ROS主口径回名册与复跑报sha/_cmp_r511.py"
BRK=0
for a in "$@"; do
  case "$a" in
    --break) BRK=1 ;;
    *) CMP="$a" ;;
  esac
done
[ -f "$CMP" ] || { echo "★被试件不在位＝$CMP ⇒ 本件记不可证、rc=9★"; exit 9; }
echo "== 开窗前闸式两组合样自测（★只报不拦：不进任何在册件的 rc；串仍 10 件★）=="
echo "   被试件＝${CMP#$ROOT/} ‖ sha 前 16＝$(sha256sum "$CMP" | cut -c1-16) ‖ 模式来源＝★从件里现读、不抄字面★"
python3 - "$CMP" "$BRK" <<'PY'
import re, sys
src, brk = open(sys.argv[1], encoding='utf-8').read(), sys.argv[2] == '1'
m = re.search(r'g5 = dict\(re\.findall\(r"([^"]+)"', src)
assert m, '★没在被试件里抓到合取五的解析式 ⇒ 本件不可证、不许当通过★'
pat = m.group(1)
if brk:
    pat = r"(D|K|TCP|UDP)=(-?\d+)"          # ★旧式（无边界、基线不入式）＝负例分支专用★
    print('   ★--break＝改用★旧★无边界式子喂同一批样本（期望：至少一枚样本判得不合）★')
print('   现读式子＝"%s"' % pat)
RX = re.compile(pat)
def judge(line):
    g = dict(RX.findall(line))
    return (g.get('D') == g.get('K') and g.get('D∖K') == '0' and g.get('TCP') == '0'
            and g.get('UDP') == '0' and g.get('基线') is not None), g
SAMP = [
    ('真窗志退净行', '  ★判定行五数＝基线=0｜D=0｜K=0｜D∖K=0｜端口 TCP=0 UDP=0★（现读 14 名计数=0） TIDY5=OK', True),
    ('反例·D≠K 而 D∖K 顶替', '基线=0｜D=5｜K=0｜D∖K=5｜端口 TCP=0 UDP=0', False),
    ('反例·K 非零被顶替', '基线=0｜D=2｜K=1｜D∖K=2｜端口 TCP=0 UDP=0', False),
    ('反例·D∖K 非零', '基线=0｜D=5｜K=5｜D∖K=3｜端口 TCP=0 UDP=0', False),
    ('反例·基线缺席', 'D=0｜K=0｜D∖K=0｜端口 TCP=0 UDP=0', False),
    ('反例·端口非零', '基线=0｜D=0｜K=0｜D∖K=0｜端口 TCP=1 UDP=0', False),
]
bad = 0
for name, line, exp in SAMP:
    got, g = judge(line)
    ok = (got == exp)
    bad += 0 if ok else 1
    print('   [%s] %-22s 解析＝%-58s 判＝%-5s 期望＝%-5s ⇒ %s' % (
        '合' if ok else '★不合★', name, g, got, exp, '符合' if ok else '★不符★'))
print('   ★不合枚数＝%d／共 %d 枚样本★（★0＝这批闸式能区分真值行与反例行；＞0＝别开窗，先修式子★）' % (bad, len(SAMP)))
sys.exit(0 if bad == 0 else 5)
PY
RC=$?
if [ "$BRK" = "1" ]; then
  if [ "$RC" = "5" ]; then echo "  ★负例分支如期望翻红＝本件 rc=5（这枚自测件有牙）★"; exit 0;
  else echo "  ★负例分支没翻红＝本件 rc=$RC ⇒ 这件是假绿、不许当通过★"; exit 5; fi
fi
echo "  本件 rc=$RC（0＝全部样本符合期望；5＝有样本不合；9＝前置缺失）；★只报不拦★"
exit "$RC"

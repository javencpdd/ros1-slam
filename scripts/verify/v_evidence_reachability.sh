#!/usr/bin/env bash
# v_evidence_reachability.sh (VAR) —— 「证据可达性」巡检件（**候选闸，未入门禁串**；小明 2026-10-10，源于 02 坑59）
#
# 为何存在：R96 普查发现我落盘的 21 件是 mode 600，且虽带 `user:ros1-slam:rwx` 条目却被 `mask::---` 掐空
#   （`#effective:---`）⇒ 协作账号点开是 Permission denied。这类失效**不会让任何现有门禁变红**，
#   却直接废掉四要素第四格「证据路径」——证据存在 ≠ 证据可达。本件把"可达"做成可复跑判定。
#
# 断言顺序写死（阳性对照在前；05 B2／C4b／坑29 纪律）：
#   P1 探针自证有分辨力：在 /tmp 造两件——①600＋条目＋`m::---`（应判**不可读**）②664＋同条目（应判**可读**）。
#      两条都符合期望才 P1 过：只认红不认绿、或只认绿不认红，都说明算法不可信（P1 不过 ⇒ 后面全不采信，直接 rc=9）。
#   P2 普查面：roots 内**每一个普通文件**按 POSIX 算法算协作账号的生效权限；既不可读又无执行需要者 ⇒ 记违规并点名。
#      粗筛用 os.stat（不启子进程），仅对"组位/其他位看着不对"的件再 `getfacl` 精判条目与 mask ⇒ 全量而不慢。
#   P3 可验面：逐证据目录核 ①含 `_sha256_manifest.txt` ②从归档址 `sha256sum -c` 全 OK ③目录内件对协作账号可读。
#      ⇒ 把"可达"与"可验"绑在一起查，这正是第四格的两半。
#
# 边界：**只读普查**——本件不 chmod／不 setfacl 任何仓内件（改权限属施工动作，由人做并同批复查 effective）。
#   也不进 v3_isolation.sh：新判据入共同门禁属判据面变更，依 05 A3 须验收先落笔；入不入归 A15/#54/A16 并案。
# 用法：bash scripts/verify/v_evidence_reachability.sh [根 ...]（缺省三根＝note ws/src scripts）
#   环境变量：VAR_USERS="ros1-slam ysc"（要保证的协作账号）｜VAR_EVD（证据目录 glob，缺省 note/进度/*/证据/log/raw/*/）
#             VAR_ROOT（仓根，缺省 /home/ros1-slam；★R574 补，异机须覆写）
#             VAR_MAX（明细最多列几件，缺省 25）｜VAR_SECRETS（保密件 glob，空格分隔；缺省见代码）
#   保密面（P4）是**反向断言**：列在 VAR_SECRETS 里的件**必须** other 位不给读——本件绝不建议也不改动它们放开，
#     且普查时**只 stat 不 open**（密码/凭据类件的正文在本件任何路径下都不会被读）。
#   退出码：0 全过｜5 有 FAIL｜9 前置缺失（根不存在／普查面为空／getfacl 不可用／P1 未过）
# 性质：零 ROS 节点、零链窗、零仓内写（临时件只在 /tmp 且跑完即删）、无 sudo。
set -u
# ★R574（批226）＝仓根补可覆写口：原第 26 行 `cd /home/ros1-slam || exit 9` 把仓根写死，
#   在异机（本仓／容器：该路径是符号链接视图或根本不存在）必 rc=9 ⇒ 串内 VAR 恒红，与件本身对不对无关。
#   修法与 VCC（v_citation_check.sh:36 的 VCC_ROOT）、账构（v_ledger_batch_consistency.sh:38 的 VLB_ROOT）同款：
#   ★只补覆写口、缺省值一字不改、rc 语义与普查面一字不改★。
ROOT="${VAR_ROOT:-/home/ros1-slam}"
cd "$ROOT" || { echo "[前置缺失] 仓根不可达：$ROOT（可用 VAR_ROOT=仓根 覆写）"; echo "VAR VERDICT rc=9"; exit 9; }
command -v getfacl >/dev/null 2>&1 || { echo "[前置缺失] getfacl 不可用 ⇒ 无法判 effective"; echo "VAR VERDICT rc=9"; exit 9; }
SELF=$(readlink -f "$0")
# --selftest：把"能红也能绿"打包成例（R187；来源＝本会话 VAR 逮到我 7 次，但它此前只有内建 P1 探针、没有可复跑的 P2/P3 用例）
#   每条守卫都必须**两面都过**：植入违规必须红、撤掉违规必须绿；只红不绿＝恒红噪声，只绿不红＝空控。
if [ "${1:-}" = "--selftest" ]; then
  npass=0; nfail=0
  T1=$(mktemp -d /tmp/var_st_perms.XXXXXX); T2=$(mktemp -d /tmp/var_st_evd.XXXXXX)
  printf 'y\n' > "$T1/keep664.txt"; chmod 664 "$T1/keep664.txt"
  mkdir -p "$T2/r99_probe"; printf 'z\n' > "$T2/r99_probe/out.txt"; chmod 664 "$T2/r99_probe/out.txt"

  # 关键：测 P2 时必须把 P3 也喂一个**合规**的证据目录（①glob 指空 ⇒ 普查面为空 ⇒ rc=9 前置缺失；
  #   ②不指 ⇒ 仓里任一未建清单的目录把两面一起染红 ⇒ S2 误判"P2 恒红"。两种都被本自测的 S3 抓到过，见 04 R189）
  mkdir -p "$T1/ev"; printf 'e\n' > "$T1/ev/item.txt"; chmod 664 "$T1/ev/item.txt"
  ( cd "$T1/ev" && sha256sum item.txt > _sha256_manifest.txt && chmod 664 _sha256_manifest.txt )
  P3ENV="VAR_EVD=$T1/ev/"
  printf 'x\n' > "$T1/bad600.txt"; chmod 600 "$T1/bad600.txt"
  o1=$(env "$P3ENV" bash "$SELF" "$T1" 2>&1); r1=$?
  if [ "$r1" = 5 ] && printf '%s' "$o1" | grep -q 'bad600.txt'; then
    npass=$((npass+1)); echo "[PASS] S1 P2 植入 600 件必须点名并 rc=5（实得 rc=$r1）"
  else nfail=$((nfail+1)); echo "[FAIL] S1 P2 未逮住植入的 600 件（rc=$r1）⇒ P2 没有分辨力"; fi
  rm -f "$T1/bad600.txt"
  o2=$(env "$P3ENV" bash "$SELF" "$T1" 2>&1); r2=$?
  if [ "$r2" = 0 ] && ! printf '%s' "$o2" | grep -q '^\[FAIL\]'; then npass=$((npass+1)); echo "[PASS] S2 撤掉违规后必须 rc=0 且无 FAIL 行（实得 rc=$r2）"
  else nfail=$((nfail+1)); echo "[FAIL] S2 撤掉违规后仍 rc=$r2 ⇒ 探针被别的检查污染或 P2 恒红：$(printf '%s' "$o2" | grep '^\[FAIL\]' | head -1)"; fi
  [ "$r1" != "$r2" ] && { npass=$((npass+1)); echo "[PASS] S3 P2 两面不同（$r1 ≠ $r2）⇒ 红确由违规而来"; } \
    || { nfail=$((nfail+1)); echo "[FAIL] S3 P2 两面同果（都 $r1）⇒ 该变异例无效，不得据此说 P2 被测过"; }

  o4=$(VAR_EVD="$T2/*/" bash "$SELF" "$T2" 2>&1); r4=$?
  if [ "$r4" = 5 ] && printf '%s' "$o4" | grep -q 'r99_probe'; then
    npass=$((npass+1)); echo "[PASS] S4 P3a-1 缺清单的证据目录必须被报并 rc=5（实得 rc=$r4）"
  else nfail=$((nfail+1)); echo "[FAIL] S4 P3a-1 未逮住缺清单目录（rc=$r4）⇒ 约定回归面没有牙"; fi
  ( cd "$T2/r99_probe" && sha256sum out.txt > _sha256_manifest.txt && chmod 664 _sha256_manifest.txt )
  o5=$(VAR_EVD="$T2/*/" bash "$SELF" "$T2" 2>&1); r5=$?
  if [ "$r5" = 0 ]; then npass=$((npass+1)); echo "[PASS] S5 补上清单后必须 rc=0（实得 rc=$r5）"
  else nfail=$((nfail+1)); echo "[FAIL] S5 补清单后仍 rc=$r5 ⇒ P3a-1 恒红：$(printf '%s' "$o5" | grep FAIL | head -1)"; fi
  [ "$r4" != "$r5" ] && { npass=$((npass+1)); echo "[PASS] S6 P3a-1 两面不同（$r4 ≠ $r5）"; } \
    || { nfail=$((nfail+1)); echo "[FAIL] S6 P3a-1 两面同果（都 $r4）⇒ 该变异例无效"; }

  rm -rf "$T1" "$T2"
  echo "VAR SELFTEST VERDICT: PASS=$npass FAIL=$nfail（每条守卫都必须既会红又会绿；只红＝恒红噪声、只绿＝空控）"
  [ "$nfail" = 0 ] && exit 0 || exit 5
fi
set +u
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION
set -u
python3 - "${VAR_USERS:-ros1-slam ysc}" "${VAR_EVD:-note/进度/*/证据/log/raw/*/}" "${VAR_MAX:-25}" "${1:-}" "${2:-}" "${3:-}" <<'PYEOF'
# -*- coding: utf-8 -*-
import fnmatch, glob, os, pwd, re, grp, subprocess, sys, tempfile

USERS = sys.argv[1].split()
SECRETS = ['note/*/证据/*密码*', 'note/*/*密码*', '.ssh/*', '*_token*', '*secret*']
EVD_GLOBS = sys.argv[2]
MAXD = int(sys.argv[3])
roots = [a for a in sys.argv[4:7] if a] or ['note', 'ws/src', 'scripts']
PY = os.path.dirname(os.path.abspath(__file__)) if '__file__' in dir() else ''
npass = nfail = 0


def mark(ok, name, detail=''):
    global npass, nfail
    if ok:
        npass += 1
        print('[PASS] %s%s' % (name, ('：' + detail) if detail else ''))
    else:
        nfail += 1
        print('[FAIL] %s%s' % (name, ('：' + detail) if detail else ''))


def parse_mode(bits):
    return ''.join(c if bits & m else '-' for c, m in (('r', 4), ('w', 2), ('x', 1)))


def acl_of(path):
    """getfacl → (named_users dict, named_groups dict, mask or None)。失败返回 None。"""
    try:
        out = subprocess.check_output(['getfacl', '--absolute-names', '-p', path],
                                      stderr=subprocess.DEVNULL).decode('utf-8', 'ignore')
    except Exception:
        return None
    nu, ng, mask = {}, {}, None
    for line in out.splitlines():
        if line.startswith('#') or not line.strip():
            continue
        parts = line.split(':')
        if len(parts) == 2 and parts[0] in ('user', 'group', 'other', 'mask'):
            if parts[0] == 'mask':
                mask = parts[1]
        elif len(parts) == 3 and parts[0] == 'user' and parts[1]:
            nu[parts[1]] = parts[2]
        elif len(parts) == 3 and parts[0] == 'group' and parts[1]:
            ng[parts[1]] = parts[2]
    return nu, ng, mask


def readable(path, who, cache={}):
    """POSIX 生效权限：owner 位 → 具名 user 条目&mask → 组（含具名组）&mask → other 位。
       粗筛先走 os.stat（不启子进程）；只有 other 位不给读时才 getfacl 精判 ⇒ 全量而不慢。"""
    st = os.stat(path)
    if who not in cache:
        try:
            pu = pwd.getpwnam(who)
            cache[who] = (pu.pw_uid, set([pu.pw_gid] + [g.gr_gid for g in grp.getgrall() if who in g.gr_mem]))
        except KeyError:
            return None
    uid, gids = cache[who]
    obits, gbits, mbits = parse_mode(st.st_mode >> 6), parse_mode(st.st_mode >> 3), parse_mode(st.st_mode & 7)
    if st.st_uid == uid:
        return 'r' in obits                      # owner 位不受 mask 影响，可当场定案
    if 'r' in mbits:
        return True                              # other 位给读 ⇒ 与 mask 无关，可当场定案
    # 到这儿只剩"other 不给读"的件（正常仓里应当极少）：要判具名条目与 mask，必须读 getfacl
    nu, ng, mask = (acl_of(path) or ({}, {}, None))
    m = mask if mask is not None else 'rwx'
    if who in nu:
        return 'r' in ''.join(c for c in nu[who] if c in m)
    if st.st_gid in gids and 'r' in ''.join(c for c in gbits if c in m):
        return True
    for gname, perm in ng.items():
        try:
            if grp.getgrnam(gname).gr_gid in gids and 'r' in ''.join(c for c in perm if c in m):
                return True
        except KeyError:
            pass
    return False


# ---------- P1 阳性对照（先证探针有分辨力，再谈零命中） ----------
tmpd = tempfile.mkdtemp(prefix='var_p1_')
good = os.path.join(tmpd, 'should_be_reachable')
bad = os.path.join(tmpd, 'should_be_unreachable')
res = []
try:
    for f, mode in ((good, 0o664), (bad, 0o600)):
        open(f, 'w').write('x\n')
        os.chmod(f, mode)
        subprocess.call(['setfacl', '-m', 'u:%s:rw-' % USERS[0], f])
        if f is bad:
            subprocess.call(['setfacl', '-m', 'm::---', f])   # 把 mask 掐空＝条目存在但 effective 为空
    r_good, r_bad = readable(good, USERS[0]), readable(bad, USERS[0])
    mark(r_good is True, 'P1a 阳性对照·可达侧：664＋具名条目 ⇒ 算法须判可读', '实得=%s' % r_good)
    mark(r_bad is False, 'P1b 阳性对照·不可达侧：600＋同条目但 mask::--- ⇒ 算法须判不可读', '实得=%s' % r_bad)
finally:
    subprocess.call(['rm', '-rf', tmpd])
if nfail > 0:
    print('VAR VERDICT rc=9（P1 未过＝探针无分辨力，**普查结果一律不采信**，不判被试）')
    sys.exit(9)

# ---------- P2 普查面 ----------
def is_secret(path):
    rel = os.path.relpath(path, '.')
    return any(fnmatch.fnmatch(rel, g) or fnmatch.fnmatch(os.path.basename(rel), g)
               for g in SECRETS)


secret_hits = []
files, skipped_pyc = [], 0
for r in roots:
    for dirpath, dirnames, filenames in os.walk(r):
        if '__pycache__' in dirpath:
            continue
        for fn in filenames:
            p = os.path.join(dirpath, fn)
            if fn.endswith('.pyc'):
                skipped_pyc += 1
                continue
            if is_secret(p):
                secret_hits.append(p)
                continue        # 保密件走 P4 反向断言，绝不进 P2（否则等于劝人放开权限）
            files.append(p)
if not files:
    print('VAR VERDICT rc=9（普查面为空＝没得扫，不判被试；坑29）')
    sys.exit(9)
unreadable = {}
for p in files:
    for who in USERS:
        try:
            if readable(p, who) is False:
                unreadable.setdefault(p, []).append(who)
        except Exception as e:
            unreadable.setdefault(p, []).append('%s(计算异常:%s)' % (who, type(e).__name__))
items = sorted(unreadable.items())
mark(not items,
     'P2 普查面：roots=%s 内 %d 件对协作账号 %s 全部可读（排除 %d 件 __pycache__/*.pyc＝可再生构建产物）'
     % (','.join(roots), len(files), '/'.join(USERS), skipped_pyc),
     ('违规 %d 件：%s' % (len(items), '; '.join('%s[%s]' % (k, ','.join(v)) for k, v in items[:MAXD]))) if items else '零违规')

# ---------- P3 可验面 ----------
evdirs = sorted(d for d in glob.glob(EVD_GLOBS) if os.path.isdir(d))
if not evdirs:
    print('VAR VERDICT rc=9（证据目录匹配为空＝没得扫，不判被试）')
    sys.exit(9)
# 清单面分三类，只有真风险那两类判 FAIL（R48 已把旧式清单的缺陷登记在册，一律不覆写）：
#   ①内容不符（哈希对不上）＝完整性风险；②新目录（清单 mtime ≥ 约定日）不可验＝我们自己的回归；
#   ③旧目录条目缺 out/ 前缀之类跑不通＝**已知态**，只计数并指名，等"旁挂 addendum"那批活收（不据此判红）。
# 判"该不该守清单约定"用**目录名里的 R 号或日期**，不用 mtime：
#   mtime 会被后续权限维护、旁挂 addendum、mv 清单入位等动作改掉（本轮实测就会把 10-08 早上的目录误判成"新目录"），
#   而目录名是落盘时人写的、正是约定的载体。约定出自 R48（2026-10-08 19:05 清单整体重建）。
R_MIN = int(os.environ.get('VAR_R_MIN', '48'))        # rNN_ 前缀的目录：R 号 ≥48 视为"应守约定"
DATE_MIN = os.environ.get('VAR_DATE_MIN', '20261009')  # 名字带 2026MMDD 的目录：日期 ≥该值视为新


def is_new_era(path):
    name = os.path.basename(path.rstrip('/'))
    m = re.match(r'^r(\d+)[_\-]', name)
    if m:
        return int(m.group(1)) >= R_MIN
    m = re.search(r'(2026\d{4})', name)
    if m:
        return m.group(1) >= DATE_MIN
    return False


CUTOFF = 0
nomani_new, nomani_old, mismatch, unverif_new, legacy = [], [], [], [], []
def _verifies(m, base_dir):
    rel = os.path.relpath(m, base_dir)
    rr = subprocess.run(['sha256sum', '-c', rel], cwd=base_dir, capture_output=True, text=True)
    return any(ln.endswith(': OK') for ln in (rr.stdout + rr.stderr).splitlines())

def run_cwd(base_dir, rel):
    rr = subprocess.run(['sha256sum', '-c', rel], cwd=base_dir, capture_output=True, text=True)
    lns = (rr.stdout + rr.stderr).splitlines()
    return rr, sum(1 for ln in lns if ln.endswith(': OK')),\
           sum(1 for ln in lns if ln.endswith(': FAILED')),\
           sum(1 for ln in lns if 'could not be read' in ln), lns

MANI_NAMES = ['_sha256_manifest.txt', '_manifest_all.txt', '_manifest_recheck.txt']
# 清单件名允许三个在册别名：正名 _sha256_manifest.txt；_manifest_all.txt（R50 那批）；
#   _manifest_recheck.txt（R48 旁挂、R98 推广）。**基准目录也有两种**：本目录／父目录（raw/）。
#   做法＝把"每个清单名 × 每个基准"都试一遍，取 **OK 条数最多**的那一组合；都跑不通才算不可验。
#   为什么不是"取第一个存在的"：182329 那种目录同时有正名（旧式不可验）与旁挂 recheck（可验），
#   只认第一个就会把已补齐的目录永远报成已知态——补建工作对巡检件隐形（本轮实测就是这个）。
empty_dirs = []
for d in evdirs:
    present = [n for n in MANI_NAMES if os.path.isfile(os.path.join(d, n))]
    parent = os.path.dirname(os.path.abspath(d)) or '.'
    best = None
    for n in present:
        for base, rel in ((d, n), (parent, os.path.join(os.path.basename(d.rstrip('/')), n))):
            rr, k_ok, k_mis, k_miss, lns = run_cwd(base, rel)
            if best is None or k_ok > best[1]:
                best = (n, k_ok, k_mis, k_miss, base, rel, rr, lns)
    nfiles = sum(len(fns) for _, _, fns in os.walk(d))
    if nfiles == 0:
        empty_dirs.append(d)
    if not present:
        (nomani_new if is_new_era(d) else nomani_old).append(d)
        continue
    nmis_total = best[2]
    if best[0] != MANI_NAMES[0]:
        print('[别名] %s 用的是 %s（正名＝%s）' % (d.split('/')[-2], best[0], MANI_NAMES[0]))
    if os.path.abspath(best[4]) != os.path.abspath(d):
        print('[基准] %s 的清单 %d 条从**父目录**跑通 ⇒ 旁挂 addendum 那族（04:405 复核命令原文如此）'
              % (d.split('/')[-2], best[1]))
    nok, nmis, nmiss = best[1], best[2], best[3]
    lines = best[7]
    r = best[6]
    new_ish = is_new_era(d)
    if nmis > 0:
        mismatch.append('%s(内容不符=%d OK=%d)' % (d, nmis, nok))
    elif nmiss > 0 or nok == 0:
        (unverif_new if new_ish else legacy).append('%s(读不到=%d OK=%d)' % (d, nmiss, nok))
    elif r.returncode != 0:
        (unverif_new if new_ish else legacy).append('%s(rc=%d)' % (d, r.returncode))
mark(not nomani_new, 'P3a-1 约定日之后落盘的证据目录**必须**有 _sha256_manifest.txt（这是自家约定的回归面，缺＝红）',
     ('缺：%s' % '; '.join(x.split('/')[-2] for x in nomani_new[:MAXD])) if nomani_new else '无缺')
print('[已知态] P3a-2 早于清单约定的目录缺清单：%d 个%s（只登记，不据此判红——它们本来就在约定之前）'
      % (len(nomani_old), '；' + '; '.join(x.split('/')[-2] for x in nomani_old[:MAXD]) if nomani_old else ''))
mark(not mismatch, 'P3b-1 完整性：无任何目录出现「哈希对不上」（这一类是真风险，必红）',
     ('内容不符：%s' % '; '.join(mismatch[:MAXD])) if mismatch else '零件')
mark(not unverif_new, 'P3b-2 新目录可验面：清单 mtime≥约定日（R48 起）的目录必须从归档址 -c 全 OK 且 OK 计数≥1',
     ('不可验：%s' % '; '.join(unverif_new[:MAXD])) if unverif_new else '新目录全部可验')
print('[已知态] P3b-3 旧式清单不可验（条目缺前缀那族，R48 已登记、补不覆）：共 %d 个目录%s'
      % (len(legacy), '；' + '; '.join(x.split('/')[-2] for x in legacy[:MAXD]) if legacy else ''))
mark(not empty_dirs, 'P3c 证据树内不该有 0 件的目录（空壳既不可验也误导读者）',
     ('空目录：%s' % '; '.join(x.split('/')[-2] for x in empty_dirs[:MAXD])) if empty_dirs else '无空壳')
print('[计数] 证据目录=%d 违规件=%d 保密件=%d 排除 .pyc=%d' % (len(evdirs), len(items), len(secret_hits), skipped_pyc))

# ---------- P4 保密面（反向断言：这些件必须对 other 关死；本件只 stat、不 open） ----------
loose = []
for p in secret_hits:
    if parse_mode(os.stat(p).st_mode & 7)[0] == 'r':
        loose.append(p)
mark(not loose, 'P4 保密面：%d 件列入 VAR_SECRETS 的件，other 位均不给读（**只 stat 不 open**）' % len(secret_hits),
     ('过松：%s' % '; '.join(loose[:MAXD])) if loose else ('清单=%s' % ' '.join(SECRETS[:2])))

print('==================================================')
print('VAR VERDICT: PASS=%d FAIL=%d%s' % (npass, nfail,
      '（P2 违规 %d 件）' % len(items) if items else ''))
sys.exit(0 if nfail == 0 else 5)
PYEOF
rc=$?
echo "VAR rc=${rc}（0 全过／5 有 FAIL／9 前置缺失或探针无分辨力）"
exit ${rc}

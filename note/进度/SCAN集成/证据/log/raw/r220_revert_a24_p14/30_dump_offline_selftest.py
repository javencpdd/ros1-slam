import json, os, re, sys, io, contextlib
src=open('ws/src/40_navigation/lite3_scan/verify/p4_replan.py',encoding='utf-8').read()
m=re.search(r'\ndef dump\(out, verdict\):.*?\n    return grade\n', src, re.S)
assert m, 'dump() 取不到'
ns={'json':json,'os':os,'re':re}
exec(m.group(0), ns)
cases=[('P4 重规划结论 = PASS|障碍被看见(94 点)',('PASS',False,True,False)),
       ('P4 重规划结论 = FAIL|扰动后未到达（sim 12.0s，dist=5094.464；物理离箱心最近 0.30m，轨迹 1.10m）',('FAIL',False,False,True)),
       ('P4 重规划结论 = INVALID|扰动未被占据图看见（0.55m 内新增占据点=0＜20）',('INVALID',True,False,False)),
       ('P4 重规划结论 = 乱写的一行（不可指认）',('INVALID',True,False,False))]
ok=0
for verdict,exp in cases:
    buf=io.StringIO()
    with contextlib.redirect_stdout(buf):
        g=ns['dump']({'sample':1}, verdict)
    lines=buf.getvalue().rstrip('\n').split('\n')
    first_is_verdict=lines[0].startswith('P4 重规划结论 = ')
    whole=buf.getvalue()
    try:
        json.loads(whole); parsed='整份可 json.loads（＝分件形状还在，撤销未生效）'
    except Exception:
        parsed='整份不可 json.loads（旧形状已回来，符合裁定）'
    d=json.loads(lines[-1])
    got=(d['grade'],bool(d['invalid']),bool(d['pass']),bool(d['fail']))
    same = (g==exp[0]) and (got==exp)
    print('%s 判据行=%s｜行数=%d｜%s｜字段=%s｜与判据行同向=%s'%(
        'OK  ' if same else 'FAIL', first_is_verdict, len(lines), parsed, got, same))
    ok+= 1 if same else 0
print('\n离线自测 %d/%d 全过（单一真值保留、分件形状已撤、污染档不再存在）'%(ok,len(cases)))
print('源码里 POLLUTION_DIST 出现次数=%d'%src.count('POLLUTION_DIST'))
sys.exit(0 if ok==len(cases) else 1)

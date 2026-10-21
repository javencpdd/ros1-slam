import json, os, re, glob, subprocess
CLAIM = re.compile(r'(已证|未证|被实证推翻|实证推翻|推翻|坐实|确证|不可终止)')
EXEMPT = re.compile(r'(待裁|待主理人|主理人裁|引自|不得引用|过期|原写|原文不回改)')
KV = re.compile(r'[A-Za-z_][A-Za-z0-9_.]{2,}\s*[=：:]\s*-?[0-9A-Za-z_.]|`[a-z_]{3,}`')
names = set()
for j in glob.glob('note/进度/**/证据/**/*.json', recursive=True):
    try: d = json.load(open(j, encoding='utf-8', errors='ignore'))
    except Exception: continue
    st = [d]
    while st:
        o = st.pop()
        if isinstance(o, dict):
            for k, v in o.items():
                if len(k) >= 3: names.add(k.lower())
                st.append(v)
        elif isinstance(o, list):
            st.extend(o[:20])
for cfg in glob.glob('ws/src/**/*.yaml', recursive=True) + glob.glob('ws/src/**/*.xml', recursive=True):
    try: txt = open(cfg, encoding='utf-8', errors='ignore').read()
    except Exception: continue
    for m in re.findall(r'([A-Za-z_][A-Za-z0-9_]{2,})\s*[:=]', txt): names.add(m.lower())
consts = subprocess.run(['grep','-rhoE','[A-Z_]{3,15} *=','ws/src/40_navigation/lite3_scan/verify/','scripts/verify/'],capture_output=True,text=True).stdout
for m in re.findall(r'([A-Z_]{3,15}) *=', consts): names.add(m.lower())
print('键名表规模=%d（全部从 json 叶子键／ws 配置键／取证件常量里抽，**未自编**）' % len(names))
docs = ['note/00_README.md','note/02_问题与坑.md','note/04_配置改动diff.md','note/05_复验作业单.md','note/06_三态通道普查.md','note/07_判据载体代际对照表.md','note/进度/00_总览.md','note/进度/00_裁定请求单_2026-10-09.md'] + sorted(glob.glob('note/进度/SCAN集成/设计/*.md')) + sorted(glob.glob('note/进度/*/进度文档.md')) + sorted(glob.glob('note/计划书v4/*.md'))
tot_a=tot_b=tot_c=0; per={}
samples=[]
for p in docs:
    if not os.path.exists(p): continue
    a=b=c=0
    for ln,line in enumerate(open(p,encoding='utf-8',errors='ignore').read().splitlines(),1):
        if not CLAIM.search(line): continue
        low=line.lower()
        if KV.search(line) or any(n in low for n in names): a+=1
        elif EXEMPT.search(line): c+=1
        else:
            b+=1
            samples.append((p,line.strip()[:130]))
    if a+b+c: per[p]=(a,b,c); tot_a+=a; tot_b+=b; tot_c+=c
print('扫描文件=%d｜含结论强度词的行合计=%d：已引用=%d｜**未引用=%d**｜属"待裁／转述／更正上下文"豁免=%d'%(len(per),tot_a+tot_b+tot_c,tot_a,tot_b,tot_c))
print('--- 未引用最多的 8 个文件（未引用／已引用／豁免）---')
for p,(x,y,z) in sorted(per.items(), key=lambda kv:-kv[1][1])[:8]: print('  %-72s %3d %3d %3d'%(os.path.basename(p),y,x,z))
META = re.compile(r'["「『`][^」』"`]{0,40}(已证|未证|推翻|坐实|确证|不可终止)|不得写|不得使|原写|措辞|标题|写成|用“|不作路线')
met=[x for x in samples if META.search(x[1])]; real=[x for x in samples if not META.search(x[1])]
print('关键发现：46 条“未引用”里 **%d 条是元话语**（引号里提到这些词、或写着“不得写/原写/措辞/标题/用于…”）——不是断言；机器闸会把它们全部拦下来' % len(met))
print('真断言且未引用数：%d ——全部列出' % len(real))
for p,x in real: print('   [%s] %s' % (os.path.basename(p), x[:104]))

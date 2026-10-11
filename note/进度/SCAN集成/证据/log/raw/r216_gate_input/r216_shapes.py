import json, os, re, glob, subprocess, time, sys
CLAIM = re.compile(r'(已证|未证|被实证推翻|实证推翻|推翻|坐实|确证|不可终止)')
EXEMPT = re.compile(r'(待裁|待主理人|主理人裁|引自|不得引用|过期|原写|原文不回改)')
KV = re.compile(r'[A-Za-z_][A-Za-z0-9_.]{2,}\s*[=：:]\s*-?[0-9A-Za-z_.]|`[a-z_]{3,}`')
META = re.compile(r'["\u300c\u300e`][^\u300d\u300f"`]{0,40}(\u5df2\u8bc1|\u672a\u8bc1|\u63a8\u7ffb|\u5750\u5b9e|\u786e\u8bc1|\u4e0d\u53ef\u7ec8\u6b62)|\u4e0d\u5f97\u5199|\u4e0d\u5f97\u4f7f|\u539f\u5199|\u63aa\u8f9e|\u6807\u9898|\u5199\u6210|\u7528\u201c|\u4e0d\u4f5c\u8def\u7ebf|\u4e0d\u5f97\u88ab\u56de\u5199')
names=set()
for j in glob.glob('note/进度/**/证据/**/*.json', recursive=True):
    try: d=json.load(open(j,encoding='utf-8',errors='ignore'))
    except Exception: continue
    st=[d]
    while st:
        o=st.pop()
        if isinstance(o,dict):
            for k,v in o.items():
                if len(k)>=3: names.add(k.lower())
                st.append(v)
        elif isinstance(o,list): st.extend(o[:20])
for cfg in glob.glob('ws/src/**/*.yaml',recursive=True)+glob.glob('ws/src/**/*.xml',recursive=True):
    try: txt=open(cfg,encoding='utf-8',errors='ignore').read()
    except Exception: continue
    for m in re.findall(r'([A-Za-z_][A-Za-z0-9_]{2,})\s*[:=]',txt): names.add(m.lower())
cs=subprocess.run(['grep','-rhoE','[A-Z_]{3,15} *=','ws/src/40_navigation/lite3_scan/verify/','scripts/verify/'],capture_output=True,text=True).stdout
for m in re.findall(r'([A-Z_]{3,15}) *=',cs): names.add(m.lower())
docs=['note/00_README.md','note/02_问题与坑.md','note/04_配置改动diff.md','note/05_复验作业单.md','note/06_三态通道普查.md','note/07_判据载体代际对照表.md','note/进度/00_总览.md','note/进度/00_裁定请求单_2026-10-09.md']+sorted(glob.glob('note/进度/SCAN集成/设计/*.md'))+sorted(glob.glob('note/进度/*/进度文档.md'))+sorted(glob.glob('note/计划书v4/*.md'))
docs=[p for p in docs if os.path.exists(p)]
def has_trace(s):
    low=s.lower()
    return bool(KV.search(s)) or any(n in low for n in names)
def run(skip_title, meta, widen, per_file=None):
    tot=0; det={}
    for p in docs:
        L=open(p,encoding='utf-8',errors='ignore').read().splitlines()
        n=0
        for i,line in enumerate(L):
            if not CLAIM.search(line): continue
            if skip_title and i<8: continue
            if meta and META.search(line): continue
            ctx=line if not widen else line+' '+(L[i-1] if i>0 else '')+' '+(L[i+1] if i+1<len(L) else '')
            if has_trace(ctx): continue
            if EXEMPT.search(line): continue
            n+=1
        det[p]=n; tot+=n
    return tot, det
t0=time.time()
res={}
for label,(a,b,c) in {'S0 行粒度·严（现行判据）':(0,0,0),'S1 ＋跳每文件前 8 行（挡标题）':(1,0,0),
                     'S2 ＋元话语过滤（引号内提及／"不得写"／"措辞"）':(0,1,0),
                     'S3 ＋邻行 ±1 也算痕迹':(0,0,1),
                     'S4 三条一起用（S1+S2+S3）':(1,1,1)}.items():
    n,det=run(a,b,c); res[label]=(n,round(time.time()-t0,2),det)
    print('%-46s 未引用=%3d' % (label,n))
_,_,det0=res['S0 行粒度·严（现行判据）']; _,_,det4=res['S4 三条一起用（S1+S2+S3）']
moved=sorted(((det0[p]-det4[p],p) for p in det0 if det0[p]!=det4[p]), key=lambda x:-abs(x[0]))[:5]
print('\n乙（只拦新增）所需基线：按文件计数已生成；两批之间**变化的文件**（S0 口径演示，S0→S4）：')
for d,p in moved: print('   %-70s %+d' % (os.path.basename(p), -d))
json.dump({'per_file_S0':det0,'per_file_S4':det4,'files':len(docs)}, open('/tmp/r216_baseline_preview.json','w'), ensure_ascii=False)
print('\n合计：S0=%d｜S4=%d ⇒ 三条降噪可把误拦从 %d 降到 %d（降 %d%%）'%(res['S0 行粒度·严（现行判据）'][0],res['S4 三条一起用（S1+S2+S3）'][0],res['S0 行粒度·严（现行判据）'][0],res['S4 三条一起用（S1+S2+S3）'][0],round(100*(1-res['S4 三条一起用（S1+S2+S3）'][0]/res['S0 行粒度·严（现行判据）'][0]))))

import io, os
H=os.path.expanduser('~/.zcode/skill/')
PAIRS=[('note/计划书v5/21_ros1-slam隔离部署计划书-v5.md','note/计划书v5/21_ros1-slam隔离部署计划书-v5.1.md'),
       ('（**1000 行**；v5＝v4 的 984 行逐字收编＋v4.7 补丁稿 Z1–Z14，R223 换真身生效）','（**1097 行**＝v5 的 1000 行逐字保留＋§0.6–§0.10；R224 依主理人评审优化表 R01–R44 修订收编、换真身生效。**先读 §0.6 生效序**：§0.6–§0.10 ＞ §16 各格 ＞ 一期正文；v5 与 v4 系全部转历史件）'),
       ('# **v5 正文＝唯一真身**（R223 收编生效）','# **v5.1 正文＝唯一真身**（R224 修订收编生效）；计划书v5/ 内 v5 与评审表＝历史／依据件'),
       ('# **计划书唯一真身＝21_ros1-slam隔离部署计划书-v5.md**（R223 起）','# **计划书唯一真身＝21_ros1-slam隔离部署计划书-v5.1.md**（R224 起；先读 §0.6）'),
       ('《ros1-slam 隔离部署计划书 **v5**》正文','《ros1-slam 隔离部署计划书 **v5.1**》正文')]
for f in ['software-company/SKILL.md','software-company/agents/主理人-佳文.md','software-company/agents/计划与风险管家-小王.md','senior-developer/SKILL.md']:
    p=H+f; s=open(p,encoding='utf-8').read(); b=len(s.splitlines()); n=0
    for o,nw in PAIRS:
        c=s.count(o)
        if c: s=s.replace(o,nw); n+=c
    io.open(p,'w',encoding='utf-8').write(s)
    print('%-46s 替换 %d 处；行数 %d -> %d；含 v5.1=%s'%(f.split('/')[-1],n,b,len(s.splitlines()),'v5.1' in s))

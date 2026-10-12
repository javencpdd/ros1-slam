# -*- coding: utf-8 -*-
import io, os, subprocess
SRC='note/计划书v5/21_ros1-slam隔离部署计划书-v5.md'
DST='note/计划书v5/21_ros1-slam隔离部署计划书-v5.1.md'
assert not os.path.exists(DST), 'v5.1 已存在'
T=subprocess.run(['date','+%F %H:%M'],capture_output=True,text=True).stdout.strip()
s=open(SRC,encoding='utf-8').read()
front=open('/tmp/v51_front.md',encoding='utf-8').read()

# 1) 标题与来历
old_title='# 21 · 103 主机 `ros1-slam` 用户：模块化架构隔离部署计划书（**v5**）'
assert s.count(old_title)==1
s=s.replace(old_title, '# 21 · 103 主机 `ros1-slam` 用户：模块化架构隔离部署计划书（**v5.1**）',1)
prov=('> **v5.1 ＝ v5 按主理人评审优化表（R01–R44）的修订收编版**（主理人 %s 明文"根据该优化表，将 v5 计划书修改为 v5.1 并落定"）。'
      '**v5 与 v4 系全部转历史件、原文一字不动**；**现行真身＝本件**。'
      '**先读 §0.6**（生效序：§0.6–§0.10 ＞ §16 各格 ＞ 一期正文 §1–§15）；本件对一期正文**只逐字改写了 5 处**（详见 §0.6 部分落地声明），'
      '其余以 §0.7 逐条处置表＋§0.8 有效态速查**替代**或**登记待施工**——凡标"待施工"者盘上未落地，不得读成已修。\n') % T
anchor='> **状态三值（v5 内每条裁定必带其一，缺标记即读作未落）**'
i=s.index(anchor); j=s.index('\n', i)+1
# 把"v5 内"改写为"文内"以免读成 v5 口径
s=s[:j]+prov+s[j:]
s=s.replace(anchor, anchor.replace('v5 内','文内'),1)
k=s.index('## v3 变更日志')
s=s[:k]+front+'\n'+s[k:]

# 2) 五处逐字改写
REP=[]
# (a) §1 A11：S9 伪前置
REP.append(('| **A11** | **v4 新增**：M4 服务层 = **mbf_costmap_nav 0.4.0 联邦式**；全局规划器加载需 S9（`ros-noetic-nav-core-adapter` 0.3.0，Q9）——**不批则整体回退 move_base 单体**（§8 回退决策表） | P3a.0：teb 双注册可被 mbf 原生加载；nav_core_adapter 未装、apt 官方源可装 |',
 '| **A11**（**v5.1 修订，治 R02**）：M4 服务层 = **mbf_costmap_nav 0.4.0 联邦式**；全局规划器加载**不需要 S9**——v4.1 X3 已裁「不装不卸、Q9 关闭」，实测复核＝`/opt/ros/noetic/include/mbf_costmap_nav/costmap_navigation_server.h` 自带 planner／controller／recovery 三对加载器（mbf 与 nav_core 各一）。**本机 0.3.0 已装 ⇒ 只登记存在、不作前置**；回退**仅由真实联调阻塞触发**，不再有"未批 S9→回退"这条伪前置（§8 回退决策表同此读法） | 原句（需 S9、不批则回退）**〔历史，由本行替代〕**；依据＝v4.1 X3＋本会话只读抽查 E4 |'))
# (b) §1 关键事实行：lidar_type 解释＋interval 现值＋话题
REP.append(('| **faster-lio 关键事实** | `lidar_type: 4`；`lid_topic: /rslidar_points_filtered`（**无生产者**）；存盘路径硬编码 `/home/ysc/lite_cog/system/map/lite3.pcd`（`laser_mapping.cc:751`；**坑2：实测两处，751 与 862**）；参数经**全局 NodeHandle** 读取（`run_mapping_online.cc:25`） | V2/V4/V5 + §C1 |',
 '| **faster-lio 关键事实**（**v5.1 修订，治 R04／R21／R07**）：`lidar_type: 4` 的枚举归属＝**C16 处理分支**（**不是 Velodyne 分支**——旧解释错在此，值本身对）；话题面现役＝**Livox Mid-360**（`imu_topic: /livox/imu`，`faster_lio_outdoor.yaml:3` 实测；`/rslidar_points*` 一族属**遗留对照**）；存盘＝`pcd_save.interval: -1`（第 29 行实测；**`10` 已作废**：每 10 帧写盘后清空 ⇒ 地图截断）；存盘路径硬编码两处（**坑2：751 与 862**）；参数经**全局 NodeHandle** 读取 ⇒ 同命名空间只能跑一套 | 原句〔历史，由本行替代〕；现值＝本会话只读抽查（§0.10），**文件值确认不等于节点已加载** |'))
# (c) §8 C1 launch：可执行名
REP.append(('<node pkg="faster_lio" type="mapping_online" name="faster_lio_online"/>',
 '<node pkg="faster_lio" type="run_mapping_online" name="faster_lio_online"/>'))
REP.append(('**建图执行流程**：tmux 版脚本',
 '**（v5.1 治 R03）**照抄旧句 `type="mapping_online"` 会**启动失败**——本会话实测 `ws/src/10_slam/faster_lio/app/CMakeLists.txt:1` ＝ `add_executable(run_mapping_online run_mapping_online.cc)`；任何建图入口执行前先跑**存在性检查**（`rospack find faster_lio`＋`ls $(catkin_find ...)`/`rosnode` 启动返回码），失败即停手不猜。**建图执行流程**：tmux 版脚本'))
# (d) §17 状态行
REP.append(('- ⏸ 未执行：P3a 及以后全部阶段（本文不执行；S9 待 Q9 用户批准，未装）；⏸ 待确认：Q1/Q3/Q4/Q5/Q6/Q7/**Q8**/**Q9**/**Q11**（§15）',
 '- **v5.1 修订（治 R14／R37）**：把"设计冻结"“工程完成"“正式验收""增量待施工"四件事**分开记录**——'
 '① **历史设计冻结**：§5–§7（P0–P2）正文只作从零重建脚本，续施工不得整段重跑；'
 '② **已施工完成（在册记录）**：P3a／P4／P5 与 P3b／G3 及后续 M2／M5／SCAN 系列有进度与证据留位，引 `note/进度/00_总览.md`——**引用在册记录不等于本会话重跑验收**；'
 '③ **正式验收**：G0–G7 PASS、**G8／G9 未到**（P4 达成仍是真机前置的关键未达项）；'
 '④ **增量待施工**：§0.7 标"待施工"的 24 项 T-R 系列＋v5 遗留的 Z3／Z4／Z5／Z6／Z12／Z14。'
 'S9 **不作前置**（R02）；Q 表现值＝Q9／Q12 关闭（翻案在册）、Q13 俯仰精测**待现场读数**、其余待确认项照 §15 逐条追。'))
for old,new in REP:
    assert s.count(old)>=1, old[:60]
    s=s.replace(old,new, s.count(old) if old.startswith('**建图执行流程') else 1)
# (e) §16.4 两句次序（治 R36：新句在旧句之前、且句中指代"上句"）
lines=s.splitlines(True)
i_old=[n for n,l in enumerate(lines) if l.startswith('- **A25 ＝待裁**')]
i_new=[n for n,l in enumerate(lines) if l.startswith('- **A25 已裁定（v5，Z4）＝已裁待施工**')]
assert len(i_old)==1 and len(i_new)==1 and i_new[0]<i_old[0], (i_old,i_new)
lines[i_new[0]],lines[i_old[0]]=lines[i_old[0]],lines[i_new[0]]
s=''.join(lines)
io.open(DST,'w',encoding='utf-8').write(s)
n=len(s.splitlines())
print('v5.1 建成：%s ＝ %d 行（v5 为 %d 行，+%d）'%(DST,n,len(open(SRC,encoding='utf-8').read().splitlines()),n-1000))
print('@@ 残留=%d；mapping_online 独立旧形残留=%d；待施工字样=%d'%(s.count('@@'), sum(1 for l in s.splitlines() if 'type="mapping_online"' in l), s.count('待施工')))

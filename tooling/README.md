# tooling｜项目自持清单（闭环资产与外部依赖）

> 来历＝主理人 2026-10-12 明文「**尽量所有所需的文件，例如 skill、mcp 工具等等，都保存放在 ros1-slam 的文件目录下，让 ros1-slam 的项目整体闭环，这一点加入计划书中**」。条款正文已入 **v5.3 §0.13**；本件是**清单与恢复法**（照抄即用）。
> 时刻只认 `date`：本清单实测于 2026-10-12 23:3x（R230 批）。数字为现跑，非估算。

## 1. 已在仓内的（自持，搬迁时一起走）

| 件 | 体积（现跑 `du -sh`） | 说明 |
|---|---|---|
| `ws/` | **1.2G** | 源码面；**`grep -rl /home/ysc ws/src`（launch／yaml／py）计数＝0** ⇒ 被测件面不指向生产路径 |
| `opt/` | **1.3G** | 第三方与原件区（`opt/ref/` 保原样存来源，属在册豁免面） |
| `note/` | **71M** | 台账志文面（含证据目录 `note/进度/SCAN集成/证据/log/raw/`） |
| `scripts/` | **332K** | 门禁件与工具（`scripts/verify/` 在册 15 件＋面清单） |
| `data/` `logs/` `docs/` | 122M／11M／8.5M | 底图与输出面 |
| `.git/` | 252M | 版本面（**永不复制 build／devel／.git 到别处**的旧红线只约束"往 ws 里塞"，搬迁整仓时 `.git` 随仓走） |
| **`tooling/skill/`** | **6.4M／133 件** | **本批新增**：技能包逐字镜像＋`_sha256_manifest.txt`（133 件全 OK）＋`RESTORE.sh` |

## 2. 不在仓内、且**搬不进来**的（外部依赖，只能登记＋给恢复法）

| 依赖 | 真实位置（现跑） | 能否搬进 `ros1-slam` | 恢复办法 |
|---|---|---|---|
| **技能包真身** | `/home/ysc/.zcode/skill/{software-company,senior-developer,note}`；另有 `/home/ysc/.qoder/skills/*` **是指向它的符号链接**（`ls -ld` 实证） | **工具读取路径固定 ⇒ 真身不能搬**；本批以**镜像＋恢复脚本**闭环 | `bash /home/ros1-slam/tooling/skill/RESTORE.sh`（校验→备份→恢复→目录 775／文件 664→重建符号链接） |
| **MCP 配置** | 本机**不存在**可搬的文件：`find / -maxdepth 4 -name 'mcp*.json'` 零命中；`/home/ysc/.qoder/` 下只有 `remote/protocol-v1` 与 `skills/`（符号链接） | **不能**——MCP servers（`browser-use`／`builtin`）由**会话宿主的 app runtime** 提供，配置不落在这台施工机上；`/home/jack/.qoder` 在本机不存在 | 只能按 §0.13 的"依赖声明"方式登记：server 名＋用途＋由谁提供；换环境时在主理人侧重建 |
| **系统 ROS** | `/opt/ros/noetic`（`ls /opt`＝containerd／nvidia／ota_package／**ros**／sublime_text；志文里 `/opt/ros` 出现 9164 次） | **不能**（apt 安装面、需 root） | 在册三步环境：`source /opt/ros/noetic/setup.bash` ＋ `source ws/devel/setup.bash`（`set +u` 子 shell） |
| **systemd 单元** | `/etc/systemd/system/{lite3-monitor,lite3-ros-bridge,transfer_ros2}.service` 等（root 属主） | **不能**（写 `/etc` 需 sudo，施工侧无 sudo） | v5.3 §0.12.1 已把每个 unit 的**功能／影响／恢复命令**逐件写死；单元文件正文可由主理人导出后入 `tooling/` |
| **生产侧** | `/home/ysc/lite_cog/`、`/home/ysc/lite_cog_ros2/`（志文里 `/home/ysc` 出现 3898 次，**绝大多数是来源记录**） | **禁止搬、禁止改、禁止删**（在册红线：永不删除 `/home/ysc/lite_cog/`；`/home/ros1-slam` 内禁 `/home/ysc` 字符串，**note 来源记录与 `opt/ref` 除外**） | 不恢复——它不属本项目资产 |
| **会话宿主家目录** | 本 agent 的 session 文件在 `/home/jack/.qoder/...`（另一台机） | **不能** | 换机时按 §0.13 的"搬迁影响清单"另出一批 |

## 3. 规矩（本清单生效期间）

1. **新工具件默认落仓内**：`tooling/<名>/`，并同批出 `_sha256_manifest.txt`；需要工具从固定路径读取的，**仓内放镜像＋仓外放符号链接**，链接由 `RESTORE.sh` 负责重建。
2. **权限照 §0.12 事故后的规矩**：目录 **775**、文件 **664**（脚本 755）；`chmod` **一律绝对路径＋`find -exec`，禁止在旧 cwd 上用通配**。
3. **不可搬项**每条都要有"恢复办法"一栏，**没有恢复办法的项＝登记为阻塞项上报**，不得留空当已完成。
4. **技能包若改动**：改前备份**带子目录或改名**（不得同名平铺 `cp`），改后**同批刷新 `tooling/skill/` 镜像与清单**——镜像与真身不一致＝本条第 4 项违规。

## 4. R245 新增：systemd 生产单元**只登不抄**（正文含在册禁字符串，仓内不给副本）

- 六份单元（monitor／ros-bridge／transfer_ros2／whoopsie／apport-autoreport.path／apport-forward.socket）的**来历表**落在 `note/03_外部文件来源清单.md`（该面是红线唯一豁免列之一）；每行给 绝对路径／属主与模／字节数／mtime／**内容 sha256**／正文是否含禁串。
- **为什么不抄进 `tooling/`**＝其中三份生产 unit 的正文含本仓在册禁字符串（红线只豁免 note 来源记录与 `opt/ref`，`tooling/` 不在列）⇒ 逐字入仓即违规。同一性由 sha 可验：`sha256sum /etc/systemd/system/<unit>` 对表即可（other 位可读，`-rw-r--r-- root root`，**不需要 sudo**）。
- **代价写死**：仓内无正文 ⇒ 单目录重建时**无法离线复原这些 unit**；要入册正文须主理人对 **A36** 择案（三选：只维持 sha 登记／给 `tooling/` 开豁免面并配"未命中即报"闸／脱敏副本＋还原说明）。
- **顺带实测到的服务面依赖（2026-10-13 03:0x，非推断）**：`transfer_ros2.service` 的 ExecStart 直指生产账户树内的 `…/lite_cog_ros2/system/scripts/transfer/start_transfer.sh`；monitor 与 bridge 两份设 `Environment=HOME=` 指向该账户、`WorkingDirectory` 与 `ExecStart` 指向 `/home/test/monitor`（**34M、ysc 属主**，内含 backend/frontend/.venv，`/home/test/monitor/README.md` 首行＝"# Lite3 Robot Monitor"）。⇒ 这一层比 R240 报的 numpy 执行面**更硬**，已作为 A36 进请求单。

- **R348 README 同步（追加句）**＝本目录在册面多一枚此前未登记＝`_forbid_str_exempt.tsv`（**R331 建**，来源＝真人 2026-10-15 勾 **A36＝乙**）＝`tooling/` 的**禁字符串显式豁免面**：现跑**总 26 行／数据行 16**（式＝`wc -l`＋`grep -cvE '^ *(#|$)'`，时刻 2026-10-17 03:0x，单位＝行），列序＝`仓内相对路径<TAB>命中处数<TAB>档位<TAB>出处（锚文本，不给裸行号）`；门用面写死三条＝①扫描范围只有 `tooling/` 全树（含 `skill/` 镜像与 `systemd/`）②表内不含被禁字面（两段拼接）③未命中即报（防条目静默失效）。★它与本 README 上一段"整仓搬迁时 `.git` 随仓走"的关系＝`note/进度/SCAN集成/设计/au_x86仿真迁出交接任务单_设计稿.md` §4.2 第 2 条写明**迁出子集包不属该条对象**（不拷 `.git`），两处文字并存、★本句只做指向、★不判谁过期。

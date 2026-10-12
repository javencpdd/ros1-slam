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

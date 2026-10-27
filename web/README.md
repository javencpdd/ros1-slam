# `/web` · 项目官网（静态单页）

零依赖、零构建、可离线打开的项目介绍页：定位、能力地图、架构、里程碑、当前判据状态、
仿真与观察入口、新算法接入方式、文档索引。

> 页面数字是**快照**，权威真身仍是 `note/进度/00_总览.md` 与
> `note/计划书v8/calibration/calibration.py` 的现跑结果。

## 启动方式（推荐：仓库自带的 `web/serve.py`）

页面里的「文档索引」用相对路径指向 `../note/`、`../docker/`。
**从仓库根目录起服务**，这些链接才可达：

```bash
cd /home/jack/ysc-code/ros1-slam
python3 web/serve.py
# 浏览器打开 http://127.0.0.1:8000/web/
```

`web/serve.py` 只做两件事：把服务根指到仓库根、给 `.md/.html/.json` 等文本补
`charset=utf-8`。后者是必要的——用 Python 自带 `http.server` 打开中文 Markdown 会显示成乱码。

改端口 / 允许局域网访问：

```bash
python3 web/serve.py --port 8080
python3 web/serve.py --port 8000 --bind 0.0.0.0
# 其他机器访问 http://<本机IP>:8000/web/
```

## 备选：Python 自带 `http.server`

```bash
cd /home/jack/ysc-code/ros1-slam
python3 -m http.server 8000
# 浏览器打开 http://127.0.0.1:8000/web/
```

站点本身正常；只是点开中文 `.md` 文档时可能乱码（缺 charset），
需要看文档时请用 `web/serve.py`。

## 备选：只服务 `web/` 目录

不需要文档链接时可用：

```bash
cd /home/jack/ysc-code/ros1-slam
python3 -m http.server -d web 8000
# 浏览器打开 http://127.0.0.1:8000/
```

此时页面里的 `../note/...` 链接会 404，属预期。

## 也可以直接双击

`index.html` 不依赖任何外部资源（无 CDN、无字体下载、无构建产物），
文件管理器里双击即可打开；同样，跨目录文档链接在 `file://` 下不可用。

## 更新约定

改动页面时同步三处：

1. **状态数字**：先跑校准件现读，再抄进页面，禁止凭记忆写。
   ```bash
   cd note/计划书v8/calibration
   python3 -B -c "import calibration,json;r=calibration.validate_frozen();print(json.dumps({k:r[k] for k in ('status','pending','digest','freeze_allowed')},ensure_ascii=False,indent=1))"
   ```
2. **页脚快照日期**与 digest 前缀。
3. 若里程碑或任务队列变化，同步「里程碑」「现状」两节，并在提交信息里写明依据的 R 号。

## 边界

- 本页**只读**：不连接 ROS、不发布目标、不控制仿真。
- 不复制台账正文，只做摘要与指针；冲突时以 `note/` 为准。
- 不放构建产物、不引入外部脚本与追踪代码。

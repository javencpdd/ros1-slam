# 04 · 配置改动 diff（登记制：旧值/新值/理由/日期；沙箱内一切偏离"原样复制"的行为都在此）
- faster_lio/src/laser_mapping.cc:751 与 :862 存盘路径 /home/ysc/lite_cog/system/map/lite3.pcd → /home/ros1-slam/data/maps/lite3.pcd（V2 隔离改写，唯一许可的第三方源码改动；sed 全局替换一次覆盖两处）2026-10-04
- 复制范围裁决（非源码改动）：copy_pkg tar 排除清单由 .git/build/devel 增补 .VSCodeCounter/.vscode（工具产物含 /home/ysc 且扩展名在 §7.4 白名单外，不排除则复扫≠0、自检[8]必挂）2026-10-04
- 复制范围裁决扩展：faster_lio 副本内运行产物目录（ /home/ros1-slam/ws/src/10_slam/faster_lio/log）含 /home/ysc 且不在 §7.4 白名单，移除副本内产物（对照组源未动；运行产物非源码）2026-10-04
- 复制范围裁决扩展②：pipeline_tracking/scripts/__pycache__/ 4 个 .pyc（CPython 3.8 字节码缓存，运行产物非源码）内嵌 /home/ysc，移除副本内产物；tar 排除清单增补 __pycache__（今后重复制同样排除）2026-10-04
- P2.4 ⑥补条：原文 find -mindepth 1 不含起始目录，/home/ros1-slam/opt/ref 根目录经③后残留 755，与期望抽查"opt/ref=555"不符；补 sudo chmod 555 /home/ros1-slam/opt/ref 达标（在 ACL ⑦ 之后执行，mask=r-x 不影响 u:ysc:rX）2026-10-04

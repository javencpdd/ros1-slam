# R348-03｜同件两版并排证（动作㉔）＝README 追加留底 `02_` 的改版前后

- 改版原因＝★首版把 `scripts/mapping/` 的权限位现读值 `674` 判成「异常、待一句」；当批现跑坐实＝该值由**目录默认 ACL** `default:user:ysc:rwx`＋`default:mask::rwx` 决定，`stat -c %a` 的中间位读作 **mask**、非组位，有效权限＝`user::rw-`／`other::r--`⇒ ★属共用工作区权限模型的显示形、★不是历批 `chmod` 漏覆盖，判语作废。
- 取证式与读数（2026-10-17 03:0x 现跑，单位＝枚）＝`getfacl -p scripts/mapping` 见默认 ACL 两行；全仓交叉普查 `git ls-files -z | xargs -0 stat -c '%U %a' | sort | uniq -c`＝`ros1-slam 674` **5045** ‖ `ysc 664` **3450** ‖ `ysc 674` **875** ‖ `ros1-slam 775` **126** ‖ `ysc 775` **73** ‖ `ysc 755` **19**。
- ★改版时点＝**尚未应用到任何目标 README 之前**、★且未涉任何在册台账 ⇒ 属留底件内部改版、★不是台账原文回改。

## 一、首版（★未应用、判语已作废）

```
APPEND|scripts/mapping/README.md
- **R348 README 同步（追加句、旧行不回改）**＝本目录另有**两枚脚本此前未登记在本 README**：`pcd_polish.py`＝**133 行／sha 前 8=`037bf5f0`**／mtime 2026-10-07 02:26；`pcd_slice.py`＝**58 行／sha 前 8=`536667fe`**／mtime 同刻（式＝`wc -l <scripts/mapping/<件>>`＋`sha256sum`，时刻 2026-10-17 02:5x，单位＝行）。二者用途＝`pcd_polish.py` 对点云做抛光、`pcd_slice.py` 按 z 切片（★只登记"在位"，不声称其当代读数；★本 README 上方走位要领一节未动）。★同批记一枚异常待一句＝两件现跑**权限位＝`674`**（在册规矩＝脚本 `755`／正文 `664`；`674`＝组可写＋其他只读，属历批 `chmod` 未覆盖面）⇒ ★施工侧不擅改权限位，请主理人给一句"归一为 664/755"或"保留并登记"。
```

## 二、定版（本批实际应用面）

```
APPEND|scripts/mapping/README.md
- **R348 README 同步（追加句、旧行不回改）**＝本目录另有**两枚脚本此前未登记在本 README**：`pcd_polish.py`＝**133 行／sha 前 8=`037bf5f0`**／mtime 2026-10-07 02:26；`pcd_slice.py`＝**58 行／sha 前 8=`536667fe`**／mtime 同刻（式＝`wc -l <scripts/mapping/<件>>`＋`sha256sum`，时刻 2026-10-17 03:0x，单位＝行）。用途＝`pcd_polish.py` 对点云做抛光、`pcd_slice.py` 按 z 切片（★只登记"在位"，不声称其当代读数；★本 README 上方走位要领一节一字未动）。
- **★同批记一枚权限位读法（防后人误判为异常）**＝本目录 16 枚在册件的 `stat -c %a` 现跑多报 **`674`**（含本 README 自身），★这不是历批 `chmod` 漏覆盖：`getfacl -p scripts/mapping` 现跑＝目录带**默认 ACL** `default:user:ysc:rwx`＋`default:mask::rwx`，文件实位＝`user::rw-`／`other::r--`，★中间那位读作 **ACL mask**（具名用户 `ysc` 拿到 `rwx` 把 mask 顶到 `rwx`）⇒ `674` 属**共用工作区权限模型的显示形**、有效权限与 `664` 同义（普查式＝`git ls-files -z | xargs -0 stat -c '%U %a' | sort | uniq -c`，2026-10-17 03:0x 现跑＝`ros1-slam 674`＝**5045** 枚 ‖ `ysc 664`＝**3450** 枚 ‖ `ysc 674`＝**875** 枚，单位＝枚）⇒ ★施工侧不对任何单件 `chmod`（单点改会造成本目录面内不一致）；★在册『脚本 `755`／正文 `664`』那句与盘面主导显示值如何统一表述＝**属全仓权限位口径、不在本 README 判定**，已另起句登请求单。
```

- 两版段数差＝首版 7 段（6 `APPEND|`＋1 `CREATE|`）→ 定版 **6 段**（`APPEND|` 6 枚）；★`CREATE|export/README.md` 那段移出本留底、改随包体面在 `05_` 首包构建留底内记（原因＝该文件属包门面、内容须与包内实盘同批出，★不宜先写空壳）。
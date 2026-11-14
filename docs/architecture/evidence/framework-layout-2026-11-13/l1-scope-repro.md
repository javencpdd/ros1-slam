# 范围与复核入口

最新更新时间：2026-11-13T22:46:53.148851+08:00。

基线HEAD107cfbd3，审核在位未提交目录迁移。实际argv、stdout和rc分别见cli-results.json、imports-results.json、shim-matrix-results.json与纠正后的shim-matrix-results-corrected.json。入口导入脚本为imports_probe.py、shim_probe.py、smoke_help_probe.py，仅本tmp证据中使用。

path-mode-links.json列真实manifest路径/执行位和9个替身标识；syntax-results.json列66个3.8语法结果；reviewed-source-sha256.json为检查后快照（并非运行环境全源冻结），用于主控最终差异核验。

不执行真实smoke或ROS main，不修改历史note/11-12证据，不重跑已完成的两层四窗。

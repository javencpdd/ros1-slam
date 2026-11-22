"""测试包的路径垫片：让同目录 test_*.py 的平铺互相 import 在 discover 下也成立。

背景：本目录若干测试用 `import test_xxx as fixture` 这种同目录平铺写法，
只有当 cwd 就是本目录时才可解析；从包根跑 `python3 -m unittest discover`
时模块被装成 `tests.xxx`，平铺名不在 sys.path ⇒ ImportError（2026-11-23 之前
表现为 representation 5 项、local_planner 2 项 loader 级错误）。

做法：把本目录追加到 sys.path（append 而非 insert，避免遮蔽同名模块）。
两种跑法（包根 discover / 目录内直跑）由此都成立，测试代码本身一字未改。
"""
import os as _os
import sys as _sys

_HERE = _os.path.dirname(_os.path.abspath(__file__))
if _HERE not in _sys.path:
    _sys.path.append(_HERE)

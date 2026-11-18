"""PCD 读取工具（2026-11-18 从三个 hdl_* smoke 抽出）。

这三处 `read_pcd` 原先各自定义了一份、**逐字完全相同**（已用 diff 核实），
是同一段"解析二进制 PCD 头 + numpy frombuffer"的复制。抽成公共模块后，
将来改解析逻辑（比如支持 ascii / 压缩 PCD）只需要改一处。

为什么不做成更通用的 pcd 库：仓内只有这三处用到，且都只要 xyz 数值；
引入依赖或写全功能解析器属于"为用不上的能力付成本"。

注意：本模块相对同目录的 smoke 脚本被 import，靠 `sys.path[0]`（脚本目录）
生效；不依赖任何 ROS 环境，可单独跑单测。
"""
import numpy as np


def read_pcd(path):
    """读二进制 PCD，返回 numpy 结构化数组。

    只支持 `DATA binary`（本项目产出的 PCD 都是这种）。头部解析按 PCD v0.7：
    FIELDS / SIZE / POINTS 三行必需，缺任一行会 IndexError
    （原实现即如此，此处保持一致，不改语义）。
    """
    with open(path, 'rb') as stream:
        header = []
        while True:
            line = stream.readline()
            if not line:
                raise ValueError('PCD header incomplete')
            header.append(line)
            if line.startswith(b'DATA'):
                break
        fields = [line.split()[1:] for line in header if line.startswith(b'FIELDS')][0]
        sizes = [int(x) for x in [line.split() for line in header
                                  if line.startswith(b'SIZE')][0][1:]]
        n = int([line.split()[1] for line in header if line.startswith(b'POINTS')][0])
        data = stream.read()
    dtype = np.dtype([(fields[i].decode(), 'f%d' % sizes[i]) for i in range(len(fields))])
    return np.frombuffer(data, dtype=dtype, count=n)

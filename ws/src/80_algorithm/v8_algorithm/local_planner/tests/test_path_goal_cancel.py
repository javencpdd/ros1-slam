"""取消后按代次拒绝旧路径重放 —— 判据测试（含反控）。

被测函数是 `path_goal_node.accepts_path_stamp(cancelled_at, path_stamp)`：
取消之后的路径是否允许重新驱动 local。

三连结构：
  主测     —— 取消后旧 stamp 必须被拒、新 stamp 必须被放行；
  反控     —— 把 cancelled_at 置 None（等于没取消）后，同一条旧路径必须被放行，
              否则说明测的其实是"stamp 是否存在"，而不是"取消后是否重放"；
  代次序列 —— 按 path→cancel→旧 path→新 path 的顺序走一遍，验证 generation 只在新路径上递增。
"""
import sys
from pathlib import Path
from unittest.mock import MagicMock

# path_goal_node 顶部 import 了 rospy 与三类消息；这里只测纯函数，用 MagicMock 顶掉。
for _name in ('rospy', 'geometry_msgs', 'geometry_msgs.msg', 'nav_msgs',
              'nav_msgs.msg', 'std_msgs', 'std_msgs.msg'):
    if _name not in sys.modules:
        sys.modules[_name] = MagicMock()

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))
import path_goal_node as pg  # noqa: E402

import unittest  # noqa: E402


def tearDownModule():
    """清掉本模块注入的 MagicMock，避免影响同进程里后续测试的 rospy 导入。"""
    for _name in ('rospy', 'geometry_msgs', 'geometry_msgs.msg', 'nav_msgs',
                  'nav_msgs.msg', 'std_msgs', 'std_msgs.msg'):
        if isinstance(sys.modules.get(_name), MagicMock):
            del sys.modules[_name]


class AcceptsPathStampTests(unittest.TestCase):

    # ---------- 主测 ----------
    def test_accepts_when_never_cancelled(self):
        self.assertEqual(pg.accepts_path_stamp(None, 100.0), (True, 'no_cancel'))
        self.assertEqual(pg.accepts_path_stamp(None, 0.0), (True, 'no_cancel'))

    def test_rejects_stale_path_after_cancel(self):
        self.assertEqual(pg.accepts_path_stamp(150.0, 120.0),
                         (False, 'stale_after_cancel'))
        # 边界：与取消时刻完全相同也算旧（<=）
        self.assertEqual(pg.accepts_path_stamp(150.0, 150.0),
                         (False, 'stale_after_cancel'))

    def test_accepts_newer_path_after_cancel(self):
        self.assertEqual(pg.accepts_path_stamp(150.0, 150.001),
                         (True, 'newer_than_cancel'))

    def test_fail_closed_without_stamp(self):
        """拿不到 stamp 就无法区分新旧，必须拒绝而不是放行。"""
        self.assertEqual(pg.accepts_path_stamp(150.0, 0.0),
                         (False, 'no_stamp_fail_closed'))
        self.assertEqual(pg.accepts_path_stamp(150.0, None),
                         (False, 'no_stamp_fail_closed'))

    # ---------- 反控 ----------
    def test_control_no_cancel_accepts_the_same_stale_path(self):
        """同一条 120 秒的路径：取消后拒、取消前收 —— 证明判定看的是取消态。"""
        rejected, _ = pg.accepts_path_stamp(150.0, 120.0)
        accepted_control, why = pg.accepts_path_stamp(None, 120.0)
        self.assertFalse(rejected)
        self.assertTrue(accepted_control)
        self.assertEqual(why, 'no_cancel')

    # ---------- 代次序列 ----------
    def test_generation_sequence(self):
        """模拟 path(100) -> cancel(150) -> path(120) -> path(160)。

        generation 只能在新路径上递增；旧路径要被计入 rejected_stale。
        """
        cancelled_at = None
        generation = 0
        rejected = 0
        for stamp in (100.0, 120.0, 160.0):
            accepted, why = pg.accepts_path_stamp(cancelled_at, stamp)
            if stamp == 120.0:
                # 第二次路径之前发生了取消
                cancelled_at = 150.0
                accepted, why = pg.accepts_path_stamp(cancelled_at, stamp)
            if accepted:
                generation += 1
                if cancelled_at is not None:
                    cancelled_at = None
            else:
                rejected += 1
        self.assertEqual(generation, 2, '只有 100 与 160 两条应被接受')
        self.assertEqual(rejected, 1, '120 这条旧路径必须被拒一次')


if __name__ == '__main__':
    unittest.main()

import re, subprocess, datetime, sys

ROOT = "/home/ros1-slam"
F = ROOT + "/note/进度/00_总览.md"
INC = ROOT + "/note/进度/SCAN集成/证据/log/raw/r313_a54_a55_rulings_arrival/14_overview_increment.txt"
MARK = "本 boot 现况增量一格＝**"

lines = open(F, encoding="utf-8").read().split("\n")
b_count = len(lines)
old2 = lines[1]
assert old2.startswith("> 日期："), "第二行不是抬头串，停手"
i = old2.index(MARK)
head = old2[:i]
tail = old2[i + len(MARK):]
m = re.search(r"\*\*更新时间：([^（]+)（(R\d+) 批同步", head)
assert m, "抬头里找不到 更新时间 串，停手"
old_ts, old_r = m.group(1), m.group(2)
new_ts = datetime.datetime.now().strftime("%Y-%m-%d %H:%M")
new2 = (head[:m.start()] + "**更新时间：" + new_ts +
        "（R313 批同步：当批滚、C4 由件报数；日期与时分整串从 date 取）。" + MARK +
        open(INC, encoding="utf-8").read().strip() +
        "｜原 " + old_ts + "（" + old_r + " 批同步）＝" + tail)
assert "\n" not in new2, "新串含换行，停手"
lines[1] = new2
open(F, "w", encoding="utf-8").write("\n".join(lines))
a_count = len(open(F, encoding="utf-8").read().split("\n"))
print("旧时次=%s 旧批=%s 新时次=%s" % (old_ts, old_r, new_ts))
print("换行前=%d 换行后=%d 新第二行字符数=%d" % (b_count, a_count, len(new2)))
print("ASSERT 行数不变:", "OK" if b_count == a_count else "FAIL")

import sys

NEW = "★档头串＝单行原位替换（行数不变）＋条目区尾部追加 **1 行** ⇒ 本件 577→**578**"
F1 = "/home/ros1-slam/note/进度/00_总览.md"
F2 = "/home/ros1-slam/note/04_配置改动diff.md"
F3 = "/home/ros1-slam/note/进度/SCAN集成/证据/log/raw/r313_a54_a55_rulings_arrival/18_overview_entry.txt"

# 1) 总览：两处 R313 自述里的"本件行数不变"
s = open(F1, encoding="utf-8").read()
old_a = "★本件行数不变（档头单行原位）"
n = s.count(old_a)
print("总览 命中『%s』= %d" % (old_a, n))
if n not in (1, 2):
    sys.exit("停手：命中数不是 1 或 2")
s = s.replace(old_a, NEW)
open(F1, "w", encoding="utf-8").write(s)

# 2) 04：一处"总览行数不变＝档头单行原位"
s = open(F2, encoding="utf-8").read()
old_b = "总览行数**不变**＝档头单行原位；单位＝行"
n = s.count(old_b)
print("04 命中= %d" % n)
if n != 1:
    sys.exit("停手：04 命中数不是 1")
s = s.replace(old_b, "总览 577→**578**（两个动作分开记＝档头串单行原位替换＋条目区尾部追加 1 行）；单位＝行")
open(F2, "w", encoding="utf-8").write(s)

# 3) 底稿 18_：与入库文本保持一致
s = open(F3, encoding="utf-8").read()
n = s.count(old_a)
print("18_底稿 命中= %d" % n)
if n not in (0, 1):
    sys.exit("停手：底稿命中数异常")
if n == 1:
    s = s.replace(old_a, NEW)
    open(F3, "w", encoding="utf-8").write(s)
print("行数复核：总览=%d 04=%d" % (
    len(open(F1, encoding="utf-8").read().split("\n")) - 1,
    len(open(F2, encoding="utf-8").read().split("\n")) - 1))
print("DONE")

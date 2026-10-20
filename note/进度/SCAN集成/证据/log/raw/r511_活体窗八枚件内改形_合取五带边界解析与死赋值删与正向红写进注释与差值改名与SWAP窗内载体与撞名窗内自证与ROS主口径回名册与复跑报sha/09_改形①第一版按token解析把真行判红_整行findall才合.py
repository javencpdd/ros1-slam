import os, ast
D='note/进度/SCAN集成/证据/log/raw/r511_活体窗八枚件内改形_合取五带边界解析与死赋值删与正向红写进注释与差值改名与SWAP窗内载体与撞名窗内自证与ROS主口径回名册与复跑报sha'
p=os.path.join(D,'_cmp_r511.py')
t=open(p,encoding='utf-8').read()
old = ('    g5 = {}\n'
       '    for tok in _five[0].split("｜"):\n'
       '        mm = re.search(r"(基线|D∖K|D|K|TCP|UDP)=(-?\\d+)", tok)\n'
       '        if mm:\n'
       '            g5[mm.group(1)] = mm.group(2)\n')
new = ('    # ★改形①的第二版★：第一版按｜分 token、每 token 只认一键 ⇒ 真窗志行里「端口 TCP=0 UDP=0」同处一枚 token，\n'
       '    #   UDP 取不到 ⇒ 把★真退净行判成 False★＝又是一枚施工侧自造的约束（开窗前 dry-test 六行拦下：真值行 True、四枚反例 False、基线缺席 False）。\n'
       '    #   正解＝整行 findall、★备选里把 D∖K 放在 D 与 K 之前★ ⇒ `D∖K=0` 被当作一枚整键吃掉、不再被再捕成 K。\n'
       '    g5 = dict(re.findall(r"(D∖K|基线|D|K|TCP|UDP)=(-?\\d+)", _five[0]))\n')
assert t.count(old)==1, t.count(old)
t=t.replace(old,new)
t=t.replace('（★旧式在这五行上给出 K:=D∖K 的假覆盖★）','（★旧式 `(D|K|TCP|UDP)=` 无边界⇒`D∖K=n` 被再捕成 K、基线不入式★）')
open(p,'w',encoding='utf-8').write(t)
ast.parse(t)
print('改形① 已改整行 findall＋整键优先 ‖ 语法OK')

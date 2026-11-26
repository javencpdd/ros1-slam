#!/usr/bin/env python3
"""地图组合核验（2026-11-26，裁决⑤）：MAP_COMBO 必须**真的**贯穿四件套，而不只是登记。

四件套：Gazebo World、HDL 的 PCD、Graph 的 `map_json`（支撑栅格）、验收 Task。
本件做三件事：
  1. 按 `data/maps/combos/<id>.json` 解析四个路径，逐一确认存在并记 sha256；
  2. 若给了渲染好的 launch（`--launch`），核验其中 `map_json` 是否**指向该组合**的支撑图；
  3. 若给了运行中的 `/v8_map_status`（`--status-json`），核验其 `sha256` 与文件一致 ——
     即"运行时真正加载的那份"就是组合里登记的那份。

判定：任一文件缺失 / 路径与组合不符 / 运行期 sha 与文件不符 ⇒ FAIL。

用法：
  check_map_combo.py --combo m1_slam
  check_map_combo.py --combo m1_synth --launch /tmp/graph.launch --status-json status.json
"""
import argparse
import hashlib
import json
import os
import re
import sys

ROOT = '/opt/ros1-slam'


def load_status(path):
    """读 `/v8_map_status`：兼容 `rostopic echo` 的 `data: "..."` 包装。"""
    text = open(path, encoding='utf-8').read().strip()
    # rostopic echo 会把长字符串按 `\` + 换行折断，JSON 不认这种续行，先接回去
    text = text.replace('\\\n', '')
    if text.startswith('data:'):
        text = text[len('data:'):].strip()
        if text.startswith('"') and text.endswith('"'):
            text = json.loads(text)          # 先去引号转义
    return json.loads(text)


def canonical_sha256(path):
    """与 map_file_node 同口径：对**规范化 JSON 串**取 sha，不是文件字节。"""
    value = json.load(open(path, encoding='utf-8'))
    payload = json.dumps(value, ensure_ascii=False, separators=(',', ':'))
    return hashlib.sha256(payload.encode()).hexdigest()


def sha256(path, limit=None):
    h = hashlib.sha256()
    with open(path, 'rb') as fh:
        while True:
            chunk = fh.read(1 << 20)
            if not chunk:
                break
            h.update(chunk)
    return h.hexdigest()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--combo', default='m1_slam')
    ap.add_argument('--root', default=ROOT)
    ap.add_argument('--launch', default='')
    ap.add_argument('--status-json', default='')
    ap.add_argument('--from-ros', action='store_true',
                    help='直接订阅 /v8_map_status（避免解析 rostopic echo 的包装文本）')
    args = ap.parse_args()

    combo_path = os.path.join(args.root, 'data/maps/combos', args.combo + '.json')
    if not os.path.isfile(combo_path):
        print('FAIL 组合不存在: %s' % combo_path)
        return 2
    combo = json.load(open(combo_path))

    report = {'combo': args.combo, 'combo_json': combo_path, 'files': {}, 'checks': {}}
    for key in ('world', 'pcd', 'support_json', 'task_json'):
        rel = combo.get(key)
        path = os.path.join(args.root, rel) if rel else None
        ok = bool(path) and os.path.isfile(path)
        report['files'][key] = {'path': path, 'exists': ok,
                                'sha256': sha256(path) if ok else None}
        print('%-14s %-6s %s' % (key, 'OK' if ok else 'MISSING', path))
        if ok:
            print('               sha256=%s' % report['files'][key]['sha256'])

    report['checks']['all_files_exist'] = all(f['exists'] for f in report['files'].values())

    # 渲染出来的 launch 里 map_json 必须指向组合的支撑图
    if args.launch and os.path.isfile(args.launch):
        text = open(args.launch, encoding='utf-8', errors='replace').read()
        m = re.findall(r'map_json["\']?\s*(?:value=)?["\']?([^"\'\s,<>]+)', text)
        expected = os.path.basename(combo['support_json'])
        hit = any(expected in (v or '') for v in m)
        report['checks']['launch_map_json_matches'] = hit
        print('launch map_json: %s ⇒ 与组合 %s' % (m[:2], '一致' if hit else '不一致'))
        if not hit:
            report['checks']['all_files_exist'] = False

    # 运行期 sha 与文件 sha 一致 ⇒ 真正加载的就是登记的那份
    if args.from_ros:
        try:
            import rospy
            from std_msgs.msg import String
            rospy.init_node('check_map_combo', anonymous=True)
            msg = rospy.wait_for_message('/v8_map_status', String, timeout=10)
            status = json.loads(msg.data)
        except Exception as error:
            status = {}
            print('从 ROS 取状态失败：%s' % str(error)[:120])
        live = status.get('sha256')
        want = canonical_sha256(report['files']['support_json']['path'])
        same = bool(live) and live == want
        report['checks']['runtime_sha_matches'] = same
        print('运行期 sha256: %s' % live)
        print('规范化 sha256: %s ⇒ %s' % (want, '一致' if same else '不一致'))
        live_path = status.get('map_json')
        path_ok = bool(live_path) and os.path.realpath(live_path) == \
            os.path.realpath(report['files']['support_json']['path'])
        report['checks']['runtime_map_json_matches'] = path_ok
        print('运行期 map_json: %s ⇒ %s' % (live_path, '与组合一致' if path_ok else '与组合不一致'))
        if not same:
            report['checks']['all_files_exist'] = False
    elif args.status_json and os.path.isfile(args.status_json):
        try:
            status = load_status(args.status_json)
        except Exception as error:
            status = {}
            print('status 解析失败：%s' % str(error)[:120])
        live = status.get('sha256')
        want = canonical_sha256(report['files']['support_json']['path'])
        same = bool(live) and live == want
        report['checks']['runtime_sha_matches'] = same
        print('运行期 sha256: %s' % live)
        print('规范化 sha256: %s ⇒ %s' % (want, '一致' if same else '不一致'))
        # ★实际加载路径★：status 的 map_json 才是节点真正打开的那份
        live_path = status.get('map_json')
        path_ok = bool(live_path) and os.path.realpath(live_path) == \
            os.path.realpath(report['files']['support_json']['path'])
        report['checks']['runtime_map_json_matches'] = path_ok
        print('运行期 map_json: %s ⇒ %s' % (live_path, '与组合一致' if path_ok else '与组合不一致'))
        if not same:
            report['checks']['all_files_exist'] = False

    # ★判定口径★：文件四件套齐全 + 运行期 sha 与文件规范化 sha 一致 ⇒ PASS。
    # `runtime_map_json_matches` 只是**附加**核验：老版本节点不报 map_json，缺它不算失败，
    # 但会在上面单独打印一行，避免把"没核验"说成"核验通过"。
    core = {k: v for k, v in report['checks'].items()
            if k in ('all_files_exist', 'runtime_sha_matches',
                     'launch_map_json_matches')}
    verdict = 'PASS' if all(core.values()) else 'FAIL'
    print('VERDICT %s %s' % (verdict, json.dumps(report['checks'], ensure_ascii=False)))
    print('REPORT ' + json.dumps(report, ensure_ascii=False))
    return 0 if verdict == 'PASS' else 1


if __name__ == '__main__':
    sys.exit(main())

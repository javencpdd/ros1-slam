#!/usr/bin/env python3
"""Foxglove/rosbridge 连接核验：握手 → /rosapi/topics → 真实话题数据。

为什么要这条（2026-11-27）：端口 listen 不等于能用；`netstat 9090` 通过但桥接的是
另一个 ROS 域（11312 空图）也照样"看起来正常"。这里四件事一次验完：
  1) WebSocket 握手成功；
  2) `/rosapi/topics` 返回的话题里**必须**含 M2 的关键话题（证明桥的是 11313 这棵树）；
  3) 订阅一个真实话题并收到 ≥1 条数据（证明数据面通）；
  4) 打印 TF 树里 map→base_link 是否可查询（Foxglove 3D 面板的固定帧依赖）。

用法：foxglove_ws_check.py [ws://127.0.0.1:9090] [--topic /v8/local/pose] [--timeout 12]
只读：只发 subscribe/rosapi 调用，不发 goal/initialpose/cmd_vel。
"""
import argparse
import json
import sys

import rospy
from autobahn.twisted.websocket import WebSocketClientFactory, WebSocketClientProtocol
from twisted.internet import reactor


LATEST = []
PASSED = [False]


class Client(WebSocketClientProtocol):
    def __init__(self, argv, opts):
        self.opts = opts
        self.topics = None
        self.got_data = 0
        self.handshake_ok = False
        self.ws_edges = set()
        self.tf_ws = None
        self.done = False
        self.fail = None

    def onOpen(self):
        LATEST.append(self)
        self.handshake_ok = True
        self.sendMessage(json.dumps({'op': 'call_service',
                                     'id': 'topics',
                                     'service': '/rosapi/topics',
                                     'args': {}}).encode())

    def onMessage(self, payload, isBinary):
        try:
            msg = json.loads(payload.decode('utf-8'))
        except Exception:
            return
        if msg.get('op') == 'service_response' and msg.get('id') == 'topics':
            values = (msg.get('values') or {}).get('topics') or []
            self.topics = values
            # 关键话题：桥必须连到 M2 那棵树，而不是 11312 的空图
            need = ['/v8/local/pose', '/v8/bt/status', '/v8/local/status']
            missing = [t for t in need if t not in values]
            if missing:
                self.fail = '桥接的域不对，缺关键话题 %s' % missing
                self.finish()
                return
            self.sendMessage(json.dumps({'op': 'subscribe', 'id': 'sub',
                                         'topic': self.opts.topic,
                                         'throttle_rate': 200}).encode())
            # ★TF 必须走**客户端实际拿到的数据**判定★（2026-11-27 裁决）：
            # 本地 tf2 能查 ≠ WebSocket 客户端能拿到 TF 数据。这里真的经桥订阅 /tf 与 /tf_static。
            for _t in ('/tf', '/tf_static'):
                self.sendMessage(json.dumps({'op': 'subscribe', 'id': 'tf' + _t,
                                             'topic': _t}).encode())
        elif msg.get('op') == 'publish' and msg.get('topic') in ('/tf', '/tf_static'):
            for tr in (msg.get('msg') or {}).get('transforms') or []:
                self.ws_edges.add((tr.get('header', {}).get('frame_id', ''),
                                   tr.get('child_frame_id', '')))
        elif msg.get('op') == 'publish' and msg.get('topic') == self.opts.topic:
            self.got_data += 1
            if self.got_data >= 3:
                self._check_tf()          # 本地 ROS 侧能否查到
                self._check_tf_ws()       # 客户端经桥能否拿到 TF
                self.finish()

    def _check_tf_ws(self):
        """用**客户端收到的 /tf+/tf_static** 边集判断 map→base_link 是否连通。"""
        parent = {}
        for p, c in self.ws_edges:
            if p and c:
                parent[c] = p
        node, hops = 'base_link', 0
        while node in parent and hops < 20:
            if parent[node] == 'map':
                self.tf_ws = True
                return
            node, hops = parent[node], hops + 1
        self.tf_ws = bool(self.ws_edges) and node == 'map'

    def _check_tf(self):
        """文档声称的 TF 可查询性必须真的查一次——不能只写在 README 里。"""
        try:
            import tf2_ros
            buf = tf2_ros.Buffer()
            tf2_ros.TransformListener(buf)
            import time as _t
            t0 = _t.monotonic()
            while _t.monotonic() - t0 < 3.0:
                try:
                    buf.lookup_transform('map', 'base_link', rospy.Time(0))
                    self.tf_ok = True
                    return
                except Exception:
                    _t.sleep(0.2)
            self.tf_ok = False
        except Exception:
            self.tf_ok = None      # 无 tf2 环境时如实标 SKIP，不当作通过

    def finish(self):
        if self.done:
            return
        self.done = True
        print('HANDSHAKE %s' % ('OK' if self.handshake_ok else 'FAIL'))
        print('TOPICS n=%d m2_ok=%s' % (len(self.topics or []),
                                        bool(self.topics) and self.fail is None))
        print('DATA %s n=%d topic=%s' % ('OK' if self.got_data >= 1 else 'FAIL',
                                         self.got_data, self.opts.topic))
        tf_ok = getattr(self, 'tf_ok', None)
        tf_ws = getattr(self, 'tf_ws', None)
        # ★SKIP 不得通过★：拿不到结论就是没验过，按 FAIL 处理（2026-11-27 裁决）。
        print('TF(local ROS) map->base_link %s' % ('OK' if tf_ok else 'FAIL'))
        print('TF(ws client)   map->base_link %s  edges=%d' % ('OK' if tf_ws else 'FAIL',
                                                              len(self.ws_edges)))
        ok = (self.handshake_ok and self.fail is None and self.got_data >= 1
              and tf_ok is True and tf_ws is True)
        PASSED[0] = ok
        print('VERDICT %s' % ('PASS' if ok else 'FAIL'))
        if self.fail:
            print('REASON %s' % self.fail)
        reactor.stop()

    def onClose(self, wasClean, code, reason):
        # autobahn 关闭回调里不能再用 is_closed 之类的内部状态（实测 AttributeError）

        if not self.done:
            self.fail = self.fail or '连接被关闭 code=%s' % code
            self.finish()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('url', nargs='?', default='ws://127.0.0.1:9090')
    ap.add_argument('--topic', default='/v8/local/pose')
    ap.add_argument('--timeout', type=float, default=12.0)
    o = ap.parse_args()

    # tf2 的 Buffer/Listener 需要一个 ROS 节点上下文，否则 lookup 直接抛异常被当成 SKIP
    rospy.init_node('foxglove_ws_check', anonymous=True, disable_signals=True)
    factory = WebSocketClientFactory(o.url)
    factory.protocol = lambda: Client(sys.argv, o)
    reactor.connectTCP(o.url.split('//')[1].split(':')[0],
                       int(o.url.rsplit(':', 1)[1].split('/')[0]), factory)
    def _timeout():
        if LATEST:
            c = LATEST[0]
            c.fail = c.fail or ('超时 %.0fs 未完成' % o.timeout)
            c.finish()
        else:
            _soft_stop(o)

    reactor.callLater(o.timeout, _timeout)
    reactor.run()
    return 0 if PASSED[0] else 1


def _soft_stop(o):
    print('HANDSHAKE ?\nTOPICS ?\nDATA FAIL n=0 topic=%s\nVERDICT FAIL\nREASON 超时 %.0fs 无完整响应'
          % (o.topic, o.timeout))
    reactor.stop()


if __name__ == '__main__':
    sys.exit(main())

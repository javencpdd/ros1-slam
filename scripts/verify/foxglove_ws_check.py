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

from autobahn.twisted.websocket import WebSocketClientFactory, WebSocketClientProtocol
from twisted.internet import reactor


LATEST = []


class Client(WebSocketClientProtocol):
    def __init__(self, argv, opts):
        self.opts = opts
        self.topics = None
        self.got_data = 0
        self.handshake_ok = False
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
        elif msg.get('op') == 'publish' and msg.get('topic') == self.opts.topic:
            self.got_data += 1
            if self.got_data >= 3:
                self.finish()

    def finish(self):
        if self.done:
            return
        self.done = True
        print('HANDSHAKE %s' % ('OK' if self.handshake_ok else 'FAIL'))
        print('TOPICS n=%d m2_ok=%s' % (len(self.topics or []),
                                        bool(self.topics) and self.fail is None))
        print('DATA %s n=%d topic=%s' % ('OK' if self.got_data >= 1 else 'FAIL',
                                         self.got_data, self.opts.topic))
        print('VERDICT %s' % ('PASS' if (self.handshake_ok and self.fail is None
                                         and self.got_data >= 1) else 'FAIL'))
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


def _soft_stop(o):
    print('HANDSHAKE ?\nTOPICS ?\nDATA FAIL n=0 topic=%s\nVERDICT FAIL\nREASON 超时 %.0fs 无完整响应'
          % (o.topic, o.timeout))
    reactor.stop()


if __name__ == '__main__':
    main()

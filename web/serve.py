#!/usr/bin/env python3
"""静态站点服务器：根目录＝仓库根，文本响应补 charset=utf-8。

直接 `python3 -m http.server` 也能起，但 Markdown 会被当 Latin-1 显示成乱码；
本脚本只做两件事：把根指到仓库根、给文本类型补 charset。

用法（仓库根目录）：
    python3 web/serve.py            # http://127.0.0.1:8000/web/
    python3 web/serve.py --port 8080 --bind 0.0.0.0
"""
import argparse
import functools
import http.server
import pathlib
import socketserver

ROOT = pathlib.Path(__file__).resolve().parent.parent
TEXT_SUFFIX = {'.md', '.txt', '.html', '.css', '.js', '.json', '.yaml', '.yml', '.xml', '.pgm', '.log'}


class Handler(http.server.SimpleHTTPRequestHandler):
    extensions_map = {
        **http.server.SimpleHTTPRequestHandler.extensions_map,
        '.md': 'text/markdown; charset=utf-8',
        '.txt': 'text/plain; charset=utf-8',
        '.html': 'text/html; charset=utf-8',
        '.css': 'text/css; charset=utf-8',
        '.js': 'application/javascript; charset=utf-8',
        '.json': 'application/json; charset=utf-8',
        '.yaml': 'text/yaml; charset=utf-8',
        '.yml': 'text/yaml; charset=utf-8',
        '.xml': 'application/xml; charset=utf-8',
    }

    def end_headers(self):
        if self.path.lower().split('?')[0].endswith(tuple(TEXT_SUFFIX)):
            self.send_header('Cache-Control', 'no-cache')
        super().end_headers()

    def do_GET(self):
        # 服务根是仓库根；访问 / 直接进站点，省得记 /web/ 路径。
        if self.path in ('', '/', '/index.html'):
            self.send_response(302)
            self.send_header('Location', '/web/')
            self.end_headers()
            return
        super().do_GET()

    def do_HEAD(self):
        if self.path in ('', '/', '/index.html'):
            self.send_response(302)
            self.send_header('Location', '/web/')
            self.end_headers()
            return
        super().do_HEAD()


class Server(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


def main():
    parser = argparse.ArgumentParser(description='ros1-slam 静态站点服务器')
    parser.add_argument('--port', type=int, default=8000)
    parser.add_argument('--bind', default='127.0.0.1')
    args = parser.parse_args()
    handler = functools.partial(Handler, directory=str(ROOT))
    with Server((args.bind, args.port), handler) as httpd:
        print('serving %s' % ROOT)
        print('open    http://%s:%d/web/' % (args.bind, args.port))
        print('docs    http://%s:%d/note/进度/00_总览.md' % (args.bind, args.port))
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            pass


if __name__ == '__main__':
    main()

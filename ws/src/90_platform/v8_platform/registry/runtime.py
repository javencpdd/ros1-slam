"""Read-only runtime discovery and composition endpoint (loopback by default).

GET /components?tag=indoor&role=controller; POST /compose {"components": [...]}.
Replies retain manifest evidence levels: successful composition is not runtime
certification of the selected algorithms.
"""
import argparse
import hashlib
import json
import sys
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlsplit

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / 'composer'))
import registry
import composer


def make_server(directory, address=('127.0.0.1', 0)):
    store = composer.load(directory)
    digest = hashlib.sha256(json.dumps(
        {e['id']: e['sha256'] for e in store.all()}, sort_keys=True
    ).encode()).hexdigest()

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_args):
            pass

        def reply(self, code, value):
            body = json.dumps(dict(value, registry_digest=digest),
                              ensure_ascii=False).encode()
            self.send_response(code)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            url = urlsplit(self.path)
            if url.path != '/components':
                return self.reply(404, {'ok': False, 'reason': 'E_RUNTIME_ENDPOINT'})
            query = parse_qs(url.query, keep_blank_values=True)
            if set(query) - {'tag', 'role', 'id'}:
                return self.reply(400, {'ok': False, 'reason': 'E_RUNTIME_QUERY'})
            entries = store.all()
            for key, values in query.items():
                for value in values:
                    entries = [e for e in entries if
                               (e['id'] == value if key == 'id' else
                                value in e['tags' if key == 'tag' else 'roles'])]
            self.reply(200, {'ok': True, 'count': len(entries),
                             'components': entries,
                             'evidence': store.evidence_separation()})

        def do_POST(self):
            if self.path != '/compose':
                return self.reply(404, {'ok': False, 'reason': 'E_RUNTIME_ENDPOINT'})
            try:
                size = int(self.headers.get('Content-Length', '0'))
                if not 0 < size <= 65536:
                    raise ValueError('body must be 1..65536 bytes')
                value = json.loads(self.rfile.read(size))
                ids = value['components']
                if (not isinstance(ids, list) or not ids or
                        any(not isinstance(cid, str) for cid in ids)):
                    raise ValueError('components must be a nonempty string list')
                result = composer.compose(store, ids)
            except (ValueError, KeyError, TypeError) as error:
                return self.reply(400, {'ok': False, 'reason': 'E_RUNTIME_REQUEST',
                                        'detail': str(error)})
            except composer.ComposerError as error:
                return self.reply(409, {'ok': False, 'reason': error.code,
                                        'detail': error.detail})
            self.reply(200, result)

    return ThreadingHTTPServer(address, Handler)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--directory', default=str(HERE / 'data' / 'manifests'))
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=0)
    args = parser.parse_args(argv)
    server = make_server(args.directory, (args.host, args.port))
    print(json.dumps({'ready': True, 'host': server.server_address[0],
                      'port': server.server_address[1]}), flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == '__main__':
    main()

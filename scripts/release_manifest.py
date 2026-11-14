#!/usr/bin/env python3
"""Write a release manifest next to a tarball: sha256, git sha, file count."""
import hashlib
import json
import subprocess
import sys
from pathlib import Path


def git(*args):
    try:
        return subprocess.check_output(['git', '-C', str(Path(__file__).resolve().parents[1]), *args],
                                       text=True, stderr=subprocess.DEVNULL).strip()
    except (OSError, subprocess.CalledProcessError):
        return ''


def main():
    artifact = Path(sys.argv[1]).resolve()
    root = Path(__file__).resolve().parents[1]
    manifest = {
        'artifact': artifact.name,
        'sha256': hashlib.sha256(artifact.read_bytes()).hexdigest(),
        'bytes': artifact.stat().st_size,
        'git_head': git('rev-parse', 'HEAD'),
        'git_dirty': bool(git('status', '--porcelain')),
        'version': (root / 'VERSION').read_text().strip(),
    }
    out = artifact.with_suffix(artifact.suffix + '.manifest.json')
    out.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print(out)


if __name__ == '__main__':
    main()

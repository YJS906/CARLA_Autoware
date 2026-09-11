#!/usr/bin/env python3
"""Restore the bundled, checksummed map without replacing an existing map."""
import argparse
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import shutil
import sys
import tarfile
import tempfile


def sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def safe_relative(value):
    path = PurePosixPath(value)
    if not value or path.is_absolute() or '..' in path.parts or str(path) != value:
        raise ValueError(f'Unsafe relative path: {value!r}')
    return path


def verify(directory, expected):
    actual = {str(p.relative_to(directory)) for p in directory.rglob('*') if p.is_file()}
    if any(p.is_symlink() for p in directory.rglob('*')) or actual != set(expected):
        raise ValueError(f'Existing/extracted map file set differs: {directory}')
    for name, entry in expected.items():
        path = directory / name
        if path.stat().st_size != entry['bytes'] or sha256(path) != entry['sha256']:
            raise ValueError(f'Map checksum mismatch: {path}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--destination', required=True, type=Path, help='Root containing the restored map directory')
    args = parser.parse_args()
    bundle = Path(__file__).resolve().parents[1] / 'maps' / 'backup-20260912'
    manifest = json.loads((bundle / 'manifest.json').read_text())
    if manifest['schema_version'] != 1:
        raise ValueError('Unsupported map manifest version')
    relative = safe_relative(manifest['map_relative_path'])
    if len(relative.parts) != 1:
        raise ValueError('Map must have exactly one top-level directory')
    archive_info = manifest['archive']
    archive_name = safe_relative(archive_info['path'])
    if len(archive_name.parts) != 1:
        raise ValueError('Archive must be in the bundle directory')
    archive = bundle / str(archive_name)
    if archive.stat().st_size != archive_info['bytes'] or sha256(archive) != archive_info['sha256']:
        raise ValueError('Archive checksum mismatch')
    expected = {}
    for entry in manifest['files']:
        name = str(safe_relative(entry['path']))
        if name in expected:
            raise ValueError(f'Duplicate manifest member: {name}')
        expected[name] = entry
    root = args.destination.expanduser().resolve()
    root.mkdir(parents=True, exist_ok=True)
    target = root / str(relative)
    if target.is_symlink():
        raise ValueError(f'Refusing existing symbolic link: {target}')
    if target.exists():
        if not target.is_dir():
            raise ValueError(f'Existing target is not a directory: {target}')
        verify(target, expected)
        status = 'already_identical'
    else:
        staging = Path(tempfile.mkdtemp(prefix='.map-restore-', dir=root))
        try:
            stage_map = staging / str(relative)
            stage_map.mkdir()
            members = {f'{relative}/{name}': name for name in expected}
            seen = set()
            with tarfile.open(archive, 'r:gz') as tar:
                for member in tar:
                    safe_relative(member.name)
                    if not member.isfile() or member.name not in members or member.name in seen:
                        raise ValueError(f'Unexpected or unsafe archive member: {member.name}')
                    name = members[member.name]
                    if member.size != expected[name]['bytes']:
                        raise ValueError(f'Unexpected archive member size: {name}')
                    seen.add(member.name)
                    destination = stage_map / name
                    destination.parent.mkdir(parents=True, exist_ok=True)
                    source = tar.extractfile(member)
                    if source is None:
                        raise ValueError(f'Cannot read archive member: {name}')
                    with source, destination.open('xb') as output:
                        shutil.copyfileobj(source, output)
                    destination.chmod(0o644)
            if seen != set(members):
                raise ValueError('Archive is missing map files')
            verify(stage_map, expected)
            # Reserve the name with an empty directory: rename replaces only this
            # empty reservation and cannot silently overwrite a populated map.
            target.mkdir()
            try:
                os.rename(stage_map, target)
            except BaseException:
                target.rmdir()
                raise
            status = 'restored'
        finally:
            shutil.rmtree(staging)
    print(json.dumps({'status': status, 'map_root': str(root), 'map_directory': str(target),
                      'map_relative_path': str(relative), 'files_verified': len(expected)}, ensure_ascii=False))


if __name__ == '__main__':
    try:
        main()
    except Exception as error:
        print(json.dumps({'status': 'error', 'error': str(error)}, ensure_ascii=False), file=sys.stderr)
        sys.exit(1)

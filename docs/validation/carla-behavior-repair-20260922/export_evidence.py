"""Lossless evidence archive. Never changes the original trial recordings."""
from pathlib import Path
import gzip
import hashlib
import json
import shutil

src = Path('/tmp/carla-behavior-repair-20260922')
dst = Path('/home/a/carla_pp/docs/validation/carla-behavior-repair-20260922')
dst.mkdir(parents=True, exist_ok=True)
compressed = []

def copy_file(p, target, compress=False):
    target.parent.mkdir(parents=True, exist_ok=True)
    if compress:
        raw = p.read_bytes()
        target = target.with_name(target.name + '.gz')
        target.write_bytes(gzip.compress(raw, compresslevel=9, mtime=0))
        compressed.append({'file':str(target.relative_to(dst)), 'raw_bytes':len(raw),
                           'raw_sha256':hashlib.sha256(raw).hexdigest(),
                           'compressed_bytes':target.stat().st_size})
    else:
        shutil.copy2(p, target)

for p in src.iterdir():
    if p.is_file() and (p.name in {'before_scene.json','run_final_suite.py','run_observed.py',
            'restore_scene.py','export_evidence.py'} or p.name.startswith(('runtime-',
            'map-', 'aeb-final-tests.', 'bridge-final-tests.', 'build', 'final-suite.',
            'scene_restore.', 'start-', 'stop-', 'final-state.'))):
        copy_file(p, dst/p.name)
for folder in ('final','attempts'):
    for p in (src/folder).rglob('*'):
        if not p.is_file():continue
        target = dst/folder/p.relative_to(src/folder)
        # Large observer streams and previous attempts are compressed, losslessly.
        copy_file(p, target, p.suffix=='.json' and (p.name.endswith('_aeb.json') or folder=='attempts'))
for name in ('carla_map_report.json','carla_lane_change_report.json'):
    copy_file(Path('/home/a/autoware_data/maps/Town05')/name, dst/'map'/name)
(dst/'compressed-records.json').write_text(json.dumps(compressed, indent=2)+'\n')
manifest = {}
for p in sorted(dst.rglob('*')):
    if p.is_file() and p.name != 'SHA256SUMS.json':
        manifest[str(p.relative_to(dst))] = hashlib.sha256(p.read_bytes()).hexdigest()
(dst/'SHA256SUMS.json').write_text(json.dumps(manifest, indent=2)+'\n')
print(json.dumps({'files':len(manifest),'compressed_records':len(compressed),
                  'destination':str(dst)}))

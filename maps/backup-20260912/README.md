# Current map backup — 2026-09-12

This archive contains the active `HL_FMA_VTD_LivingLab_topology_fixed` map: the current `lanelet2_map.osm` (including local map changes), `map_projector_info.yaml`, `pointcloud_map.pcd`, and `HL_FMA_VTD_LivingLab_laneoffset_fixed.xodr`. The PCD is the existing small placeholder used by this installation, not a full surveyed point cloud.

Restore from the repository root:

```bash
python3 scripts/restore-map-backup.py --destination "$HOME/autoware-maps-0912"
```

Use the returned `map_directory` as the Autoware map directory. The script verifies the archive and every file against `manifest.json`, refuses links and unexpected archive members, and never replaces different existing map files. Running it again with an identical destination is safe. The source map is not modified.

`map.tar.gz` uses deterministic timestamps, ownership, ordering, and gzip headers. It is compressed to stay below GitHub's normal Git file-size limit; Git LFS is not required. This backup does not include rosbag recordings or historical map backups.

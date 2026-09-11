#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
cd "${repo_root}"

failures=0
pass() { printf 'PASS %s\n' "$1"; }
fail() { printf 'FAIL %s\n' "$1" >&2; failures=$((failures + 1)); }
skip() { printf 'SKIPPED %s\n' "$1"; }

active_files=(
  autoware
  set_route
  vtd_bridge
  docker/vtd
  scripts/build-vtd-image.sh
  scripts/set_route_from_csv.py
  scripts/csv_route_preview.py
  scripts/csv_preview_runtime.sh
)

if rg -n 'autoware_vtd_overlay|/home/[[:alnum:]_.-]+/autoware(/|/src|/build|/install)' "${active_files[@]}" >/tmp/selfcar-repro-paths 2>/dev/null; then
  cat /tmp/selfcar-repro-paths >&2
  fail "active runtime path contains a developer home/overlay path"
else
  pass "active runtime path has no developer home/overlay reference"
fi

# Image-layer library copies are expected; only runtime Compose files must be
# independent of shared libraries built on the host.
if rg -n '\.so' docker/vtd/compose.yaml docker/examples/basic/dev-nvidia.compose.yaml 2>/dev/null; then
  fail "runtime compose contains a shared-library reference"
else
  pass "runtime compose has no host .so mount"
fi

if git ls-files '*.so' '*.so.*' '*.a' | grep -q .; then
  fail "binary libraries are tracked by Git"
else
  pass "no .so/.a is tracked"
fi

if git ls-files | rg '(^|/)(build|install|log)/' >/tmp/selfcar-repro-artifacts 2>/dev/null; then
  cat /tmp/selfcar-repro-artifacts >&2
  fail "build/install/log artifact is tracked"
else
  pass "build/install/log are not tracked"
fi

for required in \
  vtd_overlay/origin.yaml \
  config/vtd/config-origin.yaml \
  repositories/autoware.lock.repos \
  .env.example; do
  if [[ -s "${required}" ]]; then pass "required manifest exists: ${required}"; else fail "missing required manifest: ${required}"; fi
done

if rg -n '^\s+version:\s+(?![0-9a-f]{40}$)' repositories/autoware.lock.repos --pcre2 >/tmp/selfcar-repro-lock 2>/dev/null; then
  cat /tmp/selfcar-repro-lock >&2
  fail "lock manifest contains a non-SHA version"
else
  pass "lock manifest versions are full commit SHAs"
fi

if rg -n 'AUTOWARE_BASE_IMAGE=.*@sha256:[0-9a-f]{64}' .env.example >/dev/null; then
  pass "base image is pinned by digest"
else
  fail "base image digest is missing"
fi

custom_packages=(
  autoware_mission_planner_universe
  autoware_path_optimizer
  autoware_behavior_path_lane_change_module
  autoware_behavior_path_avoidance_by_lane_change_module
  autoware_behavior_path_external_request_lane_change_module
  autoware_behavior_path_planner
  autoware_behavior_path_planner_common
  autoware_behavior_path_static_obstacle_avoidance_module
  autoware_behavior_velocity_intersection_module
  autoware_behavior_velocity_traffic_light_module
  vtd_ros2_bridge
)
for package in "${custom_packages[@]}"; do
  package_xml=$(find vtd_overlay/src -type f -name package.xml -exec awk -v p="${package}" '/<name>/{gsub(/^[[:space:]]+|[[:space:]]+$/, "", $0); if ($0 ~ "<name>" p "</name>") print FILENAME}' {} + | head -n 1)
  if [[ -n "${package_xml}" ]]; then
    package_dir="$(dirname "${package_xml}")"
    if [[ -f "${package_dir}/CMakeLists.txt" ]]; then
      pass "complete custom package: ${package}"
    else
      fail "custom package lacks CMakeLists.txt: ${package}"
    fi
  else
    fail "custom package is missing: ${package}"
  fi
done

if docker compose --env-file .env.example -f docker/vtd/compose.yaml config --quiet >/dev/null 2>&1; then
  pass "docker compose config"
else
  fail "docker compose config"
fi

if git diff --check; then pass "git diff --check"; else fail "git diff --check"; fi

if command -v colcon >/dev/null 2>&1; then
  if colcon list --base-paths vtd_overlay/src >/tmp/selfcar-repro-colcon 2>&1; then
    for package in "${custom_packages[@]}"; do
      if grep -q "^${package}[[:space:]]" /tmp/selfcar-repro-colcon; then pass "colcon discovers ${package}"; else fail "colcon misses ${package}"; fi
    done
  else
    cat /tmp/selfcar-repro-colcon >&2
    fail "colcon package discovery"
  fi
else
  skip "colcon package discovery (colcon is not installed)"
fi

if (( failures > 0 )); then
  printf '%s\n' "${failures} reproducibility checks failed." >&2
  exit 1
fi
printf '%s\n' 'All reproducibility checks passed.'

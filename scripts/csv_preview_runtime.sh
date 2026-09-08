#!/usr/bin/env bash
# Shared CSV state and publisher ownership for the bridge/Autoware/CSV launchers.
# The bridge is the primary publisher. A standalone worker is used only without
# a managed bridge. None of these helpers submit a driving route.

prepare_csv_preview() {
  local preview_map="${VTD_MAP_DIR}/${map_relative_path}/lanelet2_map.osm"
  local preview_map_key
  if [[ ! -r "${preview_map}" ]]; then
    echo "CSV preview map is not readable: ${preview_map}" >&2
    return 1
  fi
  csv_preview_root="${AUTOWARE_CSV_PREVIEW_DIR:-${XDG_STATE_HOME:-${HOME}/.local/state}/selfcar/csv-preview}"
  mkdir -p -m 700 -- "${csv_preview_root}"
  csv_preview_root="$(realpath -- "${csv_preview_root}")"
  preview_map_key="$(printf '%s' "$(realpath -- "${preview_map}")" | sha256sum | cut -c1-20)"
  csv_preview_state="/opt/selfcar-preview/state/${preview_map_key}.json"
  csv_preview_csv="$(realpath -m -- "${AUTOWARE_CSV_PREVIEW_CSV:-${HOME}/route_example.csv}")"
}

# Caller holds runtime.lock. Only the owned visualization worker is stopped;
# durable state stays on the host, and no bridge/Autoware process is controlled.
stop_standalone_csv_preview() {
  local preview_container=selfcar-csv-route-preview
  local preview_owner
  if ! docker inspect "${preview_container}" >/dev/null 2>&1; then
    return
  fi
  preview_owner="$(docker inspect --format '{{index .Config.Labels "selfcar.csv-preview.owner"}}' "${preview_container}")"
  if [[ "${preview_owner}" != "${repo_root}" ]]; then
    echo "CSV preview container belongs to another workspace: ${preview_owner}" >&2
    return 1
  fi
  if [[ "$(docker inspect --format '{{.State.Running}}' "${preview_container}")" == true ]]; then
    docker stop --timeout 5 "${preview_container}" >/dev/null
  fi
}

ensure_csv_preview() {
  prepare_csv_preview
  local preview_container=selfcar-csv-route-preview
  local preview_map="${VTD_MAP_DIR}/${map_relative_path}/lanelet2_map.osm"
  local preview_image_id preview_fingerprint preview_owner preview_existing
  local preview_lock_fd preview_running bridge_preview bridge_root bridge_state
  local preview_csv_mounts=()
  # Serialize bridge starts and standalone publisher selection.
  exec {preview_lock_fd}>"${csv_preview_root}/runtime.lock"
  flock -w 15 "${preview_lock_fd}"
  bridge_preview="$(docker inspect --format '{{.State.Running}} {{index .Config.Labels "selfcar.csv-preview.owner"}}' selfcar-vtd-bridge 2>/dev/null || true)"
  if [[ "${bridge_preview}" == "true ${repo_root}" ]]; then
    bridge_root="$(docker inspect --format '{{index .Config.Labels "selfcar.csv-preview.state-root"}}' selfcar-vtd-bridge)"
    bridge_state="$(docker inspect --format '{{index .Config.Labels "selfcar.csv-preview.state"}}' selfcar-vtd-bridge)"
    if [[ "${bridge_root}" != "${csv_preview_root}" || "${bridge_state}" != "${csv_preview_state}" ]]; then
      echo 'The running bridge uses another CSV state/map. Restart the bridge with the current .env.' >&2
      flock -u "${preview_lock_fd}"
      exec {preview_lock_fd}>&-
      return 1
    fi
    stop_standalone_csv_preview
    flock -u "${preview_lock_fd}"
    exec {preview_lock_fd}>&-
    printf 'CSV preview: published by selfcar-vtd-bridge (state: %s)\n' "${csv_preview_state}"
    return
  fi
  preview_image_id="$(docker image inspect --format '{{.Id}}' "${vtd_image}")"
  preview_fingerprint="$(
    {
      printf '%s\n' "${preview_image_id}" "${repo_root}" "${csv_preview_root}" \
        "${preview_map}" "${ROS_DOMAIN_ID:-0}" "${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}" \
        "${csv_preview_csv}" 'bootstrap-rw-v1'
      sha256sum "${repo_root}/scripts/csv_route_preview.py" \
        "${repo_root}/scripts/set_route_from_csv.py" "${preview_map}"
    } | sha256sum | cut -d ' ' -f 1
  )"
  preview_existing="$(docker inspect --format '{{index .Config.Labels "selfcar.csv-preview.fingerprint"}}' "${preview_container}" 2>/dev/null || true)"
  if [[ -n "${preview_existing}" ]]; then
    preview_owner="$(docker inspect --format '{{index .Config.Labels "selfcar.csv-preview.owner"}}' "${preview_container}")"
    if [[ "${preview_owner}" != "${repo_root}" ]]; then
      echo "CSV preview container belongs to another workspace: ${preview_owner}" >&2
      return 1
    fi
    if [[ "${preview_existing}" != "${preview_fingerprint}" ]]; then
      # All durable data is on the host. Only this owned visualization worker is replaced.
      docker rm --force "${preview_container}" >/dev/null
      preview_existing=""
    fi
  fi
  if [[ -z "${preview_existing}" ]]; then
    if [[ -f "${csv_preview_csv}" && -r "${csv_preview_csv}" ]]; then
      preview_csv_mounts+=(--volume "${csv_preview_csv}:/opt/selfcar-preview/input/route.csv:ro")
    else
      echo "CSV bootstrap file unavailable; only existing saved state can be restored: ${csv_preview_csv}" >&2
    fi
    docker run --detach \
      --name "${preview_container}" \
      --label "selfcar.csv-preview.owner=${repo_root}" \
      --label "selfcar.csv-preview.fingerprint=${preview_fingerprint}" \
      --restart unless-stopped \
      --network host --ipc host \
      --user "$(id -u):$(id -g)" \
      --log-opt max-size=10m --log-opt max-file=2 \
      --entrypoint /bin/bash \
      --env "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}" \
      --env "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}" \
      --env ROS_LOG_DIR=/tmp/selfcar-csv-preview-logs \
      --env "AUTOWARE_CSV_SOURCE_PATH=${csv_preview_csv}" \
      --volume "${csv_preview_root}:/opt/selfcar-preview/state" \
      --volume "${VTD_MAP_DIR}:/home/aw/vtd_autoware_maps:ro" \
      --volume "${repo_root}/scripts/csv_route_preview.py:/opt/selfcar-tools/csv_route_preview.py:ro" \
      --volume "${repo_root}/scripts/set_route_from_csv.py:/opt/selfcar-tools/set_route_from_csv.py:ro" \
      "${preview_csv_mounts[@]}" \
      "${vtd_image}" -lc \
      'source /opt/ros/jazzy/setup.bash; source /opt/autoware/setup.bash; source /opt/selfcar_overlay/setup.bash; exec python3 /opt/selfcar-tools/csv_route_preview.py --state "$1" --map "$2" --csv /opt/selfcar-preview/input/route.csv' \
      csv-preview "${csv_preview_state}" \
      "/home/aw/vtd_autoware_maps/${map_relative_path}/lanelet2_map.osm" >/dev/null
  else
    preview_running="$(docker inspect --format '{{.State.Running}}' "${preview_container}")"
    if [[ "${preview_running}" != true ]]; then
      docker start "${preview_container}" >/dev/null
    fi
  fi
  flock -u "${preview_lock_fd}"
  exec {preview_lock_fd}>&-
  printf 'CSV preview: %s (saved state: %s/%s)\n' \
    "${preview_container}" "${csv_preview_root}" "${csv_preview_state##*/}"
}

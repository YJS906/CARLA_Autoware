#!/usr/bin/env bash
set -euo pipefail

CARLA_ROOT=/home/a/CARLA/0.9.16
CARLA_PYTHON=/home/a/CARLA/venv-0.9.16/bin/python
SCENARIO_RUNNER_ROOT=/home/a/scenario_runner
SCENARIO_ROOT=/home/a/carla_pp/scenarios/town05

export PYTHONPATH="${CARLA_ROOT}/PythonAPI/carla${PYTHONPATH:+:${PYTHONPATH}}"
export PYTHONPYCACHEPREFIX="/tmp/carla-scenario-pycache-${UID}"
mkdir -p "${PYTHONPYCACHEPREFIX}"

extra_args=()
scenario_name=Town05CityScenario
scenario_config="${SCENARIO_ROOT}/town05_city.xml"
for arg in "$@"; do
    case "${arg}" in
      --hazards)
        scenario_name=Town05Hazards
        scenario_config="${SCENARIO_ROOT}/town05_hazards.xml"
        ;;
      --wait-for-ego) extra_args+=(--waitForEgo) ;;
      *) extra_args+=("${arg}") ;;
    esac
done

cd "${SCENARIO_RUNNER_ROOT}"
exec "${CARLA_PYTHON}" scenario_runner.py \
    --scenario "${scenario_name}" \
    --additionalScenario "${SCENARIO_ROOT}/town05_city.py" \
    --configFile "${scenario_config}" \
    --output \
    "${extra_args[@]}"

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
if [[ "${1:-}" == "--wait-for-ego" ]]; then
    extra_args+=(--waitForEgo)
    shift
fi

cd "${SCENARIO_RUNNER_ROOT}"
exec "${CARLA_PYTHON}" scenario_runner.py \
    --scenario Town05CityScenario \
    --additionalScenario "${SCENARIO_ROOT}/town05_city.py" \
    --configFile "${SCENARIO_ROOT}/town05_city.xml" \
    --output \
    "${extra_args[@]}" \
    "$@"

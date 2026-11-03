#!/usr/bin/env bash
# Isolated real hospital/robot contact test. Stop hospital-demo before running.
set -eo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$HERE/ros_env.sh"
RUN="${1:?Provide an owned test-output directory}"
mkdir -p -- "$RUN"
export HORUS_HOSPITAL_SOURCE="$HERE"
export HORUS_HOSPITAL_RUN="$RUN"
export HORUS_HOSPITAL_SCENE="$RUN/contact.usda"
export HORUS_HOSPITAL_CONTACT_TEST=1
"${HORUS_ISAAC_HOME:-$HOME/isaac}/env_isaacsim/bin/python" "$HERE/prepare_scene.py" \
    --asset-root "${HORUS_ISAAC_ASSET_ROOT:-/media/omotoye/New Volume/IsaacSim/assets-6.2.0/Assets/Isaac/6.2}" \
    --output "$HORUS_HOSPITAL_SCENE" --contact-fixture
timeout --signal=INT --kill-after=5s 180s bash "$HERE/run_sim.sh" > "$RUN/simulation.log" 2>&1
/usr/bin/python3 -c 'import json,sys; d=json.load(open(sys.argv[1])); print(json.dumps(d,indent=2)); assert d["passed"]' "$RUN/contact-results.json"

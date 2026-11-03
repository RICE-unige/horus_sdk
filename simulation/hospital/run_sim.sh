#!/usr/bin/env bash
set -eo pipefail
ISAAC_HOME="${HORUS_ISAAC_HOME:-$HOME/isaac}"
source "$ISAAC_HOME/scripts/activate_isaac.sh"
unset ROS_DISTRO AMENT_PREFIX_PATH COLCON_PREFIX_PATH OLD_PYTHONPATH
export ROS_DISTRO=jazzy
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH="$ISAACLAB_ENV/lib/python3.12/site-packages/isaacsim/exts/isaacsim.ros2.core/jazzy/lib"
exec "$ISAACLAB_ENV/bin/isaacsim" isaacsim.exp.base.python --no-window \
    --portable-root "$ISAACSIM_PORTABLE_ROOT" \
    --merge-config="$ISAAC_HOME/config/isaacsim_local_assets.kit" \
    --enable isaacsim.ros2.bridge --enable isaacsim.code_editor.python_server \
    --/exts/isaacsim.code_editor.python_server/host=127.0.0.1 \
    --/exts/isaacsim.code_editor.python_server/port="${HORUS_HOSPITAL_KIT_PORT:-8226}" \
    --/plugins/carb.tasking.plugin/threadCount=12 \
    --/plugins/omni.tbb.globalcontrol/maxThreadCount=12 \
    --/app/window/hideUi=true \
    --exec "$HORUS_HOSPITAL_SOURCE/bootstrap.py"

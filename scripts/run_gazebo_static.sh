#!/usr/bin/env bash
set -Eeuo pipefail
PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
PERSONAL_WS="$(dirname -- "${PROJECT_ROOT}")"
MODEL_ROOT="${PERSONAL_WS}/src/vrx/vrx_gz/models"
WAMV_MODEL_ROOT="${PERSONAL_WS}/src/vrx/vrx_urdf/wamv_description/models"
for asset in dock_2022_base/model.sdf dock_block_4x4/model.sdf dock_block_4x4/mesh/dock_block_4x4.dae; do
  if [[ ! -f "${MODEL_ROOT}/${asset}" ]]; then
    echo "Missing VRX resource: ${MODEL_ROOT}/${asset}" >&2
    exit 1
  fi
done
# Direct Gazebo launch: no ROS nodes, bridges, spawning or vehicle commands.
export GZ_SIM_RESOURCE_PATH="${MODEL_ROOT}:${GZ_SIM_RESOURCE_PATH:-}"
export GZ_SIM_RESOURCE_PATH="${WAMV_MODEL_ROOT}:${GZ_SIM_RESOURCE_PATH}"
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
export GZ_PARTITION="dock_planner_static_${USER:-user}"
# Keep application logs/config/cache inside this experiment directory.
mkdir -p "${PROJECT_ROOT}/generated/runtime" "${PROJECT_ROOT}/generated/logs"
export GZ_HOMEDIR="${PROJECT_ROOT}/generated/runtime"
export XDG_CACHE_HOME="${PROJECT_ROOT}/generated/runtime/cache"
export XDG_CONFIG_HOME="${PROJECT_ROOT}/generated/runtime/config"
cd "${PROJECT_ROOT}"
python3 -B -m examples.gazebo_static
# Deliberately omit -r: start paused. The world contains only static models.
exec gz sim -v 4 "${PROJECT_ROOT}/generated/vrx_dock_static.sdf" "$@"

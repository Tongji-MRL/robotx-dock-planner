#!/usr/bin/env bash
set -Eeuo pipefail

PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
PERSONAL_WS="$(dirname -- "${PROJECT_ROOT}")"
MODEL_ROOT="${PERSONAL_WS}/src/vrx/vrx_gz/models"
WAMV_MODEL_ROOT="${PERSONAL_WS}/src/vrx/vrx_urdf/wamv_description/models"

for asset in dock_2022_base/model.sdf WAM-V-Base/mesh/WAM-V-Base.dae; do
  if [[ "${asset}" == WAM-V-Base/* ]]; then
    test -f "${WAMV_MODEL_ROOT}/${asset}" || { echo "Missing ${WAMV_MODEL_ROOT}/${asset}" >&2; exit 1; }
  else
    test -f "${MODEL_ROOT}/${asset}" || { echo "Missing ${MODEL_ROOT}/${asset}" >&2; exit 1; }
  fi
done

export GZ_SIM_RESOURCE_PATH="${MODEL_ROOT}:${WAMV_MODEL_ROOT}:${GZ_SIM_RESOURCE_PATH:-}"
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
export GZ_PARTITION="task3_geofence_search_${USER:-user}"
mkdir -p "${PROJECT_ROOT}/generated/runtime_geofence" "${PROJECT_ROOT}/generated/logs"
export GZ_HOMEDIR="${PROJECT_ROOT}/generated/runtime_geofence"
export XDG_CACHE_HOME="${PROJECT_ROOT}/generated/runtime_geofence/cache"
export XDG_CONFIG_HOME="${PROJECT_ROOT}/generated/runtime_geofence/config"

cd "${PROJECT_ROOT}"
python3 -B -m examples.gazebo_geofence_search

echo "Gazebo: 灰色=geofence，青色=主搜索线，蓝色=备用线，绿色=三个入口。"
echo "橙色 WAM-V 将沿主搜索线往返演示；本场景不发送真实推进器指令。"

gz sim -r -v 3 "${PROJECT_ROOT}/generated/task3_geofence_search.sdf" "$@" &
GAZEBO_PID=$!
ANIMATION_PID=""
cleanup() {
  if [[ -n "${ANIMATION_PID}" ]]; then
    kill "${ANIMATION_PID}" 2>/dev/null || true
  fi
  kill "${GAZEBO_PID}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

if [[ "${TASK3_GZ_ANIMATE:-1}" == "1" ]]; then
  python3 -B -m examples.gazebo_geofence_animation \
    "${PROJECT_ROOT}/generated/task3_geofence_search.json" &
  ANIMATION_PID=$!
fi
wait "${GAZEBO_PID}"

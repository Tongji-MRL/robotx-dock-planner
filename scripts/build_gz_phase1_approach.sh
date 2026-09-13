#!/usr/bin/env bash
set -Eeuo pipefail
PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "${PROJECT_ROOT}/generated"
read -r -a GZ_FLAGS <<< "$(pkg-config --cflags --libs gz-transport12 gz-msgs9)"
g++ "${PROJECT_ROOT}/scripts/gz_phase1_approach.cc" \
  -O2 -Wall -Wextra -Wpedantic \
  "${GZ_FLAGS[@]}" \
  -o "${PROJECT_ROOT}/generated/gz_phase1_approach"
echo "Built ${PROJECT_ROOT}/generated/gz_phase1_approach"

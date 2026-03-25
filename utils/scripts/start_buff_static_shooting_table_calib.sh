#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
BUILD_DIR="${REPO_ROOT}/build"

mkdir -p "${BUILD_DIR}"
cmake -S "${REPO_ROOT}" -B "${BUILD_DIR}" \
  -UBoost_DIR \
  -Uboost_headers_DIR \
  -Uboost_system_DIR \
  -DCMAKE_IGNORE_PREFIX_PATH="${CONDA_PREFIX:-}"
cmake --build "${BUILD_DIR}" --target BuffStaticShootingTableCalibration -j"$(nproc)"

cd "${REPO_ROOT}/bin"
exec ./BuffStaticShootingTableCalibration

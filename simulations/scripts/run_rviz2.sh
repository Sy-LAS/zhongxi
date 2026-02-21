#!/usr/bin/env bash
set -euo pipefail

# 载入 rviz2 环境变量（如果存在）
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
[ -f "${ROOT_DIR}/env/rviz2.env" ] && source "${ROOT_DIR}/env/rviz2.env"

# 参数：配置文件路径（可选）
RVIZ_CONFIG="${1:-${ROOT_DIR}/rviz/default.rviz}"

echo "[run_rviz2] QT_QPA_PLATFORM=${QT_QPA_PLATFORM:-unset}  LIBGL_ALWAYS_SOFTWARE=${LIBGL_ALWAYS_SOFTWARE:-unset}"
echo "[run_rviz2] Using config: ${RVIZ_CONFIG}"

exec rviz2 -d "${RVIZ_CONFIG}"

#!/usr/bin/env bash
set -euo pipefail
install -d -m 0777 /vscode-wayland-runtime
if [ -S /vscode-wayland/wayland-0 ]; then
ln -snf /vscode-wayland/wayland-0 /vscode-wayland-runtime/wayland-0
echo "[postStart] wayland runtime prepared"
else
echo "[postStart] wayland socket not found at /vscode-wayland/wayland-0" >&2
fi
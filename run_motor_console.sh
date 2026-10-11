#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ -f "$script_dir/CMakeLists.txt" ]]; then
    repo_root="$script_dir"
else
    repo_root="$(cd "$script_dir/.." && pwd)"
fi
cd "$repo_root"

require_etherlab=ON
case "${1:-}" in
    --check-config|--help|-h) require_etherlab=OFF ;;
esac

cmake_args=(-S . -B build
    -UETHERLAB_INCLUDE_DIR -UETHERLAB_LIBRARY
    -DENABLE_ETHERLAB=ON "-DREQUIRE_ETHERLAB=$require_etherlab"
    "-DETHERLAB_ROOT=${ETHERLAB_ROOT:-}")

printf '\n%s\n' '==================== EtherLab 启动检查与编译：开始 ===================='
trap 'printf "%s\n\n" "==================== EtherLab 启动检查与编译：失败 ===================="' EXIT

if (( EUID == 0 )); then
    if [[ -z "${SUDO_USER:-}" || "$SUDO_USER" == root ]]; then
        printf '%s\n' '请通过普通用户的 sudo 启动，以便编译时使用原用户权限。' >&2
        exit 1
    fi
    sudo -u "$SUDO_USER" -- cmake "${cmake_args[@]}"
    sudo -u "$SUDO_USER" -- cmake --build build --parallel 4
else
    cmake "${cmake_args[@]}"
    cmake --build build --parallel 4
fi

trap - EXIT
printf '%s\n\n' '==================== EtherLab 启动检查与编译：完成 ===================='

if (( EUID == 0 )) || [[ "$require_etherlab" == OFF ]]; then
    exec ./build/internal/motor_console "$@"
fi
exec sudo ./build/internal/motor_console "$@"

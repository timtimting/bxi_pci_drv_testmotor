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

configure_etherlab_master1() {
    local interface=enx6c1ff7c503f3
    local config=/etc/sysconfig/ethercat
    local address_file="/sys/class/net/$interface/address"
    local configured=""
    local line
    local address

    if [[ ! -r "$address_file" ]]; then
        printf '未检测到第二张网卡 %s，保留现有 EtherLab 主站配置。\n' "$interface"
        return
    fi
    if [[ ! -f "$config" ]]; then
        printf '未找到 %s，无法配置 EtherLab 主站 1。\n' "$config" >&2
        return 1
    fi
    address="$(< "$address_file")"
    while IFS= read -r line; do
        if [[ "$line" =~ ^[[:space:]]*MASTER1_DEVICE[[:space:]]*= ]]; then
            configured="${line#*=}"
            configured="${configured%%#*}"
            configured="$(printf '%s' "$configured" | tr -d "[:space:]\"'")"
        fi
    done < "$config"
    if [[ -n "$configured" && "${configured,,}" != "${address,,}" ]]; then
        printf 'MASTER1_DEVICE 已绑定 %s，与 %s 的 MAC %s 不一致；未覆盖现有配置。\n' \
            "$configured" "$interface" "$address" >&2
        return 1
    fi
    if [[ -z "$configured" ]]; then
        if [[ -c /dev/EtherCAT0 ]]; then
            printf 'EtherLab 主站 0 正在运行；为避免中断控制，未配置主站 1。请先失能、退出占用主站的程序，再停止 EtherLab 服务并重新运行本脚本。\n' >&2
            return 1
        fi
        if (( EUID == 0 )); then
            printf '\nMASTER1_DEVICE="%s"\n' "$address" >> "$config"
        else
            printf '\nMASTER1_DEVICE="%s"\n' "$address" | sudo tee -a "$config" >/dev/null
        fi
        printf '已在 %s 配置主站 1: %s (%s)\n' "$config" "$interface" "$address"
    fi
}

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

if [[ "$require_etherlab" == ON ]]; then
    configure_etherlab_master1
    if [[ ! -c /dev/EtherCAT0 || ! -r /sys/module/ec_master/parameters/main_devices ]]; then
        if [[ ! -x /etc/init.d/ethercat ]]; then
            printf '%s\n' '未找到 /etc/init.d/ethercat；请先安装并配置 EtherLab 内核主站。' >&2
            exit 1
        fi
        printf '%s\n' 'EtherLab 内核主站未就绪，正在启动 /etc/init.d/ethercat'
        if (( EUID == 0 )); then
            /etc/init.d/ethercat start
        else
            sudo /etc/init.d/ethercat start
        fi
    fi
    if [[ ! -c /dev/EtherCAT0 || ! -r /sys/module/ec_master/parameters/main_devices ]]; then
        printf '%s\n' 'EtherLab 内核主站启动后仍不可用；请检查 /etc/sysconfig/ethercat 和 /dev/EtherCAT0。' >&2
        exit 1
    fi
    printf '%s\n' 'EtherLab 内核主站已就绪: /dev/EtherCAT0'
    if [[ -r /sys/class/net/enx6c1ff7c503f3/address ]]; then
        expected_mac="$(< /sys/class/net/enx6c1ff7c503f3/address)"
        IFS=, read -r main_mac backup_mac < /sys/module/ec_master/parameters/main_devices
        backup_mac="${backup_mac//[[:space:]]/}"
        if [[ ! -c /dev/EtherCAT1 || "${backup_mac,,}" != "${expected_mac,,}" ]]; then
            printf '主站 1 未绑定 enx6c1ff7c503f3 (%s)；请先失能、退出占用主站的程序，再重启 EtherLab 服务。\n' "$expected_mac" >&2
            exit 1
        fi
        printf '%s\n' 'EtherLab 内核主站已就绪: /dev/EtherCAT1 (enx6c1ff7c503f3)'
    fi
fi

trap - EXIT
printf '%s\n\n' '==================== EtherLab 启动检查与编译：完成 ===================='

if (( EUID == 0 )) || [[ "$require_etherlab" == OFF ]]; then
    exec ./build/internal/motor_console "$@"
fi
exec sudo ./build/internal/motor_console "$@"

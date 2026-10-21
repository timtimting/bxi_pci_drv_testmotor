#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ -f "$script_dir/CMakeLists.txt" ]]; then
    repo_root="$script_dir"
else
    repo_root="$(cd "$script_dir/.." && pwd)"
fi
cd "$repo_root"

if [[ "$(uname -m)" == aarch64 ]]; then
    ethercat_interfaces=(eth0 eth1)
else
    ethercat_interfaces=(enp86s0 enx6c1ff7c503f3)
fi

require_etherlab=ON
case "${1:-}" in
    --check-config|--help|-h|--power-off) require_etherlab=OFF ;;
esac

cmake_args=(-S . -B build
    -UETHERLAB_INCLUDE_DIR -UETHERLAB_LIBRARY
    -DENABLE_ETHERLAB=ON "-DREQUIRE_ETHERLAB=$require_etherlab"
    "-DETHERLAB_ROOT=${ETHERLAB_ROOT:-}")

configure_etherlab_masters() {
    local config=/etc/sysconfig/ethercat
    local addresses=()
    local configured_devices=()
    local active_devices=()
    local line
    local configured
    local configured_modules=""
    local device
    local number
    local active_mac
    local swapped_bindings=false
    local needs_restart=false
    local needs_configuration=false
    local master_in_use=false

    if [[ ! -f "$config" ]]; then
        printf '未找到 %s，无法配置 EtherLab 主站。\n' "$config" >&2
        return 1
    fi
    if [[ ! -x /etc/init.d/ethercat ]]; then
        printf '%s\n' '未找到 /etc/init.d/ethercat，无法安全重启 EtherLab 服务。' >&2
        return 1
    fi
    for number in "${!ethercat_interfaces[@]}"; do
        if [[ ! -r "/sys/class/net/${ethercat_interfaces[number]}/address" ]]; then
            if (( number == 0 )); then
                printf '未检测到主站 0 网卡 %s。\n' "${ethercat_interfaces[number]}" >&2
                return 1
            fi
            printf '未检测到第二张网卡 %s，暂不配置主站 1。\n' "${ethercat_interfaces[number]}"
            break
        fi
        addresses[number]="$(< "/sys/class/net/${ethercat_interfaces[number]}/address")"
        configured=""
        while IFS= read -r line; do
            if [[ "$line" =~ ^[[:space:]]*MASTER${number}_DEVICE[[:space:]]*= ]]; then
                configured="${line#*=}"
                configured="${configured%%#*}"
                configured="$(printf '%s' "$configured" | tr -d "[:space:]\"'")"
            fi
        done < "$config"
        configured_devices[number]="$configured"
    done
    if (( ${#addresses[@]} == 2 )) &&
       [[ "${configured_devices[0],,}" == "${addresses[1],,}" &&
          "${configured_devices[1],,}" == "${addresses[0],,}" ]]; then
        swapped_bindings=true
    fi
    for number in "${!addresses[@]}"; do
        configured="${configured_devices[number]}"
        if [[ -n "$configured" && "${configured,,}" != "${addresses[number],,}" &&
              "$swapped_bindings" != true ]]; then
            printf 'MASTER%s_DEVICE 已绑定 %s，与 %s 的 MAC %s 不一致；未覆盖现有配置。\n' \
                "$number" "$configured" "${ethercat_interfaces[number]}" "${addresses[number]}" >&2
            return 1
        fi
        if [[ "${configured,,}" != "${addresses[number],,}" ]]; then needs_configuration=true; fi
    done
    while IFS= read -r line; do
        if [[ "$line" =~ ^[[:space:]]*DEVICE_MODULES[[:space:]]*= ]]; then
            configured_modules="${line#*=}"
            configured_modules="${configured_modules%%#*}"
            configured_modules="$(printf '%s' "$configured_modules" | tr -d "\"'")"
        fi
    done < "$config"
    if [[ -z "${configured_modules//[[:space:]]/}" ]]; then
        if [[ ! -d /sys/module/ec_generic ]] && ! modinfo ec_generic >/dev/null 2>&1; then
            printf '%s\n' '未找到 ec_generic 内核模块；请先安装与当前内核匹配的 EtherLab generic 驱动。' >&2
            return 1
        fi
        needs_configuration=true
    elif [[ "$configured_modules" == *generic* && ! -d /sys/module/ec_generic ]] &&
         ! modinfo ec_generic >/dev/null 2>&1; then
        printf '%s\n' 'DEVICE_MODULES 包含 generic，但当前内核没有 ec_generic 模块；未重启 EtherLab 服务。' >&2
        return 1
    fi

    if [[ -r /sys/module/ec_master/parameters/main_devices ]]; then
        IFS=, read -r -a active_devices < /sys/module/ec_master/parameters/main_devices
    fi
    for number in "${!addresses[@]}"; do
        device="/dev/EtherCAT${number}"
        active_mac="${active_devices[number]-}"
        active_mac="${active_mac//[[:space:]]/}"
        if [[ ! -c "$device" || "${active_mac,,}" != "${addresses[number],,}" ]]; then
            needs_restart=true
        fi
    done
    if [[ "$needs_configuration" == true ]]; then needs_restart=true; fi
    if [[ "$configured_modules" == *generic* && -d /sys/module/ec_master &&
          ! -d /sys/module/ec_generic ]]; then needs_restart=true; fi

    if [[ "$needs_restart" == true && -c /dev/EtherCAT0 ]]; then
        local open_devices=()
        for device in /dev/EtherCAT*; do
            if [[ -c "$device" ]]; then open_devices+=("$device"); fi
        done
        if ! command -v fuser >/dev/null; then
            printf '缺少 fuser，无法安全确认 EtherLab 主站是否被占用；未重启服务。\n' >&2
            return 1
        fi
        if (( EUID != 0 )); then sudo -v; fi
        if (( EUID == 0 )); then
            if fuser -s "${open_devices[@]}"; then master_in_use=true; fi
        elif sudo fuser -s "${open_devices[@]}"; then
            master_in_use=true
        fi
        if [[ "$master_in_use" == true ]]; then
            printf 'EtherLab 主站正在被占用；请先失能并退出控制程序，未自动重启服务。\n' >&2
            if (( EUID == 0 )); then
                fuser -v "${open_devices[@]}" >&2 || true
            else
                sudo fuser -v "${open_devices[@]}" >&2 || true
            fi
            return 1
        fi
        printf '%s\n' 'EtherLab 主站配置需要更新；确认无进程占用后，正在停止旧服务。'
        if (( EUID == 0 )); then
            /etc/init.d/ethercat stop
        else
            sudo /etc/init.d/ethercat stop
        fi
        if [[ -c /dev/EtherCAT0 ]]; then
            printf '%s\n' 'EtherLab 服务停止后主站设备仍存在；未修改绑定。' >&2
            return 1
        fi
    fi

    for number in "${!addresses[@]}"; do
        if [[ "${configured_devices[number],,}" != "${addresses[number],,}" ]]; then
            if (( EUID == 0 )); then
                printf '\nMASTER%s_DEVICE="%s"\n' "$number" "${addresses[number]}" >> "$config"
            else
                printf '\nMASTER%s_DEVICE="%s"\n' "$number" "${addresses[number]}" | sudo tee -a "$config" >/dev/null
            fi
            printf '已在 %s 配置主站 %s: %s (%s)\n' \
                "$config" "$number" "${ethercat_interfaces[number]}" "${addresses[number]}"
        fi
    done
    if [[ -z "${configured_modules//[[:space:]]/}" ]]; then
        if (( EUID == 0 )); then
            printf '\nDEVICE_MODULES="generic"\n' >> "$config"
        else
            printf '\nDEVICE_MODULES="generic"\n' | sudo tee -a "$config" >/dev/null
        fi
        printf '已在 %s 配置 DEVICE_MODULES="generic"\n' "$config"
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
    configure_etherlab_masters
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
    IFS=, read -r -a active_macs < /sys/module/ec_master/parameters/main_devices
    ethercat_cli="${ETHERLAB_ROOT:-/usr/local/etherlab}/bin/ethercat"
    for number in "${!ethercat_interfaces[@]}"; do
        interface="${ethercat_interfaces[number]}"
        if [[ ! -r "/sys/class/net/$interface/address" ]]; then
            break
        fi
        expected_mac="$(< "/sys/class/net/$interface/address")"
        active_mac="${active_macs[number]-}"
        active_mac="${active_mac//[[:space:]]/}"
        if [[ ! -c "/dev/EtherCAT${number}" || "${active_mac,,}" != "${expected_mac,,}" ]]; then
            printf '主站 %s 绑定不正确：实际=%s 期望=%s。\n' "$number" "$active_mac" "$expected_mac" >&2
            exit 1
        fi
        if [[ ! -x "$ethercat_cli" ]]; then
            printf '未找到 %s，无法确认主站 %s 的网卡附着状态。\n' "$ethercat_cli" "$number" >&2
            exit 1
        fi
        if (( EUID == 0 )); then
            master_status="$("$ethercat_cli" -m "$number" master)"
        else
            master_status="$(sudo "$ethercat_cli" -m "$number" master)"
        fi
        if ! grep -Eq '^[[:space:]]+Main: .*\(attached\)' <<< "$master_status"; then
            printf '主站 %s 已配置但网卡 %s 未附着；检查 DEVICE_MODULES 和 ec_generic 模块（当前主站仍在等待设备）。\n' \
                "$number" "$interface" >&2
            exit 1
        fi
        printf 'EtherLab 内核主站已就绪: /dev/EtherCAT%s (%s)\n' "$number" "$interface"
    done
fi

trap - EXIT
printf '%s\n\n' '==================== EtherLab 启动检查与编译：完成 ===================='

if (( EUID == 0 )) || [[ "$require_etherlab" == OFF ]]; then
    exec ./build/internal/motor_console "$@"
fi
exec sudo ./build/internal/motor_console "$@"

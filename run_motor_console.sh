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

configure_etherlab_masters() {
    local config=/etc/sysconfig/ethercat
    local interfaces=(enp86s0 enx6c1ff7c503f3)
    local addresses=()
    local configured_devices=()
    local active_devices=()
    local line
    local configured
    local device
    local number
    local active_mac
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
    for number in "${!interfaces[@]}"; do
        if [[ ! -r "/sys/class/net/${interfaces[number]}/address" ]]; then
            if (( number == 0 )); then
                printf '未检测到主站 0 网卡 %s。\n' "${interfaces[number]}" >&2
                return 1
            fi
            printf '未检测到第二张网卡 %s，暂不配置主站 1。\n' "${interfaces[number]}"
            break
        fi
        addresses[number]="$(< "/sys/class/net/${interfaces[number]}/address")"
        configured=""
        while IFS= read -r line; do
            if [[ "$line" =~ ^[[:space:]]*MASTER${number}_DEVICE[[:space:]]*= ]]; then
                configured="${line#*=}"
                configured="${configured%%#*}"
                configured="$(printf '%s' "$configured" | tr -d "[:space:]\"'")"
            fi
        done < "$config"
        if [[ -n "$configured" && "${configured,,}" != "${addresses[number],,}" ]]; then
            printf 'MASTER%s_DEVICE 已绑定 %s，与 %s 的 MAC %s 不一致；未覆盖现有配置。\n' \
                "$number" "$configured" "${interfaces[number]}" "${addresses[number]}" >&2
            return 1
        fi
        configured_devices[number]="$configured"
        if [[ -z "$configured" ]]; then needs_configuration=true; fi
    done

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
        if [[ -z "${configured_devices[number]}" ]]; then
            if (( EUID == 0 )); then
                printf '\nMASTER%s_DEVICE="%s"\n' "$number" "${addresses[number]}" >> "$config"
            else
                printf '\nMASTER%s_DEVICE="%s"\n' "$number" "${addresses[number]}" | sudo tee -a "$config" >/dev/null
            fi
            printf '已在 %s 配置主站 %s: %s (%s)\n' \
                "$config" "$number" "${interfaces[number]}" "${addresses[number]}"
        fi
    done
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
    IFS=, read -r main_mac backup_mac < /sys/module/ec_master/parameters/main_devices
    main_mac="${main_mac//[[:space:]]/}"
    backup_mac="${backup_mac//[[:space:]]/}"
    expected_mac="$(< /sys/class/net/enp86s0/address)"
    if [[ "${main_mac,,}" != "${expected_mac,,}" ]]; then
        printf '主站 0 绑定不正确：实际=%s 期望=%s。\n' "$main_mac" "$expected_mac" >&2
        exit 1
    fi
    printf '%s\n' 'EtherLab 内核主站已就绪: /dev/EtherCAT0 (enp86s0)'
    if [[ -r /sys/class/net/enx6c1ff7c503f3/address ]]; then
        expected_mac="$(< /sys/class/net/enx6c1ff7c503f3/address)"
        if [[ ! -c /dev/EtherCAT1 || "${backup_mac,,}" != "${expected_mac,,}" ]]; then
            printf '主站 1 绑定不正确：实际=%s 期望=%s。\n' "$backup_mac" "$expected_mac" >&2
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

#!/bin/sh
set -eu

DMC_PATH="/sys/class/devfreq/dmc"
INSTALL_PATH="/usr/local/sbin/rk3588-performance-mode"
SERVICE_PATH="/etc/systemd/system/rk3588-performance-mode.service"

require_root() {
    if [ "$(id -u)" -ne 0 ]; then
        echo "This action needs root. Run: sudo $0 $1" >&2
        exit 1
    fi
}

dmc_frequency() {
    if [ ! -r "$DMC_PATH/available_frequencies" ]; then
        echo "RK3588 DMC devfreq node not found: $DMC_PATH" >&2
        exit 1
    fi
    tr ' ' '\n' < "$DMC_PATH/available_frequencies" | sort -n | tail -n 1
}

apply_mode() {
    require_root apply
    max_frequency="$(dmc_frequency)"
    # Set max before min so the kernel always sees a valid min/max range.
    echo "$max_frequency" > "$DMC_PATH/max_freq"
    echo "$max_frequency" > "$DMC_PATH/min_freq"
    echo performance > "$DMC_PATH/governor"
    echo "DDR locked at $((max_frequency / 1000000)) MHz"
}

show_status() {
    if [ ! -d "$DMC_PATH" ]; then
        echo "RK3588 DMC devfreq node not found: $DMC_PATH" >&2
        exit 1
    fi
    echo "DDR governor: $(cat "$DMC_PATH/governor")"
    echo "DDR current:  $(( $(cat "$DMC_PATH/cur_freq") / 1000000 )) MHz"
    echo "DDR min:      $(( $(cat "$DMC_PATH/min_freq") / 1000000 )) MHz"
    echo "DDR max:      $(( $(cat "$DMC_PATH/max_freq") / 1000000 )) MHz"
}

install_mode() {
    require_root install
    install -m 0755 "$0" "$INSTALL_PATH"
    printf '%s\n' \
        '[Unit]' \
        'Description=RK3588 latency performance mode (DDR maximum frequency)' \
        'After=multi-user.target' \
        '' \
        '[Service]' \
        'Type=oneshot' \
        "ExecStart=$INSTALL_PATH apply" \
        'RemainAfterExit=yes' \
        '' \
        '[Install]' \
        'WantedBy=multi-user.target' > "$SERVICE_PATH"
    systemctl daemon-reload
    systemctl enable --now rk3588-performance-mode.service
    show_status
}

uninstall_mode() {
    require_root uninstall
    systemctl disable --now rk3588-performance-mode.service 2>/dev/null || true
    rm -f "$SERVICE_PATH" "$INSTALL_PATH"
    systemctl daemon-reload
    min_frequency="$(tr ' ' '\n' < "$DMC_PATH/available_frequencies" | sort -n | head -n 1)"
    max_frequency="$(dmc_frequency)"
    echo "$min_frequency" > "$DMC_PATH/min_freq"
    echo "$max_frequency" > "$DMC_PATH/max_freq"
    echo dmc_ondemand > "$DMC_PATH/governor"
    show_status
}

case "${1:-status}" in
    apply)
        apply_mode
        ;;
    status)
        show_status
        ;;
    install)
        install_mode
        ;;
    uninstall)
        uninstall_mode
        ;;
    *)
        echo "Usage: $0 {status|apply|install|uninstall}" >&2
        exit 2
        ;;
esac

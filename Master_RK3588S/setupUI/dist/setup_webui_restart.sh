#!/bin/bash
set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
LOCAL_NO_PROXY="localhost,127.0.0.1,::1"
export no_proxy="${no_proxy:+$no_proxy,}$LOCAL_NO_PROXY"
export NO_PROXY="${NO_PROXY:+$NO_PROXY,}$LOCAL_NO_PROXY"
BRIDGE_SCRIPT="$SCRIPT_DIR/../ar_udp_control_bridge.py"
BRIDGE_LOG="$SCRIPT_DIR/../ar_udp_control_bridge.log"
BRIDGE_PID_FILE="/tmp/ar_udp_control_bridge.pid"

start_udp_control_bridge() {
    if [ "${AR_UDP_CONTROL_BRIDGE_AUTOSTART:-1}" = "0" ]; then
        return
    fi
    if [ ! -f "$BRIDGE_SCRIPT" ]; then
        echo "UDP/control bridge script not found: $BRIDGE_SCRIPT"
        return
    fi
    if [ -f "$BRIDGE_PID_FILE" ]; then
        old_pid="$(cat "$BRIDGE_PID_FILE" 2>/dev/null || true)"
        if [ -n "$old_pid" ] && [ -r "/proc/$old_pid/cmdline" ] && grep -qa "ar_udp_control_bridge.py" "/proc/$old_pid/cmdline"; then
            echo "UDP/control bridge already running: pid=$old_pid"
            return
        fi
    fi
    setsid python3 "$BRIDGE_SCRIPT" >> "$BRIDGE_LOG" 2>&1 < /dev/null &
    echo "$!" > "$BRIDGE_PID_FILE"
    echo "UDP/control bridge started: pid=$!"
}

start_udp_control_bridge
sleep 2
kill -9 4744 2>/dev/null
sleep 1
exec "$SCRIPT_DIR/setup_webui"

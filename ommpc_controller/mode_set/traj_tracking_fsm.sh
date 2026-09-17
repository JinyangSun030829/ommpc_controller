#!/bin/bash

CONTROLLER_NODE="${CONTROLLER_NODE:-/traj_tracking_controller}"
MAVROS_NAMESPACE="${MAVROS_NAMESPACE:-/uav1/mavros}"
SET_MODE_SERVICE="${MAVROS_NAMESPACE}/set_mode"

set_param()
{
    local param_name="$1"
    local param_value="$2"
    rosrun dynamic_reconfigure dynparam set \
        "$CONTROLLER_NODE" "$param_name" "$param_value"
}

set_offboard()
{
    echo "[INFO] Waiting for ${SET_MODE_SERVICE} ..."
    local timeout=10
    local elapsed=0
    while ! rosservice list 2>/dev/null | grep -Fxq "$SET_MODE_SERVICE"; do
        if [ "$elapsed" -ge "$timeout" ]; then
            echo "[ERROR] Timed out waiting for ${SET_MODE_SERVICE}."
            return 1
        fi
        sleep 1
        elapsed=$((elapsed + 1))
    done

    local response
    response=$(rosservice call "$SET_MODE_SERVICE" \
        "base_mode: 0
custom_mode: 'OFFBOARD'" 2>&1)
    local result=$?
    echo "$response"
    if [ "$result" -ne 0 ] ||
       ! echo "$response" | grep -Eiq "mode_sent:[[:space:]]*(true|True|1)"; then
        echo "[ERROR] PX4 did not accept OFFBOARD."
        return 1
    fi
}

pulse()
{
    local parameter="$1"
    set_param "$parameter" false || return 1
    set_param "$parameter" true  || return 1
    set_param "$parameter" false || return 1
}

case "$1" in
    offboard)
        set_offboard || exit 1
        ;;
    takeoff)
        echo "[INFO] Switching to OFFBOARD and triggering TAKEOFF."
        set_param land_enabled false || exit 1
        set_param command_or_hover false || exit 1
        set_offboard || exit 1
        sleep 1
        pulse takeoff_enabled || exit 1
        ;;
    command)
        echo "[INFO] Enabling polynomial trajectory execution."
        set_param command_or_hover true || exit 1
        ;;
    hover)
        echo "[INFO] Switching to HOVER."
        set_param command_or_hover false || exit 1
        ;;
    land)
        echo "[INFO] Requesting BRAKE -> stable hover -> LAND (single update)."
        # Reset only the trigger first; do NOT cancel COMMAND in a separate call.
        set_param land_enabled false || exit 1
        rosrun dynamic_reconfigure dynparam set "$CONTROLLER_NODE" \
            '{land_enabled: true, takeoff_enabled: false, command_or_hover: false}' || exit 1
        set_param land_enabled false || exit 1
        ;;
    *)
        echo "Usage: $0 {offboard|takeoff|command|hover|land}"
        echo "Optional environment: CONTROLLER_NODE, MAVROS_NAMESPACE"
        exit 1
        ;;
esac

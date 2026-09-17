#!/bin/bash

CONTROLLER_NODE="/ommpc_controller"
SET_MODE_SERVICE="/uav1/mavros/set_mode"

set_param()
{
    local param_name="$1"
    local param_value="$2"

    rosrun dynamic_reconfigure dynparam set \
        "$CONTROLLER_NODE" \
        "$param_name" \
        "$param_value"

    if [ $? -ne 0 ]; then
        echo "[ERROR] 设置参数失败：${param_name}=${param_value}"
        return 1
    fi
}

set_offboard()
{
    echo "[INFO] 等待 MAVROS 模式切换服务：$SET_MODE_SERVICE"

    local timeout=10
    local elapsed=0

    while ! rosservice list 2>/dev/null | grep -Fxq "$SET_MODE_SERVICE"; do
        if [ "$elapsed" -ge "$timeout" ]; then
            echo "[ERROR] 等待服务超时：$SET_MODE_SERVICE"
            echo "[INFO] 当前 MAVROS set_mode 服务："
            rosservice list 2>/dev/null | grep "mavros/set_mode"
            return 1
        fi

        sleep 1
        elapsed=$((elapsed + 1))
    done

    echo "[INFO] 找到服务：$SET_MODE_SERVICE"
    echo "[INFO] 请求切换到 OFFBOARD 模式..."

    local response
    response=$(rosservice call "$SET_MODE_SERVICE" \
        "base_mode: 0
custom_mode: 'OFFBOARD'" 2>&1)

    local result=$?
    echo "$response"

    if [ "$result" -ne 0 ]; then
        echo "[ERROR] 调用 OFFBOARD 模式服务失败"
        return 1
    fi

    if echo "$response" | grep -Eiq "mode_sent:[[:space:]]*(true|True|1)"; then
        echo "[INFO] PX4 已接受 OFFBOARD 模式切换请求"
        return 0
    fi

    echo "[ERROR] PX4 未接受 OFFBOARD 模式切换请求"
    return 1
}

trigger_takeoff()
{
    # 防止同时存在降落触发
    set_param land_enabled false || return 1

    # 产生 takeoff_enabled 的 false -> true 上升沿
    set_param takeoff_enabled false || return 1
    set_param takeoff_enabled true  || return 1
    set_param takeoff_enabled false || return 1

    echo "[INFO] 起飞触发命令已发送"
}

trigger_land()
{
    # 防止同时存在起飞触发
    set_param takeoff_enabled false || return 1

    # 产生 land_enabled 的 false -> true 上升沿
    set_param land_enabled false || return 1
    set_param land_enabled true  || return 1
    set_param land_enabled false || return 1

    echo "[INFO] 降落触发命令已发送"
}

case "$1" in
    takeoff)
        echo "[INFO] 执行 OFFBOARD + 起飞流程"

        # 先切换 OFFBOARD
        set_offboard || exit 1

        # 给状态消息和控制器少量时间更新
        sleep 1

        # 再触发控制器内部的起飞状态机
        trigger_takeoff || exit 1
        ;;

    offboard)
        set_offboard || exit 1
        ;;

    land)
        echo "[INFO] 触发降落"
        trigger_land || exit 1
        ;;

    command)
        echo "[INFO] 切换到轨迹指令模式"
        set_param command_or_hover true || exit 1
        ;;

    hover)
        echo "[INFO] 切换到悬停模式"
        set_param command_or_hover false || exit 1
        ;;

    *)
        echo "Usage:"
        echo "  $0 offboard    切换到 OFFBOARD 模式"
        echo "  $0 takeoff     切换 OFFBOARD 并触发起飞"
        echo "  $0 land        触发降落"
        echo "  $0 command     切换到轨迹指令模式"
        echo "  $0 hover       切换到悬停模式"
        exit 1
        ;;
esac
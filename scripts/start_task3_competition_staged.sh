#!/usr/bin/env bash
# Start Task 3 in dependency order inside a visible tmux session.
set -euo pipefail

workspace_path="${ROBOTX_WS:-/home/robotx/robotx_ws}"
session_name="${TASK3_TMUX_SESSION:-robotx_task3}"
fcu_url="serial:///dev/serial/by-id/usb-Holybro_Pixhawk6C_38002E001851333531333639-if02:57600"
pump_enabled=false

usage() {
    echo "Usage: $0 [--pump]"
    echo "  --pump  Start the verified automatic pump path as well."
}

if [[ $# -gt 1 || ${1:-} == "--help" || ${1:-} == "-h" ]]; then
    usage
    [[ ${1:-} == "--help" || ${1:-} == "-h" ]] && exit 0
    exit 2
fi
if [[ ${1:-} == "--pump" ]]; then
    pump_enabled=true
fi

[[ -f "$workspace_path/install/setup.bash" ]] || {
    echo "Workspace install is missing: $workspace_path" >&2
    exit 1
}
command -v tmux >/dev/null || {
    echo "tmux is required" >&2
    exit 1
}

source /opt/ros/humble/setup.bash
source "$workspace_path/install/setup.bash"

if tmux has-session -t "$session_name" 2>/dev/null; then
    echo "Task3 session already exists: tmux attach -t $session_name" >&2
    exit 1
fi

has_publisher() {
    timeout 5 ros2 topic info --no-daemon --spin-time 2 "$1" 2>/dev/null |
        grep -q 'Publisher count: [1-9]'
}

for existing_topic in /livox/lidar /mavros/state /fast_lio/cloud_registered_body; do
    if has_publisher "$existing_topic"; then
        echo "Existing publisher found on $existing_topic; stop the old stack first." >&2
        exit 1
    fi
done

wait_for_stream() {
    local topic="$1"
    local timeout_s="$2"
    local deadline=$((SECONDS + timeout_s))
    until timeout 5 ros2 topic echo "$topic" --once \
        --qos-reliability best_effort >/dev/null 2>&1; do
        if (( SECONDS >= deadline )); then
            echo "Timed out waiting for $topic. Inspect: tmux attach -t $session_name" >&2
            exit 1
        fi
        sleep 1
    done
}

open_window() {
    local name="$1"
    local command="$2"
    local quoted
    printf -v quoted '%q' "$command"
    tmux new-window -t "$session_name:" -n "$name" "bash -lc $quoted"
}

ros_setup="source /opt/ros/humble/setup.bash && source '$workspace_path/install/setup.bash'"

tmux new-session -d -s "$session_name" -n obstacle \
    "bash -lc $(printf '%q' "$ros_setup && exec '$HOME/start_obstacle_stack.sh' start")"
wait_for_stream /livox/lidar 30
wait_for_stream /mavros/obstacle/send 30

open_window mavros "$ros_setup && exec ros2 run mavros mavros_node --ros-args -p fcu_url:='$fcu_url'"
wait_for_stream /mavros/state 30

open_window fastlio "$ros_setup && exec ros2 launch fast_lio task3_mid360.launch.py"
wait_for_stream /fast_lio/cloud_registered_body 45

open_window task3 "$ros_setup && exec ros2 launch usv_task3_executor task3_competition_core.launch.py start_obstacle_stack:=false start_mavros:=false start_fast_lio:=false start_window_stack:=false start_bay_indicator:=false dry_run:=false control_enabled:=true enable_aux_output:=false"

open_window oak_gimbal "$ros_setup && exec ros2 launch robotx_gimbal_interfaces task3_full.launch.py start_camera:=true start_perception:=true start_aiming:=true start_gimbal:=true imu_feedforward:=true decode_task3_signal:=true start_pump:=$pump_enabled auto_pump:=$pump_enabled bench_docked:=false show_gui:=false use_web_view:=false"
wait_for_stream /oak/rgb/image_raw 45
wait_for_stream /gimbal/actual_angles_stamped 30

open_window bay_indicator "$ros_setup && exec ros2 launch robotx_usv_perception task3_dock_vm.launch.py image_topic:=/oak/rgb/image_raw output_topic:=/robotx/perception/leds show_gui:=false"
open_window vision_adapter "$ros_setup && exec ros2 run usv_task3_executor task3_vision_contract_adapter --ros-args -p fallback_window_id:=1 -p green_confirmation_s:=5.0"
open_window monitor "$ros_setup && while true; do clear; date -Ins; ros2 node list | grep -E 'mavros|fast_lio|task3|livox|gimbal|pump' || true; sleep 3; done"

tmux select-window -t "$session_name:task3"
echo "Task3 started in stages. Detach with Ctrl-b d; stop with: tmux kill-session -t $session_name"
exec tmux attach -t "$session_name"

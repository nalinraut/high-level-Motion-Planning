#!/bin/bash

kill_descendant_processes() {
    local pid="$1"
    local and_self="${2:-false}"
    if children="$(pgrep -P "$pid")"; then
        for child in $children; do
            kill_descendant_processes "$child" true
        done
    fi
    if [[ "$and_self" == true ]]; then
        kill -9 "$pid"
    fi
}

mkdir -p Logs
python Common/system_state_service.py > Logs/system_state_service.log &
./MotionServer_kinematic > Logs/MotionServer_kinematic.log &
./ControllerDispatcher > Logs/ControllerDispatcher.log  &


int_handler()
{
    echo "Interrupted."
    kill_descendant_processes $$
    exit 1
}
trap 'int_handler' INT

while true; do
    sleep 1
    #echo "I'm still alive!"
done

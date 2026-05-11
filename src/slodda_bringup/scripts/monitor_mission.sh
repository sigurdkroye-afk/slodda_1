#!/bin/bash
# monitor_mission.sh — tmux-monitor for Slodda 1 bamse-oppdrag
# 4 paner: arm/status | bear_mission/status | yolo hz | cmd_vel

SESSION=slodda_monitor

ROS_SETUP="source /opt/ros/kilted/setup.bash && source ~/slodda_1/install/setup.bash && export ROS_DOMAIN_ID=42"

if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "Session '$SESSION' eksisterer allerede — attacher."
    tmux attach -t "$SESSION"
    exit 0
fi

tmux new-session -d -s "$SESSION" -x 220 -y 50

# Pane 0 (øvre venstre): arm status
tmux send-keys -t "$SESSION:0.0" \
    "$ROS_SETUP && ros2 topic echo /arm/status" C-m

# Pane 1 (øvre høyre): mission status
tmux split-window -h -t "$SESSION:0.0"
tmux send-keys -t "$SESSION:0.1" \
    "$ROS_SETUP && ros2 topic echo /bear_mission/status" C-m

# Pane 2 (nedre venstre): YOLO frekvens
tmux split-window -v -t "$SESSION:0.0"
tmux send-keys -t "$SESSION:0.2" \
    "$ROS_SETUP && ros2 topic hz /yolo/detections" C-m

# Pane 3 (nedre høyre): cmd_vel live
tmux split-window -v -t "$SESSION:0.1"
tmux send-keys -t "$SESSION:0.3" \
    "$ROS_SETUP && while true; do ros2 topic echo /cmd_vel --once 2>/dev/null; sleep 0.2; done" C-m

tmux select-pane -t "$SESSION:0.0"
tmux attach -t "$SESSION"

#!/bin/bash
# Stationary FRI connection from robot-rt (docs/05 procedure, docs/06 host).
# Runs on robot-rt as ~/lbr_med_stack/overlays/launch_stationary.sh, a symlink to this tracked file.
#
#   launch_stationary.sh                 no recording (process log in overlays/logs/)
#   launch_stationary.sh --record LABEL  recording services on: LBR_RECORDING_ROOT is
#                                        ~/lbr_recordings/<date>-LABEL-<HHMMSS>, runs land in <root>/run-<id>,
#                                        the process log in <root>.driver.log. The workstation fetches them with
#                                        03_projects/tactile-cable-sysid/operations/robot-sysid/fetch_recordings.sh.
#
# Same launch either way: JTC, 1000 Hz controllers, default system config (tau 0.04); only the real-time
# threads are pinned to the reserved cores. Unlike the first untracked copy (8 Nov), it also unsets
# LBR_RECORDING_DURATION_SECONDS and LBR_RECORDING_CONFIG, as the workstation's live_env.sh did.
set -eo pipefail
usage() { echo "usage: launch_stationary.sh [--record LABEL]"; exit 2; }
label=""
if [ $# -ne 0 ]; then
  [ $# -eq 2 ] && [ "$1" = --record ] || usage
  label=$2
  [[ "$label" =~ ^[A-Za-z0-9_-]+$ ]] || { echo "LABEL may contain only letters, digits, _ and -"; exit 2; }
fi

lbr_ws=$HOME/lbr_med_stack
source /opt/ros/jazzy/setup.bash
source "$lbr_ws/install/local_setup.bash"
source "$lbr_ws/overlays/live-signals/install/local_setup.bash"
export ROS_DOMAIN_ID=85 ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET RCUTILS_COLORIZED_OUTPUT=0
unset ROS_LOCALHOST_ONLY LBR_RECORDING_DIRECTORY LBR_RECORDING_ROOT LBR_RECORDING_DURATION_SECONDS LBR_RECORDING_CONFIG

if ss -lun 'sport = :30200' | grep -q 30200; then
  echo "UDP 30200 is already owned by another client; stop it first."; exit 1
fi
ip -4 route get 192.170.10.2 | head -1
echo "lbr_fri_ros2 from: $(ros2 pkg prefix lbr_fri_ros2)"

read -r stamp_date stamp_time < <(TZ=Asia/Jerusalem date '+%F %H%M%S')
if [ -n "$label" ]; then
  mkdir -p "$HOME/lbr_recordings"
  export LBR_RECORDING_ROOT="$HOME/lbr_recordings/$stamp_date-$label-$stamp_time"
  [ ! -e "$LBR_RECORDING_ROOT" ] || { echo "$LBR_RECORDING_ROOT already exists"; exit 1; }
  echo "LBR_RECORDING_ROOT=$LBR_RECORDING_ROOT"
  log=$LBR_RECORDING_ROOT.driver.log
else
  log=$lbr_ws/overlays/logs/stationary-${stamp_date//-/}-$stamp_time.process.log
  mkdir -p "$(dirname "$log")"
fi
echo "log: $log"

# Pin only the real-time threads to reserved cores; everything else stays on CPUs 0-1,6-23.
# (Pinning the whole launch puts every thread on one core: isolcpus=domain disables balancing.)
(
  for _ in $(seq 1 60); do
    sleep 1
    pid=$(pgrep -o -f "controller_manager/ros2_control_node") || continue
    fri=$(ps -L -o tid=,rtprio= -p "$pid" | awk '$2==80{print $1; exit}')
    cm=$(ps -L -o tid=,rtprio= -p "$pid" | awk '$2==50{print $1; exit}')
    if [ -n "$fri" ] && [ -n "$cm" ]; then
      taskset -pc 2 "$fri" >/dev/null && taskset -pc 3 "$cm" >/dev/null
      echo "[pin] FRI thread $fri -> CPU 2, controller loop $cm -> CPU 3"
      exit 0
    fi
  done
  echo "[pin] WARNING: real-time threads not found; nothing pinned"
) &

ros2 launch lbr_bringup hardware.launch.py \
  model:=med7 \
  ctrl:=joint_trajectory_controller \
  ctrl_cfg:="$lbr_ws/src/lbr_med_examples/config/hardware_controllers_1000hz.yaml" \
  2>&1 | tee "$log"

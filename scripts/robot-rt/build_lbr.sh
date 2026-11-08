#!/bin/bash
# Build lbr_med_stack + recorder overlay on robot-rt against /opt/ros/jazzy only.
set -eo pipefail
ws=$HOME/lbr_med_stack
cd "$ws"
export FRI_CLIENT_VERSION=1.15
source /opt/ros/jazzy/setup.bash
touch overlays/COLCON_IGNORE   # keep MCAP sources and overlay builds out of the base build
echo "== patch"
patch=../../patches/fri-recording/lbr-fri-ros2-recording.patch
if git -C src/lbr_fri_ros2_stack apply -R --check "$patch" 2>/dev/null; then
  echo "already applied"
else
  git -C src/lbr_fri_ros2_stack apply --check "$patch"
  git -C src/lbr_fri_ros2_stack apply "$patch"
fi
echo "== rosdep"
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy 2>&1 | tail -3
echo "== base build"
colcon build --symlink-install > overlays/base-build.log 2>&1 || { tail -40 overlays/base-build.log; exit 1; }
grep -E "^(Finished|Failed|Summary)" overlays/base-build.log | tail -20
source install/local_setup.bash
ov=$ws/overlays/live-signals
mkdir -p "$ov"
mcap=$(ls -d "$ws"/overlays/deps/mcap-*)
echo "== recorder overlay"
colcon --log-base "$ov/colcon-log" build --packages-select lbr_recording_interfaces lbr_fri_ros2 lbr_ros2_control \
  --allow-overriding lbr_fri_ros2 lbr_ros2_control --build-base "$ov/build" --install-base "$ov/install" \
  --cmake-args -DBUILD_TESTING=ON -DLBR_BUILD_RECORDING=ON -DLBR_ACQUISITION_SOURCE_DIR="$ws/src/lbr_acquisition" \
  -DMCAP_SOURCE_DIR="$mcap" > "$ov/build.log" 2>&1 || { tail -40 "$ov/build.log"; exit 1; }
grep -E "^(Finished|Failed|Summary)" "$ov/build.log"
{ echo "built $(date '+%F %T')"; echo "lbr-med-stack HEAD $(git rev-parse HEAD)"; echo "driver diff sha256 $(git -C src/lbr_fri_ros2_stack diff | sha256sum | cut -d' ' -f1)"; } > "$ov/source.txt"
source "$ov/install/local_setup.bash"
echo "== offline recorder test"
ctest --test-dir "$ov/build/lbr_fri_ros2" -R '^fri_driver_hooks$' --output-on-failure 2>&1 | tail -4
echo "== DONE"

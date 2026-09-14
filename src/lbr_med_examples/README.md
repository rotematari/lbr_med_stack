# lbr_med_examples

First-party `ament_python` package for the lbr_med overlay. Ships one example: `goto_pose`, a thin
MoveItPy driver that plans and executes a single Cartesian go-to-pose on the med7 arm. It is a
CLI-to-MoveItPy bridge, no wrapper layer.

## Prerequisites

Build the repository as one overlay workspace. The pinned upstream packages and this first-party
package are installed together, so every shell needs the ROS underlay and this checkout's overlay:

```bash
source /opt/ros/jazzy/setup.bash
source /path/to/lbr_med_stack/install/setup.bash
```

## Run via launch (recommended)

Always exercise this on **mock hardware** before the real robot. Three sourced shells:

**Shell 1 - mock hardware:**

```bash
ros2 launch lbr_bringup mock.launch.py model:=med7
```

**Shell 2 - MoveIt move_group (mock mode):**

```bash
ros2 launch lbr_bringup move_group.launch.py model:=med7 mode:=mock rviz:=true
```

**Shell 3 - the go-to-pose node**, against a conservative, reachable pose:

```bash
ros2 launch lbr_med_examples goto_pose.launch.py x:=0.4 y:=0.0 z:=0.6
```

The launch file injects the med7 MoveIt config (robot_description / SRDF / kinematics / OMPL) into the
node as per-node parameters and runs it under the `/lbr` namespace, matching move_group. This is the
fix for the RDFLoader-timeout failure. Confirm the planned trajectory in RViz before trusting it on
real hardware.

`x/y/z` default to `0.4 / 0.0 / 0.6` (a safe mid-workspace mock target, well inside the 800 mm reach),
so a bare `ros2 launch lbr_med_examples goto_pose.launch.py` runs the known-good target.

### Jazzy 2.12.4 teardown limitation

On Ubuntu's `ros-jazzy-moveit-py` 2.12.4, MoveItPy can segfault while destroying `MoveItCpp` after
a successful plan and execution. The upstream issue is
[`moveit2#3721`](https://github.com/moveit/moveit2/issues/3721); it reproduces even when
`MoveItPy.shutdown()` is called explicitly. Treat `Completed trajectory execution with status
SUCCEEDED` as evidence for the mock plan/execution gate, and record the final process status
separately until the upstream package fixes teardown. Do not hide the crash with a forced process
exit, because that would mask other cleanup failures.

## Legacy / broken invocation (do not use)

```bash
python3 goto_pose.py --x 0.4 --y 0.0 --z 0.6   # BROKEN
```

A bare `python3 goto_pose.py` gives MoveItPy no injected robot_description / SRDF and runs in the root
namespace instead of `/lbr`, so `RDFLoader` times out after 10 s. It is retained only to document the
diagnosed failure; use the launch file above.

## CLI

The node's own arguments (overridable escape hatches; the launch file passes `--x/--y/--z`):

| Arg | Default | Meaning |
|---|---|---|
| `--x --y --z` | required | target position (m) in `--frame` |
| `--qx --qy --qz --qw` | `0 0 0 1` | target orientation quaternion (identity) |
| `--group` | `arm` | MoveIt planning group (SRDF `<group name="arm">`) |
| `--frame` | `lbr_link_0` | planning / reference frame (SRDF chain base_link) |
| `--robot-namespace` | `lbr` | lbr_bringup namespace (topic/TF prefix) |
| `--tip-link` | `lbr_link_ee` | end-effector link the goal pose is solved for (SRDF chain tip_link) |

The `--group / --frame / --tip-link / --robot-namespace` defaults are confirmed against the med7 SRDF
(`med7_moveit_config/config/med7.srdf`, group `arm` with `base_link=lbr_link_0`, `tip_link=lbr_link_ee`)
and the `lbr_bringup` mixin namespace `lbr`. The tf prefix is `<robot_name>_link` (underscore).

```bash
ros2 run lbr_med_examples goto_pose --help
```

## Example target pose

`(0.4, 0.0, 0.6)` m in the planning frame with identity orientation: a mid-workspace pose well inside
the 800 mm reach, chosen as a safe first mock target. Pick a pose appropriate to your cell for real
runs.

## Scope

POSITION-mode plan + execute only. Impedance / torque streaming (`JOINT_IMPEDANCE` FRI mode) is
explicit **future scope** and is not implemented here.

## [VERIFY ON FIRST RUN]

- Real-hardware controller names, before the first non-mock run.
- launch args (`model:=med7`, `mode:=mock`, `rviz:=true` on the upstream files): confirm with
  `ros2 launch lbr_bringup move_group.launch.py -s`.

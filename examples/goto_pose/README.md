# goto_pose

A thin MoveItPy driver that plans and executes a single Cartesian go-to-pose on the med7 arm. It is the
only custom Python in the overlay: a CLI-to-MoveItPy bridge, no wrapper layer.

## Mock-first usage

Always exercise this on **mock hardware** before the real robot. From a built, sourced overlay
(see [`../../docs/01_fresh_machine_setup.md`](../../docs/01_fresh_machine_setup.md)):

**Shell 1 - bring up mock MoveIt:**

```bash
ros2 launch lbr_bringup move_group.launch.py model:=med7 mode:=mock rviz:=true
```

**Shell 2 - run the script** against a conservative, reachable pose:

```bash
python3 goto_pose.py --x 0.4 --y 0.0 --z 0.6
```

Orientation defaults to the identity quaternion; override with `--qx --qy --qz --qw`. Confirm the planned
trajectory in RViz before trusting it on real hardware.

### Example target pose

The values above place the tip at `(0.4, 0.0, 0.6)` m in the planning frame with identity orientation: a
mid-workspace, well inside the 800 mm reach, chosen as a safe first mock target. Pick a pose appropriate to
your cell for real runs.

## CLI

| Arg | Default | Meaning |
|---|---|---|
| `--x --y --z` | required | target position (m) in `--frame` |
| `--qx --qy --qz --qw` | `0 0 0 1` | target orientation quaternion (identity) |
| `--group` | `arm` | MoveIt planning group |
| `--frame` | `lbr/link_0` | planning / reference frame |
| `--robot-namespace` | `lbr` | lbr_bringup namespace (topic/TF prefix) |
| `--tip-link` | `lbr/link_ee` | end-effector link the goal pose is solved for |

The script exits nonzero if planning or execution fails.

## [VERIFY ON FIRST RUN]

These defaults follow lbr-stack's documented med7 flow and are not yet confirmed against the pinned
`jazzy-v2.4.3` tag. Verify each before relying on a real-robot run:

- **`--group`** (`arm`): the med7 MoveIt config planning group name.
  Check with `ros2 param get /move_group robot_description_semantic` or inspect the shipped SRDF.
- **`--frame`** (`lbr/link_0`): the planning/base frame. Confirm with `ros2 run tf2_tools view_frames`.
- **`--tip-link`** (`lbr/link_ee`): the chain tip link. Confirm from the same SRDF / TF tree.
- **`--robot-namespace`** (`lbr`): the namespace `lbr_bringup` applies. Confirm with `ros2 topic list`
  (the FRI/state topics live under `/lbr/...`).
- **MoveItPy availability**: `python3 -c "from moveit.planning import MoveItPy"` must import under
  `ros-jazzy-moveit`.
- **launch args** (`model:=med7`, `mode:=mock`, `rviz:=true`): confirm with
  `ros2 launch lbr_bringup move_group.launch.py -s`.

## Scope

POSITION-mode plan + execute only. Impedance / torque streaming (`JOINT_IMPEDANCE` FRI mode) is explicit
**future scope** and is not implemented here.

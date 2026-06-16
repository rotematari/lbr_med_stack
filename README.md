# lbr_med_stack

ROS 2 **Jazzy** workspace overlay for the lab's **KUKA LBR Med 7 R800**, built on the community
[lbr-stack](https://github.com/lbr-stack/lbr_fri_ros2_stack) and driving the arm over **FRI 1.15**
(to match Sunrise.OS Med 1.15 on the cabinet).

This repo is deliberately thin. It vendors `lbr_fri_ros2_stack` at a pinned tag via a `vcs` manifest,
reuses that stack's launch files and stock MoveIt config, and ships only:

- a pinned overlay manifest (`lbr_med_stack.repos`),
- the four setup docs in [`docs/`](docs/),
- one `MoveItPy` go-to-pose example ([`examples/goto_pose/`](examples/goto_pose/)).

No custom controllers, no MoveIt fork, no wrapper layer. Impedance / torque streaming is documented future scope.

## What you need

- Ubuntu 24.04 (Noble) with **ROS 2 Jazzy** (`/opt/ros/jazzy`).
- `ros-jazzy-moveit`, `ros-jazzy-ros2-control`, `ros-jazzy-ros2-controllers` (mock hardware ships here).
- Build tooling: `python3-colcon-common-extensions`, `python3-vcstool`, `python3-rosdep`.

Full first-time setup is in [`docs/01_fresh_machine_setup.md`](docs/01_fresh_machine_setup.md).

## Quickstart (mock, no robot)

From a clean overlay workspace:

```bash
mkdir -p ~/lbr_med_ws/src && cd ~/lbr_med_ws
export FRI_CLIENT_VERSION=1.15

# 1. pull the pinned stack, then the FRI SDK manifest the stack ships
vcs import src < /path/to/lbr_med_stack/lbr_med_stack.repos
vcs import src < src/lbr_fri_ros2_stack/lbr_fri_ros2_stack/repos-fri-1.15.yaml

# 2. resolve dependencies and build
rosdep install --from-paths src -i -r -y
colcon build --symlink-install

# 3. source the overlay
source install/setup.bash

# 4. bring up mock hardware + MoveIt (no robot)
ros2 launch lbr_bringup mock.launch.py model:=med7
# in another sourced shell:
ros2 launch lbr_bringup move_group.launch.py model:=med7 mode:=mock rviz:=true
```

> [VERIFY ON FIRST RUN] The exact `lbr_bringup` launch-file names and arguments (`mock.launch.py`,
> `move_group.launch.py`, `model:=med7`, `mode:=mock`) are taken from lbr-stack's documented flow.
> Confirm against the pinned tag with `ros2 launch lbr_bringup <file> -s` before relying on them.

Run the go-to-pose example: see [`examples/goto_pose/README.md`](examples/goto_pose/README.md).

For the real robot, do **not** start here. Work through the network, Sunrise-side, and bringup docs in order:
[`docs/02_network_setup.md`](docs/02_network_setup.md) ->
[`docs/03_sunrise_side_setup.md`](docs/03_sunrise_side_setup.md) ->
[`docs/04_hardware_bringup_checklist.md`](docs/04_hardware_bringup_checklist.md).

## Repo layout

```
lbr_med_stack/
├── README.md                       # this file
├── LICENSE                         # Apache-2.0
├── lbr_med_stack.repos             # pinned vcs manifest (lbr_fri_ros2_stack@jazzy-v2.4.3)
├── .gitignore                      # src/ build/ install/ log/ (vcs sources never committed)
├── docs/
│   ├── 01_fresh_machine_setup.md   # ROS 2 Jazzy + overlay from scratch
│   ├── 02_network_setup.md         # FRI subnet, netplan, connectivity checks
│   ├── 03_sunrise_side_setup.md    # cabinet-side Sunrise / LBRServer setup
│   └── 04_hardware_bringup_checklist.md  # phased mock -> real bringup
└── examples/
    └── goto_pose/
        ├── goto_pose.py            # thin MoveItPy plan+execute to a CLI pose
        └── README.md               # mock-first usage
```

`src/`, `build/`, `install/`, `log/` are git-ignored. The vendored sources come from `vcs import`, not from this repo.

## Versioning

`lbr_med_stack.repos` pins `lbr_fri_ros2_stack` to a **tag** (`jazzy-v2.4.3`), not the moving `jazzy` branch,
for reproducible builds. The FRI SDK (`lbr-stack/fri`) is pulled by the stack's own `repos-fri-1.15.yaml`,
imported as the second `vcs import` step. Do not hand-pin the FRI tag; take it from that manifest.

### Bumping the pinned tag

1. Pick the new tag from the [lbr_fri_ros2_stack releases](https://github.com/lbr-stack/lbr_fri_ros2_stack/releases).
   Confirm it targets ROS 2 Jazzy and check its `CHANGELOG` for breaking launch-file or config changes.
2. Edit the `version:` field in `lbr_med_stack.repos` to the new tag.
3. Re-run the quickstart in a **clean** `~/lbr_med_ws` (do not reuse a stale `src/`).
4. Re-verify the mock bringup gate (Phase 0 in `docs/04_hardware_bringup_checklist.md`) before touching the real robot.
5. If the FRI client version changes with the new release, update `FRI_CLIENT_VERSION` and the Sunrise.OS pairing in the docs.

## License

Apache-2.0 (see [`LICENSE`](LICENSE)), matching the upstream lbr-stack license. Copyright 2026 Rotem Atari.

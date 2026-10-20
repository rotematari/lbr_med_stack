# lbr_med_stack

ROS 2 **Jazzy** workspace overlay for the lab's **KUKA LBR Med 7 R800**, built on the community
[lbr-stack](https://github.com/lbr-stack/lbr_fri_ros2_stack) and driving the arm over **FRI 1.15**
(to match Sunrise.OS Med 1.15 on the cabinet).

This repo is deliberately thin. It vendors `lbr_fri_ros2_stack` at a pinned tag via a `vcs` manifest,
reuses that stack's launch files and stock MoveIt config, and ships only:

- a pinned overlay manifest (`lbr_med_stack.repos`),
- the four setup docs in [`docs/`](docs/),
- one first-party `ament_python` package with a `MoveItPy` go-to-pose example and launch file
  ([`src/lbr_med_examples/`](src/lbr_med_examples/)).

No custom controllers, no MoveIt fork, no wrapper layer. Impedance / torque streaming is documented future scope.

## What you need

- Ubuntu 24.04 (Noble) with **ROS 2 Jazzy** (`/opt/ros/jazzy`).
- `ros-jazzy-moveit`, `ros-jazzy-moveit-servo`, `ros-jazzy-ros2-control`, and
  `ros-jazzy-ros2-controllers` (mock hardware ships here).
- Build tooling: `python3-colcon-common-extensions`, `python3-vcstool`, `python3-rosdep`.

Full first-time setup is in [`docs/01_fresh_machine_setup.md`](docs/01_fresh_machine_setup.md).

## Quickstart (mock, no robot)

The repository is the overlay workspace; the committed example package and pinned
upstream sources are built together:

```bash
git clone https://github.com/rotematari/lbr_med_stack.git
cd lbr_med_stack
source /opt/ros/jazzy/setup.bash
export FRI_CLIENT_VERSION=1.15

# 1. pull the stack, FRI SDK, and IDL at their exact pinned revisions
vcs import src < lbr_med_stack.repos

# 2. resolve dependencies and build
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy
colcon build --symlink-install --event-handlers console_direct+

# 3. source the overlay
source install/setup.bash

# 4. bring up mock hardware + MoveIt (no robot)
ros2 launch lbr_bringup mock.launch.py model:=med7
# in another sourced shell:
ros2 launch lbr_bringup move_group.launch.py model:=med7 mode:=mock rviz:=true
```

The launch files and these arguments are verified against the pinned `jazzy-v2.4.3` source. You can
inspect the installed interface at any time with `ros2 launch lbr_bringup mock.launch.py -s` and
`ros2 launch lbr_bringup move_group.launch.py -s`.

Run the go-to-pose example (recommended path is the launch file, which injects the med7 MoveIt config
and namespaces the node under `/lbr`): see
[`src/lbr_med_examples/README.md`](src/lbr_med_examples/README.md).

For routine startup of the installed robot, use the
[verified stationary KONI/1 ms procedure](docs/05_stationary_fri_startup.md), confirmed on 19 October 2026.
For fresh installation, work through the network, Sunrise-side, and bringup docs in order;
their older KLI settings are not the current stationary-session settings:
[`docs/02_network_setup.md`](docs/02_network_setup.md) ->
[`docs/03_sunrise_side_setup.md`](docs/03_sunrise_side_setup.md) ->
[`docs/04_hardware_bringup_checklist.md`](docs/04_hardware_bringup_checklist.md).

## Repo layout

```
lbr_med_stack/
├── README.md                       # this file
├── LICENSE                         # Apache-2.0
├── lbr_med_stack.repos             # exact upstream stack, FRI 1.15, and IDL pins
├── .python-version / uv.lock       # Python 3.12 tooling environment
├── .gitignore                      # src/ build/ install/ log/ (vcs sources never committed)
├── docs/
│   ├── 01_fresh_machine_setup.md   # ROS 2 Jazzy + overlay from scratch
│   ├── 02_network_setup.md         # FRI subnet, netplan, connectivity checks
│   ├── 03_sunrise_side_setup.md    # cabinet-side Sunrise / LBRServer setup
│   └── 04_hardware_bringup_checklist.md  # phased mock -> real bringup
└── src/
    └── lbr_med_examples/           # first-party ament_python package (committed)
        ├── package.xml
        ├── setup.py / setup.cfg
        ├── resource/lbr_med_examples
        ├── lbr_med_examples/goto_pose.py    # thin MoveItPy plan+execute to a CLI pose
        ├── launch/goto_pose.launch.py       # injects med7 config, namespaces node under /lbr
        ├── test/test_copyright.py
        └── README.md               # run-via-launch usage
```

`build/`, `install/`, `log/` and the **vcs-imported** upstream under `src/` (`src/fri`,
`src/lbr_fri_ros2_stack`, `src/lbr_fri_idl`) are git-ignored; only `src/lbr_med_examples/` is
committed. The vendored sources come from `vcs import`, not from this repo.

## Versioning

`lbr_med_stack.repos` pins `lbr_fri_ros2_stack` to the **tag** `jazzy-v2.4.3` and pins the FRI 1.15 SDK
and matching IDL to exact commits. A single `vcs import` therefore recreates the source set without
following any moving branch.

### Bumping the pinned tag

1. Pick the new tag from the [lbr_fri_ros2_stack releases](https://github.com/lbr-stack/lbr_fri_ros2_stack/releases).
   Confirm it targets ROS 2 Jazzy and check its `CHANGELOG` for breaking launch-file or config changes.
2. Edit the `version:` field in `lbr_med_stack.repos` to the new tag.
3. Update all three exact revisions in `lbr_med_stack.repos`, then re-run the quickstart in a clean clone.
4. Re-verify the mock bringup gate (Phase 0 in `docs/04_hardware_bringup_checklist.md`) before touching the real robot.
5. If the FRI client version changes with the new release, update `FRI_CLIENT_VERSION` and the Sunrise.OS pairing in the docs.

## License

Apache-2.0 (see [`LICENSE`](LICENSE)), matching the upstream lbr-stack license. Copyright 2026 Rotem Atari.

# 04 - Hardware bringup checklist

Phased path from mock (no robot) to real-robot MoveIt. Work the phases in order. Do not skip a phase's gate.

> Safety lines are not optional. Keep the e-stop in hand from Phase 1 on, and run at reduced speed first.

## Phase 0 - Mock (this machine, no robot)

This is the on-machine gate. No network, no cabinet.

- [ ] Overlay builds clean: `colcon build --symlink-install` exits 0 (see `01_fresh_machine_setup.md`).
- [ ] Overlay sourced: `source ~/lbr_med_ws/install/setup.bash`.
- [ ] Mock hardware comes up, no error:

```bash
ros2 launch lbr_bringup mock.launch.py model:=med7
```

- [ ] MoveIt comes up against mock, `move_group` loads the med7 model:

```bash
# in a second sourced shell
ros2 launch lbr_bringup move_group.launch.py model:=med7 mode:=mock rviz:=true
```

- [ ] In RViz, plan and execute a goal to confirm the planning pipeline works on mock.
- [ ] Confirm `move_group` is live: `ros2 topic list | grep move_group`.

> [VERIFY ON FIRST RUN] The launch-file names and args (`mock.launch.py`, `move_group.launch.py`,
> `model:=med7`, `mode:=mock`, `rviz:=true`) follow lbr-stack's documented flow. Confirm exact names/args on
> the pinned tag with `ros2 launch lbr_bringup <file> -s`.

**Gate:** Phase 0 must pass before any real-robot work.

## Phase 1 - Real robot prerequisites (no motion yet)

- [ ] Network up: dedicated NIC on the FRI subnet, `ping -c4 172.31.1.147` succeeds (see `02_network_setup.md`).
- [ ] Host netplan applied: `ip addr show enpXsY` shows `172.31.1.150/16`.
- [ ] Sunrise project synced to the cabinet; safety configuration set and verified (see `03_sunrise_side_setup.md`).
- [ ] LBRServer running on the smartPAD, in FRI **monitoring** mode, client IP `172.31.1.150`, port 30200,
      send period 10 ms, control mode POSITION.
- [ ] **E-stop reachable** and tested.

**Gate:** all of Phase 1 checked before launching hardware bringup.

## Phase 2 - Hardware bringup (FRI handshake, joint states)

```bash
ros2 launch lbr_bringup hardware.launch.py model:=med7
```

- [ ] FRI handshake established: LBRServer reports **connected** on the smartPAD; ROS side logs FRI session up.
- [ ] Joint states stream: `ros2 topic echo /lbr/joint_states --once` returns the 7 joint positions.

> [VERIFY ON FIRST RUN] Confirm the real-bringup launch file and its argument name on the pinned tag with
> `ros2 launch lbr_bringup hardware.launch.py -s` (the file may be named differently, e.g. `bringup`/`real`).
> Confirm the joint-states topic name with `ros2 topic list`.

**Gate:** handshake stable and joint states streaming before enabling MoveIt motion.

## Phase 3 - MoveIt against real hardware

- [ ] Start MoveIt against real hardware (the `mode:=mock` of Phase 0 becomes the real mode for the tag).
- [ ] **E-stop in hand. Reduced speed first.** Set MoveIt velocity/acceleration scaling low.
- [ ] Run the `goto_pose` example at low speed to a conservative, reachable pose (see `examples/goto_pose/README.md`).
- [ ] Confirm planned trajectory in RViz **before** executing on the real arm.

> [VERIFY ON FIRST RUN] The MoveIt real-hardware mode value for `move_group.launch.py` on the pinned tag
> (the non-mock counterpart of `mode:=mock`) is set with `-s` introspection. Do not assume; verify.

**Gate:** every Phase 3 motion is previewed in RViz and executed at reduced speed with the e-stop reachable.

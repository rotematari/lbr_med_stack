# 06 - Robot PC real-time setup

The robot PC (`robot-rt`) is the dedicated host for the KUKA Med 7 and the Kinova Gen3.
It runs Ubuntu's PREEMPT_RT kernel with two performance cores reserved for control loops.
Set up and measured on 8 November 2026.

## Machine

| Item | Value |
| --- | --- |
| Hostname | `bro-103-01-pcl` (`bro-103-01-pcl.eng.tau.ac.il`, registered in TAU DNS) |
| Campus address | `132.66.193.54` (DHCP, stable through the DNS registration) |
| User | `roblab01` |
| CPU / RAM | i9-12900KF: CPUs 0-15 are 8 performance cores (2 threads each), 16-23 efficiency cores; 62 GiB |
| GPU | RTX 3060 Ti, NVIDIA 580 (DKMS), used for the display only |
| Network | One port, `enp4s0` (Realtek RTL8125), on the lab switch shared with both robots |
| Robot addresses on `enp4s0` | `192.170.10.1/24` (KUKA KONI, FRI) and `192.168.1.20/24` (Kinova), both permanent in `/etc/netplan/01-netcfg.yaml` (original in `01-netcfg.yaml.bak-2026-11-08`) |
| OS / ROS | Ubuntu 24.04.4, ROS 2 Jazzy; Kinova workspace in `~/ros2_kortex_ws` |

## Access

Add to `~/.ssh/config` on the workstation and copy a key with `ssh-copy-id robot-rt`:

```
Host robot-rt
    HostName bro-103-01-pcl.eng.tau.ac.il
    User roblab01
```

SSH starts at boot and returned on its own after each reboot. For the screen, xrdp listens on
port 3389: connect an RDP client (for example Microsoft's Windows App) to the hostname above. xrdp
opens its own desktop session, not the physical monitor. Close remote desktop sessions during
robot runs.

## Boot safety

`/etc/default/grub` uses `GRUB_DEFAULT=saved`, `GRUB_TIMEOUT_STYLE=menu` and `GRUB_TIMEOUT=5`
(original in `/etc/default/grub.bak-2026-11-08`). The menu shows for 5 s, and a new kernel does
not become the default until it is chosen with `grub-set-default`. To try a kernel for one boot
only:

```bash
sudo grub-reboot "Advanced options for Ubuntu>Ubuntu, with Linux <version>"
sudo reboot
```

If that boot fails, power-cycle: the saved default boots again.

## Real-time kernel

Ubuntu Pro's maintained PREEMPT_RT kernel, on the free personal subscription:

```bash
sudo pro attach            # prints a code to enter at ubuntu.com/pro/attach
sudo pro enable realtime-kernel --assume-yes
```

This installed `6.8.1-1064-realtime` (generic variant) and disabled Livepatch, which the
real-time kernel does not support. Ubuntu's NVIDIA `dkms.conf` already passes
`IGNORE_PREEMPT_RT_PRESENCE=1`, so the driver built for the new kernel without changes. Check
`dkms status` lists the `-realtime` kernel before rebooting into it. After a clean one-time boot,
the real-time kernel was made the default:

```bash
sudo grub-set-default "Advanced options for Ubuntu>Ubuntu, with Linux 6.8.1-1064-realtime"
```

The generic `6.8.0-110` kernel remains in the boot menu.

## Tuning

`/etc/default/grub.d/98-robot-rt.cfg` reserves performance cores 1-2 (CPUs 2-5) for control
loops and keeps device interrupts, timer ticks and RCU callbacks off them:

```
GRUB_CMDLINE_LINUX_DEFAULT="$GRUB_CMDLINE_LINUX_DEFAULT isolcpus=managed_irq,domain,2-5 nohz_full=2-5 rcu_nocbs=2-5 irqaffinity=0-1,6-23"
```

`/etc/systemd/system/cpu-performance.service` sets the `performance` frequency governor at boot.
`/etc/systemd/system/cpu-dma-latency.service` keeps `/dev/cpu_dma_latency` open at 2 µs, so cores
use C1E (2 µs wake-up) but not C6, C8 or C10 (220, 280 and 680 µs). Before it, CPU 2 entered C10
between FRI packets, and pendant quality fell from EXCELLENT to GOOD during the first MoveIt
motion. `cyclictest` sets the latency to 0 itself while it runs, so the measurements below did not
show this. On the workstation, deep C-states caused most late cycles until
`03_projects/tactile-cable-sysid/operations/robot-sysid/rt_tune.sh` held the latency at 0.

Run `sudo update-grub` after editing either GRUB file. Check after boot:

```bash
uname -v                                   # contains PREEMPT_RT
cat /sys/devices/system/cpu/isolated       # 2-5
sort -u /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor   # performance
```

The scheduler places nothing on CPUs 2-5 by itself, and `isolcpus=domain` also stops it moving
threads between them. Pin individual real-time threads, one per core. Do not start a whole
process with `taskset -c 2-5`: all of its threads then stay on CPU 2. On 8 November 2026 this put
46 driver threads, robot_state_publisher and the DDS threads on CPU 2 with the FRI thread; during a
1 kHz `ros2 topic hz` from the same PC, the pendant reported quality POOR (latency 1.02-1.06 ms,
jitter 0.34 ms) and LBRServer stopped with `ERROR_FRI_CMD_WRONG_STATE_ACTIVE`. Do not run 1 kHz
subscribers on the robot PC during a session.

## Measured latency

`cyclictest` with 1 ms wake-ups at priority 80 while `stress-ng` loaded the CPUs, disk and memory.
Raw histograms and settings are in
[`reports/robot-pc/2026-11-08-latency/`](../reports/robot-pc/2026-11-08-latency/).

| Kernel and setup | Threads measured | Duration | Typical | Worst case | Wake-ups over 1 ms |
| --- | --- | --- | --- | --- | --- |
| Generic 6.8.0-110, `powersave`, load on all CPUs | all 24 CPUs | 77 s (10 min run stopped early) | 1-2 µs | 1622 µs | 4 |
| Real-time 6.8.1-1064, untuned, load on all CPUs | all 24 CPUs | 1 min | 1-2 µs | 234 µs | 0 |
| Real-time, CPUs 2-5 reserved, `performance`, load on the other 20 CPUs | CPUs 2-5 | 1 min | 2 µs | 113 µs | 0 |

The tuned worst case leaves about 0.9 ms of a 1 ms control cycle. Rotem accepted it on
8 November 2026. Under full load the CPU throttles and raises thermal interrupts on every core;
these cannot be moved off the reserved cores. Runs were short, so rarer events may exceed these
worst cases.

## First session from this PC (8 November 2026)

`launch_stationary.sh` runs the 05 stationary launch with domain `85`, `SUBNET` discovery and,
without `--record`, recording disabled (see [Driver scripts](#driver-scripts-and-recordings)). It starts the launch unpinned, then moves the
FRI thread (FIFO 80) to CPU 2 and the controller-manager loop (FIFO 50) to CPU 3 with `taskset -pc`
while the driver awaits the heartbeat.

Result: `COMMANDING_ACTIVE`, pendant quality EXCELLENT (latency 1, jitter 0), no overruns or
drops in the driver log. MoveIt (`move_group.launch.py model:=med7 mode:=hardware rviz:=true`)
on the workstation, domain `85` with `SUBNET`, connected to the driver on this PC and showed
the robot; Rotem confirmed it working. Rotem then executed MoveIt goals: quality fell to GOOD
during the first motion, while CPU 2 still used C10. With `cpu-dma-latency.service` active, the
pendant stayed EXCELLENT during motion. One goal ended with A6 exactly at its 119° driver limit
(2.076942 rad); ros2_control then logged `Command of at least one joint is out of limits` once per
second until the next goal moved A6 away. Keep goals clear of the A6 limit.

## Undo

- Generic kernel: choose it in the boot menu, or `sudo grub-set-default` its entry.
- Tuning: delete `/etc/default/grub.d/98-robot-rt.cfg`, then `sudo update-grub`;
  `sudo systemctl disable cpu-performance`; `sudo systemctl disable --now cpu-dma-latency`.
- Real-time kernel package: `sudo pro disable realtime-kernel`.

## KUKA stack and network on this PC

- FRI link: on 8 November 2026 `192.170.10.1/24` moved here from the workstation, which now uses
  `192.170.10.4/24` (NetworkManager "Wired connection 1") and can still reach the cabinet. The
  cabinet at `192.170.10.2` answers through the switch; every ping reply arrives twice from one MAC
  address, with TTL 64 and 255.
- Built on 8 November 2026 in `~/lbr_med_stack` (commit `e945e89`, recording patch applied) with
  the recorder overlay in `~/lbr_med_stack/overlays/live-signals`; the offline `fri_driver_hooks`
  test passes. `overlays/` needs a `COLCON_IGNORE` file, or the base build also picks up the MCAP
  sources in `overlays/deps/`. The 307 older `ros-jazzy-*` packages were upgraded first: the
  October ros2_control failed to load against the April `diagnostic_updater`.
- `~/.bashrc` (original in `~/.bashrc.bak-2026-11-08`) sets `ROS_DOMAIN_ID=85`,
  `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET` and `FRI_CLIENT_VERSION=1.15`. The Kinova workspace
  `~/ros2_kortex_ws` builds its own ros2_control, so it is no longer sourced by default; the
  `kinova` alias loads it. Never load it in a KUKA terminal.

## Driver scripts and recordings

The robot PC's scripts are tracked in [`scripts/robot-rt/`](../scripts/robot-rt/) (8 November 2026):
`launch_stationary.sh` and `build_lbr.sh`. On this PC they will run from `~/lbr_med_stack/overlays/` as
symlinks to the tracked files once the switch-over in Open items is done; the untracked copy there has no
`--record`. Start the driver from the workstation:

```bash
ssh -t robot-rt '~/lbr_med_stack/overlays/launch_stationary.sh'                 # no recording
ssh -t robot-rt '~/lbr_med_stack/overlays/launch_stationary.sh --record tau40'  # recording services on
```

`--record LABEL` sets `LBR_RECORDING_ROOT=~/lbr_recordings/<date>-LABEL-<HHMMSS>` on this PC; each run the
recording services start lands in `<root>/run-<id>` and the process log in `<root>.driver.log`. The launch is
otherwise identical (JTC, 1000 Hz controllers, default system config with tau 0.04, the same thread pinning).
The workstation moves recordings into `03_projects/tactile-cable-sysid/runs/robot-sysid/` with
`03_projects/tactile-cable-sysid/operations/robot-sysid/fetch_recordings.sh` (rsync; Rotem, 8 November 2026:
the robot PC's copy is deleted after the transfer, except while a driver runs there, when it only copies). The
folder names match the workstation's earlier `<date>-tau40-<HHMMSS>` sessions, so the run sheets' analysis
commands find them. The recorder's message schema (`acquisition_v1.desc`) is byte-identical in this PC's and
the workstation's `live-signals` builds, so the workstation's `LBR_GENERATED_PYTHON` decodes these recordings.
The recorder creates the root when the driver starts, so a `--record` session with no recording leaves an
empty `<root>/`; delete it before analysis commands that pick the latest session.

## Open items


- 02 and 05 carry a note pointing here (8 November 2026); their bodies still describe the workstation as the FRI
  host and `LOCALHOST` discovery. The workstation live environment (`tactile-cable-sysid/operations/robot-sysid/live_env.sh`)
  now uses `SUBNET`, and its `live_driver.sh` refuses to start without `192.170.10.1`.
- Switch-over to the tracked scripts (Rotem agreed, 8 November 2026; do it with no driver running, after the
  scripts are committed and pushed): on this PC `cd ~/lbr_med_stack && git pull`, then
  `ln -sfn ../scripts/robot-rt/launch_stationary.sh overlays/launch_stationary.sh` and the same for
  `build_lbr.sh`. Until then this PC runs its untracked copy, which has no `--record`.
- `--record` not yet run live: check that `run-1` appears under the root after a prepare/start/stop, that
  `fetch_recordings.sh` moves it, and that `run_check.py` reads it.
- Workstation `~/.bashrc` now sets `ROS_DOMAIN_ID=85` and `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET` (Rotem,
  8 November 2026; original in `~/.bashrc.bak-2026-11-08`). New terminals there join the live robot's domain:
  mock bringups must source `viz/mock_env.sh` (domain 86).
- Kinova (not in use): pin its controller to CPUs 2-5 before a 1 kHz run.
- A temporary passwordless sudo rule, `/etc/sudoers.d/90-setup-temp`, exists for the setup;
  remove it when setup is finished.

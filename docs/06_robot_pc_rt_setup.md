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
| Robot addresses on `enp4s0` | `192.168.1.20/24` (Kinova, permanent in `/etc/netplan/01-netcfg.yaml`) |
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
Run `sudo update-grub` after editing either GRUB file. Check after boot:

```bash
uname -v                                   # contains PREEMPT_RT
cat /sys/devices/system/cpu/isolated       # 2-5
sort -u /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor   # performance
```

The scheduler places nothing on CPUs 2-5 by itself. Control loops must be pinned there, for
example with the `cpu_affinity` parameter of the ros2_control controller manager, or
`taskset -c 2-5` for other processes.

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

## Undo

- Generic kernel: choose it in the boot menu, or `sudo grub-set-default` its entry.
- Tuning: delete `/etc/default/grub.d/98-robot-rt.cfg`, then `sudo update-grub`;
  `sudo systemctl disable cpu-performance`.
- Real-time kernel package: `sudo pro disable realtime-kernel`.

## Open items

- KUKA FRI link: `192.170.10.1/24` moves here from the workstation (02 and 05 still describe the
  workstation). A ping to the cabinet at `192.170.10.2` through the switch succeeded from a
  temporary `192.170.10.3`.
- Build this overlay on the robot PC (01) and repeat the stationary 1 ms startup (05).
- Kinova 1 kHz run without controller-manager overruns, pinned to CPUs 2-5.
- A temporary passwordless sudo rule, `/etc/sudoers.d/90-setup-temp`, exists for the setup;
  remove it when setup is finished.

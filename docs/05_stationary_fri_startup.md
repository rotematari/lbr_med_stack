# KUKA Med 7 stationary FRI startup

This is the current procedure for the already commissioned lab robot. It reproduces
the KONI connection used for the successful one-second stationary recording on
19 October 2026. Use it for routine startup; the older installation guides describe
an earlier KLI setup. Robot power-on, pendant operation and live connection require
Rotem as operator. This procedure holds the current joint positions; motion is a
separate experiment after recording inspection.

## Settings from the successful run

| Setting | Value |
| --- | --- |
| Robot | KUKA LBR Med 7 R800 |
| Controller OS | Sunrise.OS Med 1.15, confirmed by Rotem |
| Controller FRI option version | Not yet independently identified; do not record it as 1.16 |
| PC FRI client SDK | 1.15, verified in the installed SDK header |
| Physical connection | Existing KONI connection, confirmed by Rotem |
| Controller KONI address | `192.170.10.2` |
| PC address | `192.170.10.1/24` (mask `255.255.255.0`) |
| PC Ethernet interface on this machine | `enp130s0` |
| FRI transport | UDP port `30200` |
| Pendant mode | **AUT**, including LBRServer setup |
| Robot application | **LBRServer** |
| LBRServer remote PC | **192.170.10.1** |
| Send period / receive multiplier | **1 ms / 1** |
| Control mode / client command mode | **POSITION / POSITION** (driver reports `POSITION_CONTROL_MODE` / `POSITION`) |
| ROS controller manager rate | **1000 Hz** |
| ROS discovery used for this session | Domain `85`, `LOCALHOST` |

KONI identifies the existing connection, not a cabinet socket number to guess.
Do not use the old X69=KONI description: the Med cabinet manual identifies X69 as
the KSI service interface. Keep the established cabling.

## 1. Power on and prepare the pendant

1. Power on the cabinet using the lab's normal procedure and wait for the smartPAD
   and controller to finish starting. Keep the work area clear and the normal stop
   controls accessible.
2. Use the existing commissioned project and safety configuration. Resolve any
   controller fault through the normal lab procedure before starting the session.
3. Select **AUT**. The old instruction to set up this LBRServer session in T1 is
   superseded by Rotem's current procedure.
4. Select **LBRServer** and prepare the settings in the table above. In particular,
   the remote PC is **192.170.10.1**, not `172.31.1.148` or `172.31.1.150`.
   If configuration dialogs appear only when the program starts, enter these values
   when prompted in step 5 below.

The application to run here is LBRServer. Transport-position and sine-overlay
examples are separate motion applications and are outside this stationary startup.

## 2. Check the PC network

Inspect the Ethernet address:

```bash
ip -brief -4 address show dev enp130s0
```

If `192.170.10.1/24` is missing, add it temporarily:

```bash
sudo ip address add 192.170.10.1/24 dev enp130s0
```

This adds an address alongside the existing lab and other-robot addresses. It does
not replace them. It requires local sudo authentication and is not a persistent
configuration: recheck after reboot or a network reconnect. Skip the add command
when the address is already present. On another PC, first identify its actual
Ethernet interface instead of copying `enp130s0`.

Verify the selected source address and reachability:

```bash
ip -4 route get 192.170.10.2
ping -c 3 -W 1 -I 192.170.10.1 192.170.10.2
```

The route must show `dev enp130s0 src 192.170.10.1`. The `/24` address normally adds
the needed connected route automatically. A successful ping confirms network
reachability; the FRI handshake is checked separately below.

## 3. Prepare the PC runtime

Use one FRI client. Before starting a new one, check whether UDP 30200 is already
owned:

```bash
ss -lunp 'sport = :30200'
```

An existing `ros2_control_node` means a client may already be connected. Reuse or
stop that session with the operator; do not start a second standalone/ROS FRI client.

The following is the exact overlay order used for the successful recording:

```bash
lbr_ws=/home/rotem/Documents/Claude/Projects/PHD/packages/lbr-med-stack
source /opt/ros/jazzy/setup.bash
source "$lbr_ws/install/local_setup.bash"
source /tmp/lbr-duration-change/install/local_setup.bash

export ROS_DOMAIN_ID=85
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
unset ROS_LOCALHOST_ONLY
export RCUTILS_COLORIZED_OUTPUT=0
```

The `/tmp/lbr-duration-change/install` overlay contains the duration-capable recorder
driver and its rebuilt hardware plugin. It is a temporary build location and may
disappear; if it is missing, restore/rebuild the paired overlay following
[the recorder build instructions](../patches/fri-recording/README.md) before using
this recording procedure. Do not silently substitute an old hardware plugin for
the rebuilt one. SDK and ROS controller source were not changed for the recorder.

For connection only, disable recording explicitly:

```bash
unset LBR_RECORDING_DIRECTORY LBR_RECORDING_ROOT
```

For an agreed **stationary recording** (10 seconds in this example), use this instead, before launch:

```bash
capture_day=$(TZ=Asia/Jerusalem date +%F)
capture_stamp=$(TZ=Asia/Jerusalem date +%H%M%S)
capture_parent="/home/rotem/Documents/Claude/Projects/PHD/03_projects/tactile-cable-sysid/runs/robot-sysid/${capture_day}-stationary-fri"
mkdir -p "$capture_parent"
export LBR_RECORDING_DIRECTORY="$capture_parent/run-${capture_stamp}-$$"
export LBR_RECORDING_DURATION_SECONDS=10
```

One-shot mode: create only the parent directory. The recorder requires the run
directory to be new and creates it itself. It records once per client process. It
allows **60 s from recorder startup to the first eligible POSITION/COMMANDING_ACTIVE
state**; prepare the pendant and network before launching.

For **several recordings in one session via ROS services** (instead of the one-shot
variables above; both cannot be set), before launch:

```bash
unset LBR_RECORDING_DIRECTORY LBR_RECORDING_DURATION_SECONDS
export LBR_RECORDING_ROOT="/home/rotem/Documents/Claude/Projects/PHD/03_projects/tactile-cable-sysid/runs/robot-sysid/$(TZ=Asia/Jerusalem date +%F)-services-$(TZ=Asia/Jerusalem date +%H%M%S)"
```

Then, after the handshake in step 6, control runs with the service calls in
[the acquisition README](../src/lbr_acquisition/README.md#recording-services-in-the-live-driver--20-october-2026).
The launch log must show `Recording services enabled`; an ERROR naming
`LBR_RECORDING_ROOT` means the variable was not set in the launch terminal.
Service mode rules: call `prepare`/`start` only after the robot reports
`COMMANDING_ACTIVE`; the default first-sample timeout is **5 s from `start`**
(`config/recording/limits.yaml`). Any `FAILED` run, source loss or driver
deactivation blocks every later `run_id` until the driver process is restarted.
Confirm the service path with `ros2 service list | grep recording` first.
This mode has offline verification only until its first operator-led run.

## 4. Start the sole PC client

In the prepared terminal:

```bash
lbr_launch_log="${LBR_RECORDING_DIRECTORY:-/tmp/lbr-stationary-$(TZ=Asia/Jerusalem date +%Y%m%d-%H%M%S)}.process.log"
ros2 launch lbr_bringup hardware.launch.py \
  model:=med7 \
  ctrl:=joint_trajectory_controller \
  ctrl_cfg:="$lbr_ws/src/lbr_med_examples/config/hardware_controllers_1000hz.yaml" \
  2>&1 | tee "$lbr_launch_log"
```

The absolute controller YAML path is intentional: it selects the checked 1000 Hz
configuration in the source tree. The joint trajectory controller is loaded but
receives no trajectory during this stationary procedure.

Expected initial messages include `Socket opened successfully`, UDP `30200`, and
`Awaiting robot heartbeat`. The successful run also reported FIFO priority `80`.
An open socket alone does not establish a robot connection.

## 5. Start or resume LBRServer on the pendant

With the PC client listening, start/resume **LBRServer** using the pendant's normal
Play control in **AUT**. Enter or confirm:

1. Remote PC: `192.170.10.1`.
2. Send period: `1 ms`.
3. Receive multiplier: `1`.
4. Control mode: `POSITION`.
5. Client command mode: `POSITION`.

Follow the existing application's prompts through to the connection. Keep the
robot at its current position. Do not launch MoveIt or publish a trajectory as a
connection test.

## 6. Verify the handshake and any recording

The successful run progressed through:

```text
MONITORING_WAIT -> MONITORING_READY -> COMMANDING_WAIT -> COMMANDING_ACTIVE
```

Confirm `Robot connected`, **COMMANDING_ACTIVE**, **EXCELLENT** connection quality,
and a reported period of **0.001 s**. EXCELLENT alone is not sufficient. In a second
terminal with the same ROS environment, these read-only checks are also useful:

```bash
ros2 param get /lbr/controller_manager update_rate
ros2 control list_controllers -c /lbr/controller_manager
ros2 topic echo /lbr/joint_states --once
```

Expect update rate `1000` and active controllers. Joint-state publication is a ROS
connectivity check; with this driver's `open_loop=true`, use the native recording
for actual encoder positions rather than treating published ROS positions as raw
measurements.

When recording is enabled, its selected-duration window begins at the first eligible
state. Duration accepts positive decimal seconds (default 1), with no fixed
10-second ceiling; it must fit the checked clock/storage budgets. The live queue
and native verification memory stay bounded. Recording then finalizes independently while FRI control continues. Inspect
the sibling `<run-directory>.capture.json` and read the bundle with the Python
reader. Require all seven actual positions, all seven submitted positions, and
measured, controller-commanded and estimated external joint torques. Check timing
and gaps with Rotem before accepting the run or discussing motion.

## 7. End the session when the operator is ready

Stop LBRServer with the normal pendant program-stop procedure, then use Ctrl+C in
the PC launch terminal. Confirm the client has exited and UDP 30200 is free before
starting another session. The recorder finishing does not stop LBRServer or the
PC client. Existing blocking driver shutdown may delay exit; do not start a second
client while the first still owns the port.

## Troubleshooting from this session

| Observation | Next check |
| --- | --- |
| No heartbeat | PC `192.170.10.1/24`, route source, pendant remote IP and LBRServer actually running |
| Port 30200 already owned | Identify the existing session; coordinate its shutdown before another client |
| Recording ends `NoData` | No eligible commanding state arrived within 60 s; fix startup, then use a fresh process and run path |
| ROS asks to increase update rate despite `1000` | Verify the parameter; the warning can indicate delayed ROS read cycles, not an incorrectly selected rate |
| Cannot see ROS nodes from another terminal | Match domain `85` and localhost discovery settings |
| Confusion about 1.15 versus 1.16 | OS Med 1.15 and PC SDK 1.15 are verified; controller FRI option version remains unconfirmed |

The reference capture is
`03_projects/tactile-cable-sysid/runs/robot-sysid/2026-10-19-stationary-fri/run-111209`
relative to the PHD workspace, with sibling `.inspection.json`, `.capture.json` and
`.process.log`. It contains 1000 states and 1000 matched submissions, all five
requested signal groups, constant submitted positions and exact 1 ms controller
timestamp increments. This is evidence of successful stationary acquisition, not
a blanket real-time qualification or approval for motion.

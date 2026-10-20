# Project-owned FRI recorder additions

`PHD-FRI-RECORDER` identifies our additions. Upstream C++/CMake blocks use
BEGIN/END markers with a short purpose; new acquisition files use one purpose
header. The patch is the complete change record, including replaced lines.

Base: `lbr_fri_ros2_stack` commit
`14ecad839f482da0ee4ac4b9d222d0787db3bf11` (jazzy-v2.4.3).
SDK: unchanged FRI 1.15, archive SHA256
`7c6a179dfc89ff4aea0b01c27174294b0069d5adb39e0d8f5db060efda2476d5`.

This patch implements roadmap steps 3–4: raw-state and final position-submission
observation, plus an opt-in configurable-duration session. The project-owned adapter and
storage live in `src/lbr_acquisition`; the seven-file driver patch leaves control
calculations, filters, guards, ROS controllers and KUKA SDK source unchanged.
Recording support defaults OFF. The final independent review and operator-led
qualification are recorded in the project’s one-second-stationary-driver-plan.md.

## Apply or inspect

From the `packages/lbr-med-stack` root, against a clean checkout at the base:

```bash
git -C src/lbr_fri_ros2_stack apply --check ../../patches/fri-recording/lbr-fri-ros2-recording.patch
git -C src/lbr_fri_ros2_stack apply ../../patches/fri-recording/lbr-fri-ros2-recording.patch
```

Check reversal against an applied patch; run the second line only when intending
to remove this patch and after reviewing other local changes:

```bash
git -C src/lbr_fri_ros2_stack apply -R --check ../../patches/fri-recording/lbr-fri-ros2-recording.patch
git -C src/lbr_fri_ros2_stack apply -R ../../patches/fri-recording/lbr-fri-ros2-recording.patch
```

Locate the additions, including the ignored upstream checkout:

```bash
rg -n --no-ignore 'PHD-FRI-RECORDER' src/lbr_fri_ros2_stack/lbr_fri_ros2 src/lbr_acquisition
```

## Offline build and test (verified 2026-10-19)

Run from the stack root. Dependencies come from the existing ROS Jazzy/stack
installation; output goes to separate build/install directories. These commands
compile and run an in-memory fixture; they do not connect to a robot.

```bash
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
colcon --log-base /tmp/lbr-duration-change/colcon-log build \
  --packages-select lbr_fri_ros2 lbr_ros2_control \
  --allow-overriding lbr_fri_ros2 lbr_ros2_control \
  --build-base /tmp/lbr-duration-change/build \
  --install-base /tmp/lbr-duration-change/install \
  --cmake-args -DBUILD_TESTING=ON -DLBR_BUILD_RECORDING=ON \
  -DLBR_ACQUISITION_SOURCE_DIR="$PWD/src/lbr_acquisition" \
  -DMCAP_SOURCE_DIR=/tmp/lbr-review-build/mcap-00e47a1ba6901cbc0cf448cccebbc554c545c4fa
source /tmp/lbr-duration-change/install/local_setup.bash
ctest --test-dir /tmp/lbr-duration-change/build/lbr_fri_ros2 \
  -R '^fri_driver_hooks$' --output-on-failure
```

The final setup line is necessary: otherwise the loader can select the ordinary
installed driver, which has no recorder symbols. Colcon reports dependencies
reused from the existing installation; those packages are not rebuilt here.

The single fixture checks raw actual q and all three torque arrays, exact filtered
and guard-corrected submitted q, current-packet absence, state association,
transition duplication and outgoing byte equality with recording off/on/full.
It also records 10,000 states and 10,000 submissions over a half-open ten-second
window, finalizes while the FRI producer continues, exercises no-data/source-stop/
malformed-input endings, and checks setup without opening a socket. All fixture
measurements are synthetic. Its fake clock advances the ten-second window without
requiring a real-time robot simulator.

Verified 2026-10-19: combined fixture passes, acquisition CTest 13/13, Python
robot_sysid 97/97; Python reads the integrated 2000-event bundle as `fri-sdk` with
a successful Duration outcome. Recording ON/OFF builds pass. `lbr_ros2_control`
was rebuilt against the patched headers. A temporary pluginlib smoke test loaded
and constructed its hardware plugin alongside ROS's MCAP storage library in both
load orders, without initialization/configuration/activation or socket calls.
No defined MCAP symbols are exported by the patched driver. This is a process-load
check, not hardware or hard-real-time qualification.

The full driver CTest retains the three pre-existing `test_command_interfaces`
cases failing with `Time constant must be positive and greater zero.` This was
reproduced on the untouched base: its fixture initializes filter time constants
to zero. No unrelated upstream test fix is included.

## Explicit recording stop — offline evidence, 19 October 2026

The project-owned `FriRecording::request_stop(run_id)` requests Manual stop without
waiting or allocating. Wrong run IDs are rejected; duplicate requests preserve
the existing terminal cause. Before start, it cancels the prepared recording.
Requests may overlap already-running callbacks/finalization, but must not overlap
`start()` or destruction; complete a prepared cancellation before starting.
Use the existing owner-thread `wait()` and final status/outcome to distinguish
request acceptance from completed saving. The object stays single-use; no ROS
service or live recorder replacement is introduced by this increment.

The isolated build is `/tmp/lbr-explicit-stop/{build,install}`. The extended
`fri_driver_hooks` fixture and all 13 acquisition CTest suites pass. Full CTest
reports 14/15 passing because the existing upstream `test_command_interfaces`
suite still has its three documented zero-time-constant failures. Python bundle
tests pass 25/25. Fresh Python readback confirms Manual finalization and exact
actual/submitted positions plus all three torque arrays: three State/Submission
pairs normally, or three States/two Submissions when stop falls between the last
pair. The unpaired terminal State is retained without synthesizing a command.
Five subsequent source cycles and a cycle after recording-owner destruction send
the same bytes as recording disabled. Stop requests allocate zero times in the
fixture. Empty/prepared cancellations, wrong IDs, repeats and prior faults are
covered. These are offline results, not a robot or real-time qualification.

KUKA SDK, upstream driver/controller sources and the tracked driver patch are
unchanged by this increment. No installed robot overlay is replaced.

## Operator-controlled activation

After source selection and the stationary setup are agreed with Rotem, set
`LBR_RECORDING_DIRECTORY` to a **new absolute directory** in the existing ROS
launch process. Unset means disabled. Empty/relative/existing paths fail setup;
setting it on a build without recording support fails explicitly. Preparation
happens before UDP opens. No new controller, second client, or target change is
introduced. Source the isolated overlay last, and use the rebuilt hardware consumer.

The timer starts at the first eligible POSITION/COMMANDING_ACTIVE state, not at
process launch. Set `LBR_RECORDING_DURATION_SECONDS=10` for ten seconds; unset defaults to one
second. Positive decimal durations (up to nanosecond precision) have no fixed
ceiling beyond representable clock/storage budgets. Invalid values fail before
UDP opens. The total record budget derives from duration at 1 kHz with two records
per cycle plus 4096 headroom; file budgets derive from that total. Fixed settings:
60-second initial eligible-state timeout, 4096 queue slots, 1 ms coordinator polling
and 5-second writer-stall budget. The live writer holds a 64-byte acknowledgement
summary, and native finalization scans/hashes the file in bounded blocks.
Expiry closes recorder admission and drains/finalizes on background workers while
ordinary control continues. A source failure stops recording without issuing a
robot command. No automatic retry or recorder restart occurs in the same App.

Artifacts include `signals.mcap`, manifest/quality files, session diagnostics and
`build.txt` (embedded source/build identities and runtime limits). The terminal
outcome is the sibling `<directory>.capture.json`. File completion and signal
validity are separate: inspect the outcome and Python bundle before accepting a
run. `bundle_success` reports generic capture validity; it does not establish this
milestone. Accepting the stationary milestone additionally requires seven actual
positions, exact submitted positions, and all three seven-joint torque arrays
throughout the captured States, with timing/gaps inspected with Rotem. Missing
torque is unavailable, not zero, and prevents milestone acceptance. Build provenance is a diagnostic,
not a signed attestation or an outcome-hashed artifact.

Driver class layout changed. Rebuild all binary consumers against these headers;
never mix an old hardware plugin with the patched library. Install the adapter
before callbacks; runtime replacement is unsupported. Its endpoint remains alive
through client ownership, and a gate closes future offers and waits for in-flight
offers before joining the recording session. The upstream detached FRI worker and
its blocking shutdown behavior are unchanged; stopping recording does not require
that worker to exit. A hung filesystem can leave recorder cleanup waiting
after the original control shutdown; the stall budget detects a stall but cannot
interrupt an operating-system write. App destruction issues its original stop
request and closes the socket before waiting for recorder cleanup.

The observer records the driver's final successful position setter, not every
internal SDK setter, and not proof of packet transmission, receipt or execution.
The existing ROS driver stays the sole active FRI client. Rotem must be involved
as operator before any connection; stationary capture and inspection precede motion.

## Duration and memory extension — offline evidence, 2026-10-19

Rotem approved configurable finite duration and constant-memory live verification.
SDK, seven-file driver patch and ROS controller source are unchanged by this extension.
The newly built paired overlay is `/tmp/lbr-duration-change/install`; the older
one-second process can continue using its old overlay until the operator stops it.

Native tests: 13/13. Python robot_sysid: 99/99, including a 60,000-record bundle
larger than 16 MiB. The real-driver/SDK offline fixture reads back 10,000 states
and 10,000 submissions with all three torque arrays and a Duration outcome; emitted
command bytes match recording disabled. A 60.25-second configuration and invalid
inputs are checked without opening sockets. Existing one-second live readback
still succeeds. Native memory probes, including writing/finalization/outcome hashing,
used peak RSS 11,372 KiB at 10,000 records and 11,480 KiB at 120,000 records. These
measurements support bounded process memory; they are not a real-time guarantee.
Independent source review (GPT-6 Sol, high) found no confirmed actionable defect;
it did not rerun the tests. Operator-led ten-second capture remains pending. The
full driver suite still has the three previously reproduced zero-time-constant
command-interface failures; acquisition and integration checks pass.

## Recording services — offline evidence, 20 October 2026

The patch now also touches `lbr_fri_ros2/package.xml`, `lbr_ros2_control/src/system_interface.cpp`
(one call: `app_ptr_->attach_recording_services(get_node())`) and a comment in
`lbr_bringup/launch/hardware.launch.py`. `App` gains `LBR_RECORDING_ROOT` manager mode,
a producer-exit flag and service attachment; the marked CMake block compiles the
project-owned `src/lbr_acquisition/ros/` glue and test, and installs
`config/recording/limits.yaml`. Interface and yaml-cpp libraries are linked privately
(not exported). New project-owned package: `src/lbr_recording_interfaces`.
Build it together with `lbr_fri_ros2 lbr_ros2_control`. Verified: patch applies to the
upstream revision and reverses cleanly; recording-OFF build contains no recorder
symbols; native CTest 15/16 (known upstream `test_command_interfaces` only).
See [the acquisition README](../../src/lbr_acquisition/README.md#recording-services-in-the-live-driver--20-october-2026).


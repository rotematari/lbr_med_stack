# Offline acquisition format prototype

Milestone 1: write synthetic joint-position states and submitted positions to
MCAP/Protobuf, then check them with an independent native reader and the project
Python reader. This executable has no ROS or FRI dependency and does not connect
to hardware. Milestone 2A adds the bounded queue/lifecycle core described below;
2B.4 adds an offline streaming session. The 18 October format increment adds
three distinct joint-torque signals to the baseline. These standalone fixtures
exercise synthetic values. The optional driver adapter subsequently completed a
one-second live stationary capture on 19 October, followed by a ten-second
stationary capture with the configurable-duration, bounded-memory extension.

## Recording services in the live driver — 20 October 2026

Numbered recordings on one FRI connection, controlled through ROS services while
the robot stays under its normal position controller. **Opt-in, set before launch:**

```bash
export LBR_RECORDING_ROOT=/abs/path/session-$(date +%Y%m%d-%H%M%S)  # required
export LBR_RECORDING_CONFIG=/abs/path/limits.yaml                   # optional override
```

Runs are saved to `<LBR_RECORDING_ROOT>/run-<run_id>/` with `request.yaml` (label,
requested limits). `LBR_RECORDING_ROOT` and the one-shot `LBR_RECORDING_DIRECTORY`
are mutually exclusive. When `LBR_RECORDING_ROOT` is unset in a recording build, the
driver runs normally, logs an ERROR with these instructions, and every service refuses
with the same message. Builds without `LBR_BUILD_RECORDING=ON` reject both variables.

**Limits** come from `config/recording/limits.yaml`, installed to
`share/lbr_fri_ros2/config/recording/` (source: `ros/config/recording/limits.yaml`).
Unknown keys or invalid values stop the driver at startup.

| Key | Default | Meaning |
|---|---|---|
| `max_duration_cap_s` | 300 | Largest per-run `max_duration_s` (watchdog for a missed stop). |
| `default_first_sample_timeout_s` | 5 | Used when a request sends `first_sample_timeout_s <= 0`. |
| `poll_period_ms` | 50 | Timer that advances handoffs and finalizes stopped runs. |

**Services** (package `lbr_recording_interfaces`) on the hardware component node,
expected at `/lbr/lbr_system_interface/recording/{prepare,start,stop,status}`;
confirm with `ros2 service list | grep recording`.

```bash
srv=/lbr/lbr_system_interface/recording
ros2 service call $srv/prepare lbr_recording_interfaces/srv/PrepareRecording "{run_id: 1, max_duration_s: 30.0, label: 'stationary'}"
ros2 service call $srv/start   lbr_recording_interfaces/srv/RecordingCommand "{run_id: 1}"
ros2 service call $srv/status  lbr_recording_interfaces/srv/RecordingStatus  "{run_id: 1}"   # until phase 3 (RECORDING)
ros2 service call $srv/stop    lbr_recording_interfaces/srv/RecordingCommand "{run_id: 1}"
ros2 service call $srv/status  lbr_recording_interfaces/srv/RecordingStatus  "{run_id: 1}"   # until phase 5 (FINALIZED)
```

The client chooses strictly increasing `run_id`s. `prepare` accepted means files
and buffers are ready; status RECORDING means the first valid sample was captured;
FINALIZED means capture ended, data was written and the outcome published. Runs
finalize automatically after stop or the watchdog. A failed run, source loss or
driver deactivation blocks further runs until the driver restarts. Requests are
served by one callback group (the manager's single owner); the default services
QoS queue keeps 10 pending requests per service, so do not flood it.

Offline verification: a services test runs the real driver client over the
in-memory connection while a multithreaded executor serves the services: two
Manual runs on one adapter, stale/duplicate/cap/label/existing-output rejections,
cancellation, concurrent status requests, producer exit while recording, disabled
mode and limits/environment validation. Python readback of the service-recorded
runs verifies all five signal groups. ASan/UBSan clean (leak checking off: rclcpp
shutdown). No live run yet.

## Reusable recording manager — 19 October 2026

`FriRecordingManager` records several runs, one after another, on one FRI
connection. Its `adapter()` is installed once (in `ObservedConnection` and
`AsyncClient::set_recorder`) and never replaced. Each run is a fresh single-use
`FriRecording`; the manager owns at most one run plus one compact last result.

```text
prepare(config, new_dir, descriptor) -> start(id) -> [first State] -> request_stop(id)
  -> poll() until detached and workers joined -> finish(id) -> next prepare(...)
```

Routing changes only at the start of an FRI cycle. `start()` and stop publish one
request in a single-slot mailbox; the FRI thread picks it up in `reset_packet()`
at the next receive entry and acknowledges it. The manager releases a run only
after the FRI thread has acknowledged detachment, or after `producer_stopped()`
confirms that no further callbacks can occur. A timeout or closed socket is not
that confirmation. Routed hooks perform no allocation, locks, waits or file work;
the routing atomics are checked lock-free at compile time.

Status meanings:

| Field | Meaning |
|---|---|
| `Armed` | Started; first eligible State not yet admitted. |
| `Recording` | The recorder accepted its first eligible State. |
| `Stopping` | Admission closed (Manual, duration or fault); detachment/drain pending. |
| `detached` | The FRI thread acknowledged detach, or the producer is confirmed stopped. |
| `finish()` true | The run is retired. Check `phase`/`session` for its quality. |
| `Finalized` | Complete, non-empty, fault-free capture with published outcome. |
| `Cancelled` | Prepared run stopped before start; another run may follow. |
| `Failed` | Empty, faulted or unpublished run. Blocks further runs; no retry. |

`Finalized`/`Failed` come only from the run's own outcome. Source loss after an
earlier Manual or duration stop does not change that first cause, so a complete
run stays `Finalized` even if FRI closes before the detach cycle; no new run may
start after source loss or `producer_stopped()`.

All manager methods belong to one non-FRI owner thread, which must call `poll()`
regularly. Only `RecorderAdapter::source_stopped()` may come from another thread.
Run IDs must be nonzero and strictly increasing; output directories must be new.
Source loss blocks new runs. Destroying the manager requires a quiescent producer.
The manager is not yet installed in the live driver App and has no ROS interface.

Offline verification extends the in-memory real-driver fixture with A, idle, then
B on one connection; automatic duration; ten repeated Manual runs; stop between a
routed State and Submission with zero routed-hook allocations; stop before attach;
worker completion before detach acknowledgement; shutdown while Prepared, Armed,
Recording and Stopping; a complete Manual run finished after confirmed producer
end; source loss, no data, invalid data and failed outcome publication; stale,
duplicate and reused IDs/directories. Allocation checks cover all four routed hooks. Outgoing command bytes
match recording disabled. Two deliberate mutations of the handoff are each caught.
The fixture also passes under ASan/UBSan; ThreadSanitizer is unavailable on this
host. Python readback of all successful manager bundles verifies all five signal
groups, state associations and disjoint A/B events. No hardware or real-time
qualification is claimed.

## Explicit stop on the FRI recording object — 19 October 2026

The optional project-owned `FriRecording` now exposes
`bool request_stop(uint64_t run_id) noexcept`. A matching ID requests Manual stop
through the existing atomic admission gate; a different ID returns false without
stopping the run. The request performs no allocation, disk work or wait. Repeated
requests do not replace an existing stop cause. Before `start()`, it cancels the
prepared run and prevents its later start.

A true return means the request was recognized, **not** that saving completed.
The existing non-FRI finalizer drains and publishes the outcome; check `status()`
and use `wait()` from the owner thread. Requests may overlap running callbacks
and finalization, but must not overlap `start()` or object destruction. Complete
a prepared-run cancellation before attempting `start()`.
Stopping recording does not close FRI or issue a robot stop. `FriRecording` remains
single-use: reusable sessions and ROS start/stop controls are separate future work.

The existing in-memory driver fixture verifies Manual finalization, continued
source callbacks and identical outgoing command bytes, stale/duplicate requests,
pre-sample cancellation, earlier fault preservation and owner release with retained
callbacks. A stop between State and Submission admission may leave one terminal
State without a Submission; counters retain that fact and no command is invented.
An empty manually stopped run has no captured-data evidence, regardless of generic
file completion. The trajectory runner must apply its required coverage checks.

Offline verification: driver fixture plus 13 acquisition suites pass; 25 Python
bundle tests pass. Python readback verifies three States/three Submissions for the
normal case and three/two for the boundary case, including all five signal groups.
The full driver CTest still includes the documented upstream command-interface
suite's three zero-time-constant failures. Build logs retain warnings from the
test-only allocation instrumentation and dependency/build configuration; Python
reports three dependency deprecation warnings. No new hardware qualification is
claimed. See [the driver integration notes](../../patches/fri-recording/README.md).

## KUKA status signals — 24 October 2026

Each State can also carry three KUKA-side signals, each present only when the FRI
packet held the field (presence comes from the packet inspector, values from the SDK):

| Field | Live source (FRI tag) | Meaning |
|---|---|---|
| `q_commanded_rad` | `getCommandedJointPosition()` (monitorData 3) | Controller's last commanded joint positions, rad; 0 or 7 values |
| `tracking_performance` | `getTrackingPerformance()` (ipoData 12) | Documented 1.0 = commands executed instantly, dropping towards 0 when velocity, acceleration or jerk exceed the robot's capabilities |
| `safety_state` | `getSafetyState()` (robotInfo 2) | Raw ESafetyState code: 0 normal, 1–3 safety stop levels 0–2 |

Presence matters because `getCommandedJointPosition()` returns the SDK's decode buffer
from whichever packet last filled it, and `getTrackingPerformance()` returns 0.0 when
the field is absent. The capture record holds them in `signal_presence`
(`CommandedJointPosition`) and the metadata bits `Safety` / `Tracking`. Validation:
partial arrays are structural errors; nonfinite commanded positions, tracking
performance outside [0, 1] and unknown safety codes are kept and reported as issues.
The change is additive (`schema_version` stays 1). The Python readers
(`robot_sysid.data`, `robot_sysid.bundle`) also accept the pinned descriptor of the
18 October baseline below, so every recording made before this change still loads;
one file may not mix descriptors, and a run's manifest must name the descriptor its
MCAP carries. The native writer and readback accept only the current descriptor.

## Position and torque baseline — 18 October 2026

Every State carries seven measured positions in radians. Each of its three torque
signals independently contains zero values (unavailable) or seven values in Nm,
in A1..A7 order:

| Field | Intended live source | Meaning |
|---|---|---|
| `tau_measured_nm` | `getMeasuredTorque()` | Measured joint torque |
| `tau_commanded_nm` | `getCommandedTorque()` | Controller-commanded joint torque |
| `tau_external_estimated_nm` | `getExternalTorque()` | Controller estimate of external joint torque |

These are State measurements. Submission records retain the exact seven submitted
position values and their State association; they do not contain torque signals.
The fixed capture record uses independent `torque_presence` bits. Clear bits
exclude dormant array contents from serialization. Python exposes unavailable
torques as `None`; seven zeros remain an available reading. Nonfinite values are
preserved and reported per signal, making `data_valid=false`. Array lengths other
than zero or seven are structural errors. The position nonfinite mask continues
to describe positions only.

The manifest records `units: {"position": "rad", "torque": "Nm"}` and MCAP
channels carry `position_units=rad` and `torque_units=Nm`. This was a replacement
experimental baseline: earlier position-only recordings are unsupported, with no
migration path. (Its own descriptor stays readable after the 24 October addition.) The existing `acquisition_v1` names are retained for this prototype.

`test_capture_encoding --fixture <new-directory> <descriptor>` exercises the real
Recorder → conversion → MCAP path with one State lacking torques, one State with
all three distinct torque arrays, and one submitted position command. The Python
round-trip test verifies exact values (including signed zero), availability,
timestamps, units and association. The older `offline_fixture` remains an
additional synthetic example whose torque signals are unavailable.

## Build and verify

Requires CMake ≥3.22, C++17, Protobuf/protoc **3.21.12**, OpenSSL Crypto development
headers, Python **3.12**, and `uv` (or an equivalent isolated virtual environment).
OpenSSL supplies SHA-256 and random run IDs. The tested Linux versions and source
hashes are in the parent project's `reports/robot-sysid/acquisition-offline-format/`.
No global installation or ROS rebuild is needed. `COLCON_IGNORE` keeps this
standalone experiment out of normal colcon workspace discovery; use the explicit
CMake commands below.

From the PHD workspace root, choose an empty build directory:

```bash
acq_source="$PWD/packages/lbr-med-stack/src/lbr_acquisition"
acq_project="$PWD/03_projects/tactile-cable-sysid/tactile-cable-sysid"
acq_build="$(mktemp -d /tmp/lbr-acquisition.XXXXXX)"
acq_commit=00e47a1ba6901cbc0cf448cccebbc554c545c4fa
curl --fail --location "https://github.com/foxglove/mcap/archive/$acq_commit.tar.gz" \
  --output "$acq_build/mcap.tar.gz"
tar -xzf "$acq_build/mcap.tar.gz" -C "$acq_build"
cmake -S "$acq_source" -B "$acq_build" \
  -DMCAP_SOURCE_DIR="$acq_build/mcap-$acq_commit" -DBUILD_TESTING=ON
cmake --build "$acq_build" -j4
ctest --test-dir "$acq_build" --output-on-failure
uv venv --python python3.12 "$acq_build/venv"
uv pip sync --python "$acq_build/venv/bin/python" --require-hashes \
  "$acq_source/requirements-offline.lock"
PYTHONPATH="$acq_project/src:$acq_build/generated" \
LBR_OFFLINE_FIXTURE="$acq_build/offline_fixture" \
LBR_OFFLINE_DESCRIPTOR="$acq_build/generated/acquisition_v1.desc" \
  "$acq_build/venv/bin/python" -m pytest "$acq_project/tests/robot_sysid" -q
```

CMake verifies all pinned MCAP native-header hashes, rejects vendor 1.3.1, and
produces C++, Python and the descriptor from the same `.proto`. Generated files
stay in `generated/` under the build. The original descriptor bytes are embedded
unchanged. `requirements-offline.lock` pins the complete trial Python environment
with artifact hashes. Rebuilding against different versions is a new experiment.

## Write and read one fixture

```bash
"$acq_build/offline_fixture" --output "$acq_build/example-run" \
  --descriptor "$acq_build/generated/acquisition_v1.desc"
PYTHONPATH="$acq_project/src:$acq_build/generated" "$acq_build/venv/bin/python" -c \
  'from pathlib import Path; from robot_sysid.data import read_signals; import sys; print(read_signals(Path(sys.argv[1])))' \
  "$acq_build/example-run/signals.mcap"
```

The output directory must not exist. A successful run contains:

| File | Meaning |
|---|---|
| `manifest.json` | Synthetic source, schema, descriptor hash, run ID, clocks, units, dependency versions, limits |
| `signals.mcap` | Three measured-position states and two position submissions |
| `terminal.log` | Fixture stages and native readback diagnostics |
| `quality.json` | Separate container validity, data validity and finalization status |

The fixture includes an absent submission, absent clock versus present zero,
negative zero, distinct values for all seven joints, and timestamps above 2^53.
These are synthetic fixtures, not hardware measurements or an SDK timing test.
The original `offline_fixture` does not create `events.jsonl`. The 2B.4 session
does create recorder-owned `events.jsonl` and `session.log`. The manifest states
that the storage writer does not emit `events.jsonl`, but session diagnostics
may provide it. This is a scope description, not a file inventory. Capture
external build/test terminal output separately; `terminal.log` is only the
fixture's own diagnostic stream.

`write_fixture` is an allocating offline API. It must never run inside the future
capture callback. Its `FileOps` overload enables real write/close fault tests.
I/O retries EINTR and successful short writes, rejects zero progress, and closes
at most once. MCAP exceptions call `terminate()` before destruction to prevent a
second write during cleanup. Partial artifacts are retained on failure.

Both `signals.mcap` and `quality.json` use `.partial` while being written. A
checked close and independent native readback precede the signals rename; a
checked close precedes the quality rename. No fsync/power-loss durability claim
is made. A finalized signal container alone never establishes a completed run:
require valid `quality.json` with `finalization=complete` and the matching manifest.
Correctly flagged nonfinite measurements remain readable with `data_valid=false`.
Timing and SysID suitability remain `not_evaluated` even on success.

Python `read_signals(path)` is strict and bounded. `recover_prefix=True` returns
only validated records before the first corruption/truncation, with
`container_complete=False`; it never repairs or changes the original file.
Unsupported schemas and resource-limit violations always raise. This reader
accepts only the uncompressed, two-topic offline profile, not arbitrary MCAP.

The installed Jazzy MCAP vendor is left intact. This standalone result does not
establish that different native MCAP versions may share one ROS process safely.


## Bounded capture core (milestone 2A)

`acquisition_capture` is a separate C++ library with no Protobuf, MCAP or ROS link
dependency. It uses the reviewed Boost 1.83 SPSC header (verified by CMake).
The full standalone build still needs the dependencies listed above. It is an
offline, single-use core tested with fake producers/sinks; milestone 2B will add
owned workers, file/bundle integration and terminal-output capture. The existing
`offline_fixture` remains the file-format experiment; it does not use this core.

Headers: `lbr_acquisition/records.hpp` and `lbr_acquisition/recorder.hpp`.
`capture::Event` owns seven position doubles, three seven-double torque arrays,
and separate torque/state-metadata presence masks. `capture::Recorder` takes an explicit Config: run ID, duration in ns,
first-sample timeout in ns, maximum events and usable queue capacity. No production
capacity or duration default is chosen. Queue capacity is capped at 10000; the total event budget is explicit per run,
independent of queue capacity; internal queue storage includes Boost's extra slot.

Call ownership is part of the API contract:

| Caller | Operations | Restrictions |
|---|---|---|
| One producer | `offer(event, eligible)` | One attempt; copies fixed values; never retries or calls sink |
| One writer | `writer_step(sink)` | At most one record or one finish per call; sink may block/throw |
| Serialized coordinator | `start(run_id, now)`, `stop(run_id)`, `tick(now)` | Supplies host monotonic time; tick independently of producer/writer |
| Observer | `status()` | Atomic observations; live counters are not one coherent snapshot |

Prepare the Recorder and sink before starting the producer. The core creates no
threads/timer and cannot enforce expiry if the coordinator is not scheduled.
Capture duration starts at the first admitted State, independently of first-sample
wait timeout. A repeated start preserves the original timing; stop before start
prevents later start. The run is never reset/reused. Stop closes admission while
allowing an already admitted offer to finish; finalization waits for that offer.
External owners must stop/join all roles and prevent future calls before deleting
any endpoint or sink. Internal quiescence alone is not permission to free memory.

A queue overflow rejects one attempted record and stops admission. A throwing
sink write may already have side effects: it contributes to `write_uncertain`,
remaining queued records become `abandoned`, and writes are never retried.
Terminal accounting is `attempted = enqueued + rejected` and
`enqueued = written + abandoned + write_uncertain`. Stale/inactive/ineligible
calls do not count as attempted records. At the configured event limit, the next
eligible offer rejects and ends the run with an explicit failure.

`Complete` means only that the supplied sink's `finish()` returned and no
material failure flags were set. For each material Reason r, `failure_flags`
uses bit `1u << static_cast<uint32_t>(r)`; None, Manual and Duration set no bit. It says
nothing about bytes on disk, bundle finalization, power-loss durability, requested
duration achievement or SysID quality. A stalled sink leaves `writer_done=false`;
status/stop remain available, and the core never cancels/detaches its owner.
Numerical metadata anomalies remain raw for downstream validation; this core
checks event IDs, association and monotonic host event time, not the full storage
schema. Exactly one producer is mandatory; upstream command threads must not
share this SPSC producer role.

CTest includes `recorder` for lifecycle/capacity/fault behavior and
`recorder_concurrency` for deterministic interleavings, blocked sinks, ordered
concurrency and allocation checks. The latter compiles a **separate** test-only
core with barrier seams; the shipped `acquisition_capture` target has no test
hook symbol or calls. Tests do not establish deadline behavior or live FRI safety.

`recorder_threads` runs the non-barrier thread/allocation tests against the
production library, avoiding test-hook synchronization in those checks.


## Fixed-record conversion (2B.1)

`capture_encoding.hpp` provides `encode_capture_batch(events, expected_run_id)`:
a bounded, allocating offline conversion from fixed capture records to existing
Protobuf payloads. It preserves order, all seven values, integer timestamps and
optional availability; it rejects mixed runs, unsupported fields and stale
submission associations. It never belongs in a capture callback. Numeric capture
run ID is checked but not persisted by this converter; the synthetic fixture
manifest has its own UUID. This is not a streaming sink or live recorder.

`capture_encoding` and the Python `test_capture_encoding.py` exercise a fake
Recorder through conversion, the existing MCAP writer, and independent readback.
Persistent worker, terminal-log and full run-bundle integration remain later steps.

### Synthetic batch sink (2B.2)

`acquisition_batch_sink` provides `OfflineBatchWriter(run_id, new_dir,
descriptor, io)` for `Recorder::writer_step`. It stages at most 10,000 fixed
records, then converts and writes the existing synthetic fixture in `finish()`.
All methods require serialized writer-side access; `FileOps` outlives the sink.
Copy/move are disabled. The destructor never writes files. Failed/finalized sinks
reject further calls; errors propagate to the recorder's existing failure path.

This target is **not a live dataset path**. `Recorder::written` counts staging
acknowledgments, not disk persistence; abnormal exit before finish loses the RAM
batch. The const `staged_events()` view retains acknowledged records for inspection
after failure. Conversion faults create no output directory. Disk errors preserve
stage-specific artifacts: a late quality failure can leave `signals.mcap` alongside
failed quality diagnostics. There is no automatic retry or overwrite.

A capture overflow may yield core `Failed` and sink `Finalized`: the MCAP contains
the admitted prefix, with format-only `container_complete` / `data_valid` and
`finalization: complete`. Those fields do not establish capture success. The
fixture manifest remains `offline-fixture-v1`, `source: synthetic`; its random
run UUID is not the numeric capture run ID. Persisted recorder outcome, bundle
identity mapping, logging integration, streaming and live FRI remain later steps.

The new native `offline_batch_writer` test covers completion, empty batches,
retained copies, limits, poisoned writers, overflow, short writes, disk/close
faults, no-overwrite and destructor behavior. Existing Python readback is unchanged.

### Terminal outcome sidecar (2B.3)

After all producer, coordinator and writer calls have stopped, take a terminal
`Recorder::status()` snapshot and call `write_capture_outcome(sink, status, io)`.
The caller must observe `writer_done=true`; the helper does not own or join
threads. It rejects inconsistent terminal accounting and sink state, verifies a
present final MCAP against staged events, and publishes a sibling
`<run-directory>.capture.json`. The sibling location records failures that occur
before the batch sink creates the run directory. It uses exclusive `.partial`
creation, checked writes and close, and Linux `renameat2(RENAME_NOREPLACE)`;
existing sidecars are never overwritten. A failed publication leaves the partial
sidecar for diagnosis. No destructor writes or fsync are performed.

The sidecar holds the decimal string capture run ID, terminal reason, flags and
counters, plus SHA-256 and byte sizes for present final manifest, quality and
signals files. The manifest's 32 lowercase hex synthetic ID remains distinct
from the numeric capture ID. `written` still counts RAM staging acknowledgments;
the sidecar only describes the outcome observed after draining. A sidecar can
record core Failed even when the saved MCAP and quality file are complete.

`robot_sysid.bundle.read_capture_bundle(run_dir)` verifies the sidecar and file
bindings with strict size, schema and identity checks. It reports
`capture_complete`, `container_complete`, `data_valid` and `bundle_success`
separately. The last value requires nonempty State evidence, no rejected or
uncertain events, valid quality and clean readback. None of these values imply
timing acceptance or suitability for SysID. The earlier `read_signals` API remains
available for standalone MCAP inspection; a missing sidecar cannot be promoted
to a successful capture bundle.

### Offline streaming session (2B.4)

`StreamingCaptureWriter` prepares a fresh run directory outside the producer,
appends each accepted event through the shared incremental MCAP engine, and
closes its chunk before returning from `write()`. A returned write means the
checked sink accepted that chunk into kernel-visible output. It does not mean
fsync or power-loss durability. The live writer retains a fixed 64-byte `CaptureSummary` (counts, first/last state
clocks and a SHA-256 hash chain), rather than retaining acknowledged records.
The chain covers each exact encoded payload, kind, host timestamp and length in
order. Native finalization scans one MCAP record at a time (at most 1 MiB), checks
chunk/data CRCs and the complete expected summary, then hashes artifacts in
64 KiB blocks. Live state association uses the latest state ID; repeated quality
warnings are stored once per category. Recording and native verification memory
therefore do not grow with duration. Verification remains linear in file size.
The separate synthetic batch fixture still retains its bounded input vector.
At the original 2B.4 checkpoint, the batch and fixture APIs, payload schema,
manifest fields, quality format and Python readers were preserved. The later
torque-baseline changes are documented above. An exception after some bytes reach the
partial file leaves the event uncertain: the acknowledgement summary excludes it, the sink is
poisoned, and the partial file is never promoted or retried.
Final `signals.mcap` and `quality.json` publication refuses to replace an
existing destination; a collision leaves the `.partial` file for inspection.

`StreamingCaptureSession` owns one writer thread and a separate monotonic
coordinator thread. Construct it with explicit positive poll and writer-stall
periods, a `capture::Config`, a new output directory, descriptor and `FileOps`
that outlives the session. `start()` returns after the core is armed; the single
fake producer timestamps events with `now_ns()` and calls `offer()` without
logging, file I/O, allocation, locks or notification. `request_stop(run_id)`
only posts an atomic request; the coordinator alone calls the core's
start/stop/tick while threads exist. The first accepted stop reason wins even
when callers race. The producer owner must stop and join its producer before
calling session `join()` or destroying the session. `join()` waits for both
workers, writes diagnostics, then publishes the capture sidecar
if the core and session can truthfully support it. A blocked OS write or finish
can make join block; the independent timer and status remain responsive, and no
thread is forcibly canceled or detached. Repeated start/stop/join do not reopen
or republish a run. Destruction stops and joins workers without publishing a
success sidecar.

The coordinator records at most 32 observed phase/fault snapshots with native
monotonic timestamps. After workers join, checked, exclusive writes publish
`events.jsonl` and `session.log`. These files describe this synthetic recorder
session only. They do not contain arbitrary process stdout/stderr, previous
terminal history or ROS/FRI logs, and they are outside the legacy manifest and
sidecar artifact map. A diagnostic write/close/collision is a runtime failure
and suppresses the outcome sidecar even if the core itself reached Complete.
A writer stall while acquisition is open closes admission with WriterFailure;
a slow drain or finish after a stop is diagnostic only. In both cases, join waits
for the actual result and can publish its outcome once the writer returns.
Thrown writes/finish still fail the capture. `session.log` records `stalled` and
`runtime_failed` at diagnostic publication. Status distinguishes diagnostic
failure from outcome-publication failure (which can happen after that log).
For this session, `written` means a closed MCAP chunk in a kernel-visible file,
without fsync; before successful finalization that file is `signals.mcap.partial`.
The batch sink's `written` instead counts RAM staging. The legacy sidecar does
not encode sink type, so interpret the counter using the producing API; it alone
never establishes a finalized file, power-loss durability, or capture success. A recoverable MCAP prefix remains an
incomplete artifact; use `read_signals(..., recover_prefix=True)` for inspection.
This is an offline behavior test, not a qualified live acquisition path.

### Configurable live duration (2026-10-19)

`LBR_RECORDING_DIRECTORY` enables a one-shot capture in a new absolute directory.
`LBR_RECORDING_DURATION_SECONDS` selects positive decimal seconds, with up to nine
fractional digits; unset defaults to `1`. Examples: `10`, `60.25`, `.5`. There is
no fixed duration ceiling; values must fit the monotonic nanosecond clock and
checked storage-budget arithmetic. Invalid values fail during setup, before UDP
opens. Time starts at the first eligible state. The initial wait remains 60 s.

The live queue stays at 4096 records. The total admission budget is two records
per 1 ms cycle rounded up, plus 4096 records of headroom. File/event verification
budgets derive from that total and are recorded in the manifest and `build.txt`;
these ceilings do not allocate RAM or reserve disk. Disk-full, source interruption,
writer failure/stall, and queue overflow still produce explicit failed outcomes.
This supports a selected finite duration, not an indefinite run-until-stop mode.

Python `read_capture_bundle(path)` uses the declared budgets and streams artifact
hashing. Its signal analysis result intentionally materializes the records in RAM;
optional `max_file_bytes` and `max_events` arguments limit that analysis. Standalone
`read_signals` accepts those budgets explicitly and otherwise retains the small
fixture defaults. Long-run analysis memory is separate from live recorder memory.
Use the startup guide and involve the operator before connecting or capturing.

# OpenArm 2.0 — Data Collection Pipeline

A data-collection platform for the OpenArm 2.0 bimanual arm: read joint
telemetry over CAN-FD, capture synchronized frames from four cameras, store
episodes in a structured robotics format, and drive it all from a live web
dashboard with a REST API.

**This is the software-only build.** I don't have the hardware, so the CAN bus
and cameras are mocked — but the mock is faithful to the real system (real
Damiao frame layout, real motor lineup, real CAN IDs, real CLI commands), so the
parsing, synchronization, storage, and serving code is the same code that would
run against hardware. Where something is simulated, it's called out explicitly
both here and in the code.

Built against the OpenArm docs (`docs.openarm.dev`) and the provided
hardware/API reference.

![Dashboard](docs/dashboard.png)

*Live dashboard: joint telemetry for both arms, four camera feeds, per-sensor sync skew, episode list, and recording control.*

---

## What's implemented

| # | Task | Status | Notes |
|---|------|--------|-------|
| 1 | CAN interface setup | ✅ mock | `openarm-can-cli ... can_configure` + zero-position calibration, faithful terminal output |
| 2 | CAN data reading | ✅ mock | Live position/velocity/torque/temp via the real Damiao feedback-frame codec |
| 3 | Multi-camera sync | ✅ mock | 4 cameras at native rates, frame-driven nearest-neighbour alignment to joint state |
| 4 | Storage + REST API | ✅ | MCAP episodes + **export to OpenArm's native dataset format**; list / metadata / download endpoints |
| 5 | Monitoring dashboard | ✅ | Live joint bars, camera feeds, sync health, episode list, Start/Stop |

All five tasks run end-to-end in software. I leaned into 3–5 (the rubric says
that's equally strong to 1–2 on hardware) while keeping 1–2 as faithful to the
real protocol as I could without a CAN adapter.

---

## Quickstart

```bash
python -m venv .venv && source .venv/bin/activate
pip install -e .

make can       # Task 1: bring up can0/can1 (CAN-FD 1M/5M) + zero position
make monitor   # Task 2: live joint telemetry sample
make demo      # Tasks 2–4: record a 3s episode to MCAP, read it back
make export    # record an episode, then export to OpenArm's native dataset format
make serve     # Tasks 4–5: dashboard + API at http://localhost:8000
make test      # codec round-trip + synchronizer tests
```

`make serve` then open `http://localhost:8000` and hit **Start recording**.

---

## Architecture

```
                 ┌─────────────────── Recorder (orchestrator) ───────────────────┐
  CAN-FD         │                                                               │
  can0 ─┐        │   MockCANBus(right) ─┐                                         │
        ├─ Damiao │   MockCANBus(left)  ─┼─► joint loop (500 Hz) ─┐               │
  can1 ─┘  codec  │                      │                        ▼               │
                 │   MockCamera ×4 ──────┼─► frame buffers ──► FrameSynchronizer  │
  4 cameras      │   (60/60/30/30 fps)   │      (per-sensor    nearest-neighbour  │
  (threaded      │                       │       ring buffers)  anchored to the   │
   JPEG encode)  │                       │                      slowest camera)   │
                 │                       │                        │ SyncedBundle  │
                 │                       │                        ▼               │
                 │                       └──────────────► EpisodeRecorder (MCAP)  │
                 └───────────────────────────────────────────────┬───────────────┘
                                                                  │
                          FastAPI ── REST (list/meta/download) ───┤
                                  ├─ WebSocket (10 Hz live state) ─┤
                                  └─ dashboard (HTML/JS) ──────────┘
```

The sensors are behind interfaces (`CANBus`, `MockCamera`), the orchestrator
owns the live system and recording lifecycle, and the API/dashboard only ever
read `live_state()` or toggle recording. Swapping mock → hardware is a one-line
change in the bus factory.

```
src/openarm_pipeline/
  config.py            joints, motor limits, CAN IDs, camera config (edit here for hardware)
  can/damiao.py        Damiao feedback-frame + command-frame codec  ← real bit layout
  can/bus.py           CANBus interface, MockCANBus, SocketCANBus (hardware path)
  can/cli.py           Task 1: can_configure + zero-position mock
  cameras/camera.py    Task 3: mock cameras, threaded JPEG encode
  cameras/sync.py      Task 3: TimedBuffer + FrameSynchronizer
  storage/mcap_store.py Task 4: EpisodeRecorder + EpisodeStore (MCAP)
  storage/openarm_export.py  export a recorded episode to OpenArm's parquet dataset format
  recorder.py          orchestrator + live state
  api/server.py        Task 4/5: REST + WebSocket + dashboard
  api/static/dashboard.html
```

---

## Design decisions & trade-offs

### CAN parsing is real, not faked
`damiao.py` implements the actual 8-byte Damiao MIT-mode feedback layout
(position as uint16, velocity/torque as uint12, MOSFET/rotor temps, error nibble
+ CAN id). The mock bus *encodes* its synthetic motion into that layout and the
read path *decodes* it — so quantization and parsing are genuinely exercised, not
bypassed. Motor types, torque caps, and CAN IDs (J1–J7 → `0x01`–`0x07` /
`0x11`–`0x17`, gripper `0x08`/`0x18`) match the hardware table. The enable /
disable / set-zero command words match the docs (`FFFF…FC/FD/FE`), verified in
tests.

### Timestamping: monotonic at acquisition
Every sample is stamped with `time.monotonic()` at capture, not wall-clock, so an
NTP step can't corrupt the timeline. Wall-clock is recorded separately for
human-readable indexing. On hardware these would be promoted to driver/kernel SOF
timestamps and the ZED's onboard clock.

### Synchronization: frame-driven nearest-neighbour, anchored to the slowest camera
Rather than a free-running clock, each synchronized bundle is triggered by a new
frame from the **anchor** (slowest) camera, then every other sensor contributes
its nearest buffered sample. Consequences, measured on this machine:

- anchor camera skew: **0 ms** (by construction)
- joint state (500 Hz): **~0.4 ms** — effectively exact
- 60 Hz wrist cameras: **~3–6 ms** (within half a period)
- 30 Hz ZED: **~28 ms**, low variance — this is *phase*, not drift

That last point drove the degraded-detection design. Two free-running **same-rate**
cameras (30 Hz anchor + 30 Hz ZED) can sit up to one full frame period (~33 ms)
apart from clock phase alone; nearest-neighbour cannot do better without a shared
hardware trigger or PTP. So a bundle is flagged **degraded** only when a sensor is
more than one anchor period stale — i.e. a frame was genuinely dropped — rather
than using an arbitrarily tight threshold that would false-positive on every
frame. Degraded bundles are kept and marked, never silently paired with stale
data and never dropped.

Handling different frame rates: faster cameras simply have more candidate frames
to pick the nearest from; the anchor being the slowest guarantees every bundle
holds a fresh frame from the hardest-to-satisfy sensor. Images are never
interpolated (meaningless for pixels); joint state could be interpolated to the
tick but at 500 Hz it's already sub-millisecond, so it isn't worth it.

### Real-time awareness: encode off the event loop
The first cut ran JPEG encoding inline on the asyncio loop and the ZED fell
**>100 ms** behind. Moving the encode to worker threads (`asyncio.to_thread`;
PIL/numpy release the GIL during the C encode) brought every sensor back within
its frame period. Capture timestamps are taken *before* the encode, mirroring a
hardware SOF timestamp, and the capture loop drops rather than queues if it falls
behind, so latency can't spiral. Camera resolution is set to ~480p to match what
real teleop datasets (ALOHA, LeRobot) actually record — smaller frames keep
episodes trainable and storage sane.

### Storage: MCAP for recording, with export to OpenArm's native format
An episode is several sensor streams at different rates sharing one timeline.
That is exactly MCAP's job: timestamped messages per channel, indexed by log
time, schemas embedded in the file, append-friendly streaming writes, and a
direct path into Foxglove and rosbag2. So MCAP is the **live recording log**.

For data *at rest*, OpenArm has its own canonical format (the `openarm_dataset`
layout: a directory tree with `metadata.yaml` + `episodes/<id>/`, per-arm
`state.parquet` for qpos/qvel/qtorque, and `cameras/<name>/<unix_ns>.jpeg`), with
a built-in LeRobot v2.1 export. Rather than ignore it, the pipeline records to
MCAP and then **exports to the OpenArm format** via
`openarm_pipeline.storage.openarm_export` (`make export`). The produced tree
matches OpenArm's documented structure exactly — directory layout, nanosecond
filenames, joint names (`joint1…joint7`, `gripper`), camera stream names
(`wrist_left`, `wrist_right`, `ceiling`, `head`), and `metadata.yaml` fields — so
their `openarm-dataset-validate` / `openarm-dataset-convert` tools can pick it up
and convert to LeRobot for training.

This split is deliberate: MCAP is better while data is *streaming in*; the
parquet tree is better for *training and sharing*. HDF5/zarr would be a third
option for dense homogeneous arrays, but OpenArm already standardizes on the
parquet layout, so that's the export target. One honest caveat: the *internal
column convention* of `state.parquet` isn't given in OpenArm's public docs (it
lives in their loader), so the exporter uses an explicit, documented convention
and recommends running `openarm-dataset-validate` before production use. Live
MCAP layout: `/joint_states/{arm}` (json), `/camera/{name}` (jpeg), `/sync`
(json) + an episode metadata record.

### API design
Thin and resource-oriented: `GET /api/episodes`, `GET /api/episodes/{id}`,
`GET /api/episodes/{id}/download`, `POST /api/recording/{start,stop}`,
`GET /api/state`, `GET /api/cameras/{name}/preview`, and `WS /ws` for live state.
Downloads stream the raw `.mcap`; metadata is read back from the file's own MCAP
summary so it can't drift from the data. Missing episodes return 404.

---

## What's mocked (so it's clear)

- **CAN bus** — no adapter, so `MockCANBus` synthesizes plausible joint motion and
  round-trips it through the real Damiao codec. `SocketCANBus` (the python-can
  hardware path) is written and inert.
- **Cameras** — synthetic frames with on-frame metadata and a moving bar (so sync
  is visible to the eye), encoded as real JPEGs at the cameras' native rates.
- **`can_configure` / zero calibration** — prints the real commands and their
  expected output against the mock bus; no `ip link` is actually issued.

Everything downstream of the sensor interfaces — synchronization, storage, REST,
WebSocket, dashboard — is the real implementation.

---

## What I'd do next (with hardware / more time)

1. **Hardware bring-up.** Wire `SocketCANBus` to a real adapter, validate the
   Damiao decode against `candump`, and confirm the CAN-ID table per arm. Most of
   the risk is in motor enable/timeout tuning, which the docs flag.
2. **True time sync.** Hardware-triggered camera capture or PTP across cameras to
   collapse the ~one-period phase skew between same-rate cameras; promote
   software timestamps to kernel/driver SOF and the ZED onboard clock.
3. **ZED depth.** Record the depth/disparity stream alongside the stereo pair,
   and calibrate camera extrinsics into the arm frame.
4. **Backpressure & integrity.** Bounded queues with explicit drop accounting per
   stream, plus per-episode checksums and a post-record validation pass.
5. **Training-ready export.** The MCAP → OpenArm parquet export exists; the next
   steps are recording the full-rate (500 Hz) joint stream as a separate channel
   so exported obs aren't downsampled to the 30 Hz bundle grid, splitting the
   stereo `head` into `head_left`/`head_right`, and validating the parquet schema
   against `openarm-dataset-validate` end to end.
6. **Dashboard depth.** Per-joint torque/temperature history, drift/skew charts
   over an episode, and frame-accurate scrubbing via the MCAP time index.
7. **Hardening.** Auth on the API, disk-space guards, graceful recovery if a
   sensor stalls mid-episode, and integration tests against recorded fixtures.

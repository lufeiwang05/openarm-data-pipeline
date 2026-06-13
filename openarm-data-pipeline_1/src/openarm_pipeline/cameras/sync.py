"""Multi-sensor synchronization (task 3, alignment side).

The core problem: four cameras at 30/60 fps and a 500 Hz joint stream all run on
independent clocks and arrive at different instants. We need, for each recorded
step, one bundle that pairs every camera's frame with the joint state at the
same moment.

Strategy — software timestamp + nearest-neighbour against a master tick:

  1. Every sample (frame or joint snapshot) is stamped with `time.monotonic()`
     at acquisition. Monotonic, not wall-clock, so NTP steps can't dent the
     timeline. (On real hardware you'd promote this to the driver/kernel
     SOF/hardware timestamp and, for the ZED, its onboard clock — see README.)

  2. The recorder emits a master tick at SYNC_HZ, chosen to equal the *slowest*
     camera (30 Hz). At each tick we pick, per sensor, the buffered sample whose
     timestamp is closest to the tick time.

  3. Each match carries its |Δt|. A bundle is flagged degraded only if some
     sensor's nearest sample is more than one anchor period away — meaning a
     frame was genuinely dropped, not merely out of phase. We keep degraded
     bundles (marked) rather than discarding data.

Why nearest-neighbour and not interpolation: images can't be interpolated
meaningfully, and anchoring the tick to the slowest camera means every bundle
contains a *real* fresh frame from that device (its own skew is 0). Joint state,
sampled ~16x faster than the tick, lands within ~1 ms of the tick, so arm pose
is effectively exact. The fast wrist cameras (60 Hz) fall within half their
period. The catch is two *same-rate* free-running cameras (the 30 Hz anchor and
the 30 Hz ZED): without a common hardware trigger or PTP time sync they can sit
up to a full frame period apart from clock phase alone — that is the floor of
software-timestamp sync, and why the degraded threshold is one period rather
than something tighter. Hardware-triggered capture or PTP is the real fix and is
noted in the README as future work.
"""

from __future__ import annotations

import bisect
from dataclasses import dataclass, field

from openarm_pipeline.can.bus import JointState
from openarm_pipeline.cameras.camera import Frame
from openarm_pipeline.config import SYNC_TOLERANCE_MS


class TimedBuffer:
    """Small ring buffer keyed by monotonic timestamp, with nearest lookup."""

    def __init__(self, maxlen: int = 240):
        self._maxlen = maxlen
        self._ts: list[float] = []
        self._items: list = []

    def push(self, t: float, item) -> None:
        self._ts.append(t)
        self._items.append(item)
        if len(self._ts) > self._maxlen:
            self._ts.pop(0)
            self._items.pop(0)

    def nearest(self, t: float):
        """Return (item, dt_seconds) closest to t, or (None, inf) if empty."""
        if not self._ts:
            return None, float("inf")
        i = bisect.bisect_left(self._ts, t)
        best_i, best_dt = None, float("inf")
        for cand in (i - 1, i, i + 1):
            if 0 <= cand < len(self._ts):
                dt = abs(self._ts[cand] - t)
                if dt < best_dt:
                    best_i, best_dt = cand, dt
        return self._items[best_i], best_dt


@dataclass
class SyncedBundle:
    """One synchronized recording step."""

    step: int
    t_tick: float
    joints: dict[str, JointState]          # by arm name
    frames: dict[str, Frame]               # by camera name
    max_skew_ms: float                     # worst |Δt| in this bundle
    degraded: bool                         # any sensor beyond tolerance
    per_sensor_skew_ms: dict[str, float] = field(default_factory=dict)


class FrameSynchronizer:
    def __init__(self, arm_names: list[str], camera_names: list[str]):
        self._joint_bufs = {a: TimedBuffer() for a in arm_names}
        self._frame_bufs = {c: TimedBuffer() for c in camera_names}
        self._step = 0

    def add_joint(self, state: JointState) -> None:
        self._joint_bufs[state.arm].push(state.t_mono, state)

    def add_frame(self, frame: Frame) -> None:
        self._frame_bufs[frame.camera].push(frame.t_mono, frame)

    def sample(self, t_tick: float) -> SyncedBundle:
        """Build the synchronized bundle nearest to t_tick."""
        joints, frames, skews = {}, {}, {}

        for arm, buf in self._joint_bufs.items():
            item, dt = buf.nearest(t_tick)
            if item is not None:
                joints[arm] = item
                skews[f"joints:{arm}"] = dt * 1000.0

        for cam, buf in self._frame_bufs.items():
            item, dt = buf.nearest(t_tick)
            if item is not None:
                frames[cam] = item
                skews[f"cam:{cam}"] = dt * 1000.0

        max_skew = max(skews.values(), default=0.0)
        bundle = SyncedBundle(
            step=self._step,
            t_tick=t_tick,
            joints=joints,
            frames=frames,
            max_skew_ms=max_skew,
            degraded=max_skew > SYNC_TOLERANCE_MS,
            per_sensor_skew_ms=skews,
        )
        self._step += 1
        return bundle

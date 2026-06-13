"""Recorder orchestrator.

Owns the live system: per-arm CAN buses, all cameras, the synchronizer, and
(when armed) an MCAP recorder. Runs three kinds of async work:

  * camera capture loops    — each at its native fps (inside MockCamera)
  * joint polling loop      — JOINT_STREAM_HZ, feeds the synchronizer + live UI
  * sync tick loop          — SYNC_HZ, builds bundles; writes them iff recording

The dashboard/API never touch the sensors directly — they read `live_state()`
and toggle `start_recording()` / `stop_recording()`.
"""

from __future__ import annotations

import asyncio
import base64
import time
from datetime import datetime, timezone

from openarm_pipeline.can.bus import make_bus
from openarm_pipeline.cameras.camera import MockCamera
from openarm_pipeline.cameras.sync import FrameSynchronizer
from openarm_pipeline.config import (
    ARMS,
    CAMERAS,
    ANCHOR_CAMERA,
    JOINT_STREAM_HZ,
    SYNC_HZ,
)
from openarm_pipeline.storage.mcap_store import EpisodeRecorder, EpisodeStore


class Recorder:
    def __init__(self, mock: bool = True):
        self.mock = mock
        self.buses = {
            arm.name: make_bus(arm, mock=mock, seed_phase=i)
            for i, arm in enumerate(ARMS)
        }
        self.cameras = {c.name: MockCamera(c) for c in CAMERAS}
        self.sync = FrameSynchronizer(
            arm_names=list(self.buses), camera_names=list(self.cameras)
        )
        self.store = EpisodeStore()

        self._tasks: list[asyncio.Task] = []
        self._running = False
        self._recorder: EpisodeRecorder | None = None
        self._last_bundle = None
        self._latest_joints: dict = {}

    # ----- lifecycle ----------------------------------------------------- #
    async def start(self) -> None:
        for bus in self.buses.values():
            await bus.start()
        for cam in self.cameras.values():
            await cam.start()
        self._running = True
        self._tasks = [
            asyncio.create_task(self._joint_loop()),
            asyncio.create_task(self._sync_loop()),
        ]

    async def stop(self) -> None:
        self._running = False
        if self._recorder is not None:
            self.stop_recording()
        for t in self._tasks:
            t.cancel()
        await asyncio.gather(*self._tasks, return_exceptions=True)
        for cam in self.cameras.values():
            await cam.stop()
        for bus in self.buses.values():
            await bus.stop()

    # ----- background loops ---------------------------------------------- #
    async def _joint_loop(self) -> None:
        period = 1.0 / JOINT_STREAM_HZ
        while self._running:
            for name, bus in self.buses.items():
                state = await bus.read()
                self.sync.add_joint(state)
                self._latest_joints[name] = state
            await asyncio.sleep(period)

    async def _sync_loop(self) -> None:
        """Frame-driven synchronization.

        We poll faster than any camera, push each device's newest frame into the
        synchronizer buffers (deduped by sequence), and emit one synchronized
        bundle whenever the *anchor* camera (the slowest, ANCHOR_CAMERA) produces
        a new frame. Anchoring to real frame arrivals means the anchor's skew is
        0 by construction and every bundle holds a genuinely fresh frame from the
        slowest sensor — so a "degraded" flag now means a real dropped frame, not
        clock phase.
        """
        last_seq: dict[str, int] = {c: -1 for c in self.cameras}
        anchor = ANCHOR_CAMERA
        while self._running:
            for name, cam in self.cameras.items():
                fr = cam.latest
                if fr is not None and fr.seq != last_seq[name]:
                    last_seq[name] = fr.seq
                    self.sync.add_frame(fr)
                    if name == anchor:
                        bundle = self.sync.sample(fr.t_mono)
                        self._last_bundle = bundle
                        if self._recorder is not None:
                            self._recorder.write(bundle)
            await asyncio.sleep(1.0 / (SYNC_HZ * 8))

    # ----- recording control --------------------------------------------- #
    def start_recording(self, notes: str = "") -> dict:
        if self._recorder is not None:
            return {"status": "already_recording", "episode_id": self._recorder.episode_id}
        episode_id = datetime.now(timezone.utc).strftime("ep_%Y%m%dT%H%M%SZ")
        self._recorder = EpisodeRecorder(episode_id, notes=notes)
        return {"status": "recording", "episode_id": episode_id}

    def stop_recording(self) -> dict:
        if self._recorder is None:
            return {"status": "idle"}
        meta = self._recorder.close()
        self._recorder = None
        return {"status": "stopped", **meta}

    # ----- live state for the dashboard ---------------------------------- #
    def live_state(self, include_frames: bool = False) -> dict:
        joints = {
            arm: state.as_dict()["joints"] for arm, state in self._latest_joints.items()
        }
        b = self._last_bundle
        sync_health = {
            "max_skew_ms": round(b.max_skew_ms, 3) if b else None,
            "degraded": b.degraded if b else None,
            "per_sensor_skew_ms": (
                {k: round(v, 3) for k, v in b.per_sensor_skew_ms.items()} if b else {}
            ),
        }
        state = {
            "t": time.time(),
            "recording": self._recorder is not None,
            "episode_id": self._recorder.episode_id if self._recorder else None,
            "recorded_steps": self._recorder.steps if self._recorder else 0,
            "episode_count": len(self.store.list_ids()),
            "joints": joints,
            "sync": sync_health,
            "cameras": [c for c in self.cameras],
        }
        if include_frames and b is not None:
            state["frames"] = {
                cam: "data:image/jpeg;base64," + base64.b64encode(fr.jpeg).decode()
                for cam, fr in b.frames.items()
            }
        return state

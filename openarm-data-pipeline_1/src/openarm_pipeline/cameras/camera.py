"""Mock cameras (task 3, capture side).

Each camera runs its own async capture loop at its native frame rate and stamps
every frame with a monotonic capture time. Frames are synthetic but carry the
metadata a real Arducam/ZED frame would (device, seq, timestamp) and are
JPEG-encoded, so storage and dashboard handle the exact byte type hardware
produces.

Real-time note: JPEG encoding is CPU-bound, so doing it inline on the asyncio
event loop starves the other capture loops and the joint poller (we measured the
ZED falling >100 ms behind that way). Real camera stacks deliver frames on their
own threads; we mirror that by offloading the encode to a worker thread
(`asyncio.to_thread`) — PIL/numpy release the GIL during the C encode, so the
loops actually run concurrently and each camera holds its rate. The frame is
timestamped at the *intended* capture instant, before the encode, which is what
a hardware SOF timestamp represents.

The ZED head is one device emitting a single side-by-side image (left|right):
the two eyes share a timestamp by construction, exactly like a real stereo pair.
"""

from __future__ import annotations

import asyncio
import io
import time
from dataclasses import dataclass

import numpy as np
from PIL import Image, ImageDraw

from openarm_pipeline.config import CameraConfig

_BASE_COLORS = {
    "wrist_left": (20, 30, 60),
    "wrist_right": (60, 30, 20),
    "ceiling": (20, 50, 40),
    "head": (40, 40, 20),
}


@dataclass
class Frame:
    camera: str
    seq: int
    t_mono: float        # monotonic capture instant (pre-encode)
    t_wall: float
    width: int
    height: int
    jpeg: bytes


class MockCamera:
    def __init__(self, cfg: CameraConfig):
        self.cfg = cfg
        self._seq = 0
        self._latest: Frame | None = None
        self._task: asyncio.Task | None = None
        self._running = False
        # Precompute the static background once (cheap per-frame copies after).
        w = cfg.width * (2 if cfg.stereo else 1)
        base = np.zeros((cfg.height, w, 3), dtype=np.uint8)
        base[:, :] = _BASE_COLORS.get(cfg.name, (30, 30, 30))
        self._base = base

    @property
    def latest(self) -> Frame | None:
        return self._latest

    def _render(self, seq: int, t: float) -> bytes:
        cfg = self.cfg
        img = Image.fromarray(self._base.copy())
        draw = ImageDraw.Draw(img)

        def panel(x_off: int, eye: str | None) -> None:
            bar_x = int((t * 0.5 % 1.0) * (cfg.width - 8)) + x_off
            draw.rectangle([bar_x, 0, bar_x + 8, cfg.height], fill=(200, 200, 60))
            label = cfg.name + (f" [{eye}]" if eye else "")
            draw.text((x_off + 8, 8), label, fill=(235, 235, 235))
            draw.text((x_off + 8, 24), f"seq {seq}", fill=(180, 180, 180))
            draw.text((x_off + 8, 40), f"t {t:8.3f}s", fill=(180, 180, 180))

        if cfg.stereo:
            panel(0, "L")
            panel(cfg.width, "R")
            draw.line([cfg.width, 0, cfg.width, cfg.height], fill=(90, 90, 90), width=2)
        else:
            panel(0, None)

        buf = io.BytesIO()
        img.save(buf, format="JPEG", quality=60)
        return buf.getvalue()

    async def _loop(self) -> None:
        cfg = self.cfg
        period = 1.0 / cfg.fps
        next_t = time.monotonic()
        while self._running:
            capture_t = time.monotonic()   # stamp at intended capture instant
            seq = self._seq
            jpeg = await asyncio.to_thread(self._render, seq, capture_t)
            self._latest = Frame(
                camera=cfg.name,
                seq=seq,
                t_mono=capture_t,
                t_wall=time.time(),
                width=cfg.width * (2 if cfg.stereo else 1),
                height=cfg.height,
                jpeg=jpeg,
            )
            self._seq += 1
            next_t += period
            # If we fell behind, resync rather than spiral (drop, don't queue).
            sleep = next_t - time.monotonic()
            if sleep < -period:
                next_t = time.monotonic()
                sleep = 0.0
            await asyncio.sleep(max(0.0, sleep))

    async def start(self) -> None:
        self._running = True
        self._task = asyncio.create_task(self._loop())

    async def stop(self) -> None:
        self._running = False
        if self._task:
            await asyncio.gather(self._task, return_exceptions=True)

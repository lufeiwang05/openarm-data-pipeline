"""CAN-FD bus abstraction.

`CANBus` is the interface the rest of the pipeline talks to. There are two
implementations:

  * `MockCANBus`     — generates physically plausible joint motion and emits it
                       as real Damiao feedback frames (encoded via damiao.py),
                       so downstream code runs identical parsing logic to the
                       hardware path. This is what runs in software-only mode.

  * `SocketCANBus`   — reads real frames off a Linux SocketCAN interface using
                       python-can. Wired up but inert without hardware; kept in
                       the tree to show exactly where the seam is.

Both yield the same `JointState` snapshots, so swapping is a one-line change in
the recorder.
"""

from __future__ import annotations

import asyncio
import math
import time
from dataclasses import dataclass, field

from openarm_pipeline.can.damiao import JointFeedback, decode_feedback, encode_feedback
from openarm_pipeline.config import ArmConfig, JointConfig


@dataclass
class JointState:
    """A full snapshot of one arm at one instant, in engineering units."""

    arm: str
    interface: str
    t_mono: float                      # time.monotonic() at sample time
    t_wall: float                      # time.time() (wall clock) at sample time
    joints: dict[str, JointFeedback] = field(default_factory=dict)

    def as_dict(self) -> dict:
        return {
            "arm": self.arm,
            "interface": self.interface,
            "t_mono": self.t_mono,
            "t_wall": self.t_wall,
            "joints": {
                name: {
                    "position": fb.position,
                    "velocity": fb.velocity,
                    "torque": fb.torque,
                    "t_mos": fb.t_mos,
                    "t_rotor": fb.t_rotor,
                    "error": fb.error,
                }
                for name, fb in self.joints.items()
            },
        }


class CANBus:
    """Interface for a per-arm CAN-FD bus."""

    def __init__(self, arm: ArmConfig):
        self.arm = arm

    async def start(self) -> None: ...
    async def stop(self) -> None: ...

    async def read(self) -> JointState:
        """Return the most recent synchronized joint snapshot for this arm."""
        raise NotImplementedError

    def set_zero(self) -> None:
        """Latch the current pose as the zero position (task 1)."""
        raise NotImplementedError


class MockCANBus(CANBus):
    """Software arm. Each joint follows a slow sinusoid plus light noise, with
    velocity/torque derived consistently. The continuous signal is *encoded to
    Damiao frames and decoded back*, so quantization and parsing are real."""

    def __init__(self, arm: ArmConfig, seed_phase: float = 0.0):
        super().__init__(arm)
        self._t0 = time.monotonic()
        self._zero_offset: dict[str, float] = {j.name: 0.0 for j in arm.joints}
        # Per-joint motion params so the two arms don't move identically.
        self._phase = {
            j.name: seed_phase + 0.7 * i for i, j in enumerate(arm.joints)
        }
        self._amp = {j.name: 0.6 if j.name != "gripper" else 0.3 for j in arm.joints}
        self._freq = {
            j.name: 0.15 + 0.05 * i for i, j in enumerate(arm.joints)
        }
        self._running = False

    async def start(self) -> None:
        self._running = True

    async def stop(self) -> None:
        self._running = False

    def _model(self, j: JointConfig, t: float) -> tuple[float, float, float]:
        """Return (position, velocity, torque) for joint j at time t."""
        w = 2 * math.pi * self._freq[j.name]
        a = self._amp[j.name]
        ph = self._phase[j.name]
        pos = a * math.sin(w * t + ph) - self._zero_offset[j.name]
        vel = a * w * math.cos(w * t + ph)
        # Torque ~ gravity/inertia proxy: opposes acceleration plus a static term.
        acc = -a * w * w * math.sin(w * t + ph)
        torque = 0.05 * acc + 0.3 * math.sin(w * t + ph)
        return pos, vel, torque

    async def read(self) -> JointState:
        t = time.monotonic() - self._t0
        snapshot = JointState(
            arm=self.arm.name,
            interface=self.arm.interface,
            t_mono=time.monotonic(),
            t_wall=time.time(),
        )
        for j in self.arm.joints:
            pos, vel, torque = self._model(j, t)
            # Round-trip through the real wire format.
            payload = encode_feedback(
                can_id=j.recv_can_id & 0x0F,
                limits=j.limits,
                position=pos,
                velocity=vel,
                torque=torque,
                t_mos=40 + int(2 * math.sin(t)),
                t_rotor=45 + int(2 * math.sin(t + 1)),
            )
            snapshot.joints[j.name] = decode_feedback(payload, j.limits)
        return snapshot

    def set_zero(self) -> None:
        t = time.monotonic() - self._t0
        for j in self.arm.joints:
            pos, _, _ = self._model(j, t)
            # Fold current reading into the offset so "now" reads ~0.
            self._zero_offset[j.name] += pos


class SocketCANBus(CANBus):  # pragma: no cover - requires hardware
    """Real-hardware path over Linux SocketCAN via python-can.

    Left unexercised in software-only mode. The structure mirrors what the
    OpenArm C++ library does: open the FD-enabled interface, request feedback
    from each motor, decode with the same damiao.decode_feedback().
    """

    def __init__(self, arm: ArmConfig):
        super().__init__(arm)
        self._bus = None
        self._latest: dict[int, JointFeedback] = {}

    async def start(self) -> None:
        import can  # python-can; only needed on the hardware path

        self._bus = can.Bus(
            interface="socketcan",
            channel=self.arm.interface,
            fd=True,
        )

    async def stop(self) -> None:
        if self._bus is not None:
            self._bus.shutdown()

    async def read(self) -> JointState:
        import can

        snapshot = JointState(
            arm=self.arm.name,
            interface=self.arm.interface,
            t_mono=time.monotonic(),
            t_wall=time.time(),
        )
        by_recv = {j.recv_can_id: j for j in self.arm.joints}
        # Drain whatever feedback frames are buffered this tick.
        while True:
            msg = self._bus.recv(timeout=0.0)
            if msg is None:
                break
            j = by_recv.get(msg.arbitration_id)
            if j is not None:
                snapshot.joints[j.name] = decode_feedback(bytes(msg.data), j.limits)
        return snapshot

    def set_zero(self) -> None:
        # On real hardware this sends the Damiao "set zero" command frame
        # (cansend canX <id>##1FFFFFFFFFFFFFFFE) to every motor.
        raise NotImplementedError("set_zero requires hardware")


def make_bus(arm: ArmConfig, mock: bool = True, seed_phase: float = 0.0) -> CANBus:
    return MockCANBus(arm, seed_phase=seed_phase) if mock else SocketCANBus(arm)

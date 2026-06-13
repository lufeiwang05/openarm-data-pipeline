"""Tests for the two pieces with real correctness conditions: the Damiao frame
codec (quantization round-trip) and the synchronizer (nearest-neighbour pick).
Run with `pytest -q`.
"""

import math

from openarm_pipeline.can.damiao import (
    command_frame,
    decode_feedback,
    encode_feedback,
)
from openarm_pipeline.cameras.sync import TimedBuffer
from openarm_pipeline.config import MOTOR_TYPES


def test_feedback_roundtrip_within_quantization():
    limits = MOTOR_TYPES["DM4310"]
    for pos, vel, tau in [(0.0, 0.0, 0.0), (1.23, -4.5, 2.1), (-12.0, 29.0, -6.5)]:
        payload = encode_feedback(can_id=1, limits=limits,
                                  position=pos, velocity=vel, torque=tau)
        assert len(payload) == 8
        fb = decode_feedback(payload, limits)
        # 16-bit position -> sub-mrad; 12-bit vel/torque -> coarser.
        assert abs(fb.position - pos) < (2 * limits.p_max) / 2**16 + 1e-6
        assert abs(fb.velocity - vel) < (2 * limits.v_max) / 2**12 + 1e-6
        assert abs(fb.torque - tau) < (2 * limits.t_max) / 2**12 + 1e-6
        assert fb.healthy


def test_feedback_clamps_out_of_range():
    limits = MOTOR_TYPES["DM4310"]
    fb = decode_feedback(
        encode_feedback(1, limits, position=999.0, velocity=0, torque=0), limits
    )
    assert math.isclose(fb.position, limits.p_max, rel_tol=1e-3)


def test_can_id_and_error_survive():
    limits = MOTOR_TYPES["DM8009P"]
    payload = encode_feedback(can_id=7, limits=limits, position=0, velocity=0,
                              torque=0, error_code=0xA)  # overcurrent
    fb = decode_feedback(payload, limits)
    assert fb.can_id == 7
    assert fb.error == "overcurrent"
    assert not fb.healthy


def test_command_frames_match_docs():
    assert command_frame("enable").hex().upper() == "FFFFFFFFFFFFFFFC"
    assert command_frame("disable").hex().upper() == "FFFFFFFFFFFFFFFD"
    assert command_frame("set_zero").hex().upper() == "FFFFFFFFFFFFFFFE"


def test_timed_buffer_nearest():
    buf = TimedBuffer(maxlen=10)
    for t in [0.0, 0.10, 0.20, 0.30]:
        buf.push(t, f"f@{t:.2f}")
    item, dt = buf.nearest(0.16)
    assert item == "f@0.20"
    assert abs(dt - 0.04) < 1e-9


def test_timed_buffer_empty():
    item, dt = TimedBuffer().nearest(1.0)
    assert item is None and dt == float("inf")

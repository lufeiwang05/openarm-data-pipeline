"""Damiao DM-series motor frame codec.

OpenArm joints are Damiao DM-series actuators. In MIT/feedback mode each motor
emits an 8-byte feedback payload. Decoding it correctly is the whole point of
"CAN data reading", so we implement the real bit layout rather than inventing
our own — the mock producer (bus.py) encodes with the *same* functions, so the
round-trip exercises genuine parsing code.

Feedback frame layout (8 bytes, big-endian fields):

    byte 0 : [7:4] error code   [3:0] motor controller id
    byte 1 : position[15:8]
    byte 2 : position[7:0]
    byte 3 : velocity[11:4]
    byte 4 : [7:4] velocity[3:0]   [3:0] torque[11:8]
    byte 5 : torque[7:0]
    byte 6 : t_mos    (MOSFET temperature, °C)
    byte 7 : t_rotor  (rotor temperature, °C)

Position is a uint16 spanning [-P_MAX, +P_MAX]; velocity and torque are uint12
spanning [-V_MAX, +V_MAX] and [-T_MAX, +T_MAX]. The MAX values come from the
motor configuration (see config.MotorLimits).
"""

from __future__ import annotations

from dataclasses import dataclass

from openarm_pipeline.config import MotorLimits

# Error codes reported in the high nibble of byte 0 (subset that matters here).
ERROR_CODES = {
    0x0: "ok",
    0x8: "overvoltage",
    0x9: "undervoltage",
    0xA: "overcurrent",
    0xB: "mos_overtemp",
    0xC: "rotor_overtemp",
    0xD: "lost_comm",
    0xE: "overload",
}


# Special command payloads sent to a motor's *sender* CAN id (from the docs:
# `cansend can0 001#FFFFFFFFFFFFFFFC` enables motor 1, etc.). These are the
# fixed control words for the Damiao MIT protocol.
CMD_ENABLE = bytes.fromhex("FFFFFFFFFFFFFFFC")
CMD_DISABLE = bytes.fromhex("FFFFFFFFFFFFFFFD")
CMD_SET_ZERO = bytes.fromhex("FFFFFFFFFFFFFFFE")
CMD_CLEAR_ERROR = bytes.fromhex("FFFFFFFFFFFFFFFB")


def command_frame(kind: str) -> bytes:
    """Return the 8-byte control word for enable/disable/set_zero/clear_error."""
    return {
        "enable": CMD_ENABLE,
        "disable": CMD_DISABLE,
        "set_zero": CMD_SET_ZERO,
        "clear_error": CMD_CLEAR_ERROR,
    }[kind]


def _float_to_uint(x: float, x_min: float, x_max: float, bits: int) -> int:
    """Map a float in [x_min, x_max] to an unsigned int of `bits` width."""
    span = x_max - x_min
    x = max(x_min, min(x_max, x))
    return int((x - x_min) * ((1 << bits) - 1) / span)


def _uint_to_float(v: int, x_min: float, x_max: float, bits: int) -> float:
    """Inverse of _float_to_uint."""
    span = x_max - x_min
    return v * span / ((1 << bits) - 1) + x_min


@dataclass
class JointFeedback:
    """Decoded single-motor feedback."""

    can_id: int
    position: float       # rad
    velocity: float       # rad/s
    torque: float         # N·m
    t_mos: int            # °C
    t_rotor: int          # °C
    error: str

    @property
    def healthy(self) -> bool:
        return self.error == "ok"


def encode_feedback(
    can_id: int,
    limits: MotorLimits,
    position: float,
    velocity: float,
    torque: float,
    t_mos: int = 40,
    t_rotor: int = 45,
    error_code: int = 0x0,
) -> bytes:
    """Build an 8-byte Damiao feedback payload. Used by the mock motor."""
    p = _float_to_uint(position, -limits.p_max, limits.p_max, 16)
    v = _float_to_uint(velocity, -limits.v_max, limits.v_max, 12)
    t = _float_to_uint(torque, -limits.t_max, limits.t_max, 12)

    return bytes(
        [
            ((error_code & 0x0F) << 4) | (can_id & 0x0F),
            (p >> 8) & 0xFF,
            p & 0xFF,
            (v >> 4) & 0xFF,
            ((v & 0x0F) << 4) | ((t >> 8) & 0x0F),
            t & 0xFF,
            t_mos & 0xFF,
            t_rotor & 0xFF,
        ]
    )


def decode_feedback(payload: bytes, limits: MotorLimits) -> JointFeedback:
    """Parse an 8-byte Damiao feedback payload into engineering units."""
    if len(payload) < 8:
        raise ValueError(f"feedback frame too short: {len(payload)} bytes")

    error_code = (payload[0] >> 4) & 0x0F
    can_id = payload[0] & 0x0F

    p_raw = (payload[1] << 8) | payload[2]
    v_raw = (payload[3] << 4) | (payload[4] >> 4)
    t_raw = ((payload[4] & 0x0F) << 8) | payload[5]

    return JointFeedback(
        can_id=can_id,
        position=_uint_to_float(p_raw, -limits.p_max, limits.p_max, 16),
        velocity=_uint_to_float(v_raw, -limits.v_max, limits.v_max, 12),
        torque=_uint_to_float(t_raw, -limits.t_max, limits.t_max, 12),
        t_mos=payload[6],
        t_rotor=payload[7],
        error=ERROR_CODES.get(error_code, f"unknown_0x{error_code:X}"),
    )

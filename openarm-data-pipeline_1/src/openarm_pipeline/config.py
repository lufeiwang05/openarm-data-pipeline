"""Central configuration for the OpenArm 2.0 data-collection pipeline.

Values here describe the *physical* system we are modelling so that the mock
data stream is faithful to real hardware. They are derived from the OpenArm
docs (docs.openarm.dev) and the Damiao DM-series motor datasheets:

  * OpenArm 2.0 is bimanual: one CAN-FD bus per arm (can0 = right, can1 = left).
  * Each arm is 7-DOF, driven by Damiao DM-series motors, plus a 1-DOF gripper.
  * CAN-FD runs 1 Mbit/s nominal, 5 Mbit/s data phase.

If you swap in real hardware, this is the single file you edit to match your
motor IDs and per-joint limits.
"""

from __future__ import annotations

from dataclasses import dataclass, field


# --------------------------------------------------------------------------- #
# Motor parameters
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class MotorLimits:
    """Damiao MIT-mode scaling limits.

    The motor encodes position/velocity/torque as fixed-width unsigned ints
    spanning [-MAX, +MAX]. These caps are what the on-wire integers decode
    against, so they must match the motor's configured P_MAX/V_MAX/T_MAX.
    """

    p_max: float  # rad
    v_max: float  # rad/s
    t_max: float  # N·m


# The three Damiao motors OpenArm 2.0 actually ships with, biggest at the
# shoulder for payload, smallest toward the wrist. t_max uses each motor's peak
# torque (the cap the on-wire torque integer decodes against); p_max/v_max are
# the Damiao firmware MIT-mode spans. Source: docs.openarm.dev motor table
# (DM-J4310-2EC 3/7 N·m, DM4340 9/27 N·m, DM-J8009P 20/40 N·m).
MOTOR_TYPES: dict[str, MotorLimits] = {
    "DM8009P": MotorLimits(p_max=12.5, v_max=20.0, t_max=40.0),  # shoulder
    "DM4340": MotorLimits(p_max=12.5, v_max=10.0, t_max=27.0),   # upper arm/elbow
    "DM4310": MotorLimits(p_max=12.5, v_max=30.0, t_max=7.0),    # forearm/wrist/gripper
}


@dataclass(frozen=True)
class JointConfig:
    name: str
    motor_type: str
    send_can_id: int  # host -> motor (Sender CAN ID),     J1..J7 = 0x01..0x07
    recv_can_id: int  # motor -> host (Receiver/Master ID), J1..J7 = 0x11..0x17

    @property
    def limits(self) -> MotorLimits:
        return MOTOR_TYPES[self.motor_type]


# One arm = 7 actuated joints + gripper. Joint *names* match OpenArm's dataset
# embodiment exactly — ('joint1', … 'joint7', 'gripper') — so recordings line up
# with the openarm_dataset format without renaming. CAN IDs follow the OpenArm
# motor-ID table: sender 0x0N / receiver 0x1N, gripper (J8) at 0x08 / 0x18.
def _build_arm() -> list[JointConfig]:
    layout = [
        ("joint1", "DM8009P"),  # shoulder pitch
        ("joint2", "DM8009P"),  # shoulder roll
        ("joint3", "DM4340"),   # shoulder yaw
        ("joint4", "DM4340"),   # elbow
        ("joint5", "DM4310"),   # forearm roll
        ("joint6", "DM4310"),   # wrist pitch
        ("joint7", "DM4310"),   # wrist roll
        ("gripper", "DM4310"),
    ]
    joints = []
    for i, (name, motor) in enumerate(layout, start=1):
        send_id = 0x08 if name == "gripper" else i
        recv_id = 0x18 if name == "gripper" else 0x10 + i
        joints.append(JointConfig(name, motor, send_id, recv_id))
    return joints


@dataclass(frozen=True)
class ArmConfig:
    name: str
    interface: str  # SocketCAN interface name
    joints: list[JointConfig] = field(default_factory=_build_arm)


ARMS: list[ArmConfig] = [
    ArmConfig(name="right", interface="can0"),
    ArmConfig(name="left", interface="can1"),
]

# CAN-FD timing (docs.openarm.dev recommended for a single arm).
CAN_NOMINAL_BITRATE = 1_000_000
CAN_DATA_BITRATE = 5_000_000

# How fast we poll/stream joint feedback. Real teleop logs ~500 Hz–1 kHz; we
# default to 500 Hz which is plenty to show the sync machinery without burying
# a laptop under synthetic load.
JOINT_STREAM_HZ = 500


# --------------------------------------------------------------------------- #
# Cameras
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class CameraConfig:
    name: str
    fps: int
    width: int
    height: int
    stereo: bool = False  # ZED head: a single device delivering an L|R pair


# Resolutions match what teleoperation / robot-learning pipelines actually
# record at (ALOHA, LeRobot collect ~480p, not 720p — smaller frames keep
# episodes trainable and storage sane). Raise these for archival capture, but
# note high-res streams should arrive via hardware DMA/USB, not synthetic encode.
CAMERAS: list[CameraConfig] = [
    CameraConfig("wrist_left", fps=60, width=640, height=480),
    CameraConfig("wrist_right", fps=60, width=640, height=480),
    CameraConfig("ceiling", fps=30, width=640, height=480),
    # ZED stereo head — OpenArm's dataset calls this stream "head"; we keep the
    # side-by-side L|R image (one device, shared timestamp) and split it into
    # head_left/head_right only on export if needed.
    CameraConfig("head", fps=30, width=640, height=480, stereo=True),
]

# Synchronization is driven by the slowest camera's frame arrivals (the anchor),
# not a free-running clock — so every recorded bundle contains a genuinely fresh
# frame from the anchor and "degraded" means a real dropped frame. SYNC_HZ is
# therefore the anchor camera's rate (informational); the recorder emits one
# bundle per anchor frame.
ANCHOR_CAMERA = "ceiling"
SYNC_HZ = 30
# A bundle is "degraded" only if some sensor is more than one anchor-frame period
# stale — i.e. a frame was actually dropped. Two free-running same-rate cameras
# (the ceiling anchor and the ZED, both 30 Hz) can sit up to a full period apart
# purely from clock phase; nearest-neighbour can't do better than that without a
# hardware trigger or PTP (see cameras/sync.py). So tolerance = one period; below
# it is expected phase jitter, above it is a real drop.
SYNC_TOLERANCE_MS = 1000.0 / SYNC_HZ  # ~33.3 ms

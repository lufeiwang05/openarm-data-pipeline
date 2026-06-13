"""Mock of the OpenArm CAN CLI (task 1).

Reproduces the real setup flow from docs.openarm.dev. On hardware you run:

    # bring up both CAN-FD interfaces (1M nominal / 5M data)
    openarm-can-cli -i can0 can_configure
    openarm-can-cli -i can1 can_configure

    # set zero position, one arm at a time
    openarm-can-zero-position-calibration --canport can0 --arm-side right_arm
    openarm-can-zero-position-calibration --canport can1 --arm-side left_arm

    # verify telemetry
    openarm-can-cli -i can0 monitor

Those commands need a SocketCAN adapter, so this mock issues the *same* steps
against the software bus and prints the same observable results (interface UP,
motors enabled, MST_ID query, zero set, live telemetry sample). Run:

    python -m openarm_pipeline.can.cli can_configure          # both arms
    python -m openarm_pipeline.can.cli monitor -i can0        # telemetry sample
"""

from __future__ import annotations

import argparse
import asyncio

from openarm_pipeline.can.bus import MockCANBus
from openarm_pipeline.can.damiao import command_frame
from openarm_pipeline.config import (
    ARMS,
    CAN_DATA_BITRATE,
    CAN_NOMINAL_BITRATE,
)


def _iface_to_arm(iface: str):
    for arm in ARMS:
        if arm.interface == iface:
            return arm
    return None


def _print_link_up(iface: str) -> None:
    print(f"$ openarm-can-cli -i {iface} can_configure")
    print(f"  ip link set {iface} type can bitrate {CAN_NOMINAL_BITRATE} "
          f"dbitrate {CAN_DATA_BITRATE} fd on")
    print(f"  ip link set {iface} up")
    print(f"$ ip link show {iface}")
    print(f"  {iface}: <NOARP,UP,LOWER_UP,ECHO> mtu 72 qdisc pfifo_fast "
          f"state UP qlen 10")
    print(f"      can FD state ERROR-ACTIVE (berr-counter tx 0 rx 0) restart-ms 100")
    print(f"      bitrate {CAN_NOMINAL_BITRATE} dbitrate {CAN_DATA_BITRATE} "
          f"  [ MOCK: no physical adapter ]")
    print(f"  [OK] {iface} UP  (CAN-FD "
          f"{CAN_NOMINAL_BITRATE//1000}k/{CAN_DATA_BITRATE//1_000_000}M)")


async def _bring_up_and_zero(arm) -> None:
    side = f"{arm.name}_arm"
    print(f"\n--- {arm.name} arm on {arm.interface}  ({len(arm.joints)} motors) ---")
    _print_link_up(arm.interface)

    bus = MockCANBus(arm)
    await bus.start()

    # Enable + query MST_ID, mirroring enable_all() / query_param_all(MST_ID).
    print(f"\n$ # enable_all() -> cansend {arm.interface} "
          f"0xx#{command_frame('enable').hex().upper()}")
    snap = await bus.read()
    print("  === Querying Motor Recv IDs ===")
    for j in arm.joints:
        print(f"    {j.name:<18} send 0x{j.send_can_id:02X}  "
              f"recv(MST_ID) 0x{j.recv_can_id:02X}  [{j.motor_type}]")
    print(f"  [OK] {len(snap.joints)}/{len(arm.joints)} motors responding")

    # Zero position.
    print(f"\n$ openarm-can-zero-position-calibration "
          f"--canport {arm.interface} --arm-side {side}")
    bus.set_zero()
    snap = await bus.read()
    max_resid = max(abs(fb.position) for fb in snap.joints.values())
    print(f"  set_zero -> max residual {max_resid*1000:.2f} mrad across all joints")
    print(f"  [OK] zero position set on {arm.interface} ({side})")
    await bus.stop()


async def can_configure() -> None:
    print("=" * 66)
    print("OpenArm 2.0 CAN-FD bring-up + zero calibration  (software mock)")
    print("=" * 66)
    for arm in ARMS:
        await _bring_up_and_zero(arm)
    print("\nAll interfaces UP and zeroed. Ready to record.\n")


async def monitor(iface: str, cycles: int = 5) -> None:
    arm = _iface_to_arm(iface)
    if arm is None:
        print(f"unknown interface {iface!r}")
        return
    print(f"$ openarm-can-cli -i {iface} monitor   ({arm.name} arm)")
    print(f"{'joint':<18}{'pos[rad]':>10}{'vel[rad/s]':>12}"
          f"{'tau[Nm]':>10}{'Tmos':>6}{'Trot':>6}  state")
    bus = MockCANBus(arm)
    await bus.start()
    for _ in range(cycles):
        snap = await bus.read()
        for name, fb in snap.joints.items():
            print(f"{name:<18}{fb.position:>10.3f}{fb.velocity:>12.3f}"
                  f"{fb.torque:>10.3f}{fb.t_mos:>6}{fb.t_rotor:>6}  {fb.error}")
        print("-" * 68)
        await asyncio.sleep(0.2)
    await bus.stop()


def main() -> None:
    parser = argparse.ArgumentParser(prog="openarm-can-cli")
    parser.add_argument("command", choices=["can_configure", "monitor"])
    parser.add_argument("-i", "--interface", default="can0",
                        help="CAN interface for `monitor` (can0|can1)")
    args = parser.parse_args()
    if args.command == "can_configure":
        asyncio.run(can_configure())
    elif args.command == "monitor":
        asyncio.run(monitor(args.interface))


if __name__ == "__main__":
    main()

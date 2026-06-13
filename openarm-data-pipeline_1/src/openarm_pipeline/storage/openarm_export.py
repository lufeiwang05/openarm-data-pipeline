"""Export a recorded MCAP episode into OpenArm's native dataset format (0.3.0).

We keep MCAP as the live recording log (streaming, multimodal, time-indexed) and
use this exporter to produce OpenArm's canonical at-rest layout for training and
sharing. From there, OpenArm's own `openarm_dataset` toolkit handles resampling
and LeRobot v2.1 conversion (`openarm-dataset-convert`).

Layout produced (per docs.openarm.dev/dataset/api "Dataset Structure"):

    <output>/
      metadata.yaml
      episodes/<id>/
        obs/arms/<side>/state.parquet     qpos+qvel+qtorque, columns = joint names
        action/arms/<side>/qpos.parquet   commanded qpos
        cameras/<name>/<unix_ns>.jpeg      one JPEG per frame, ns-timestamp filename

Fidelity notes (kept honest):
  * Directory tree, filenames (nanosecond Unix timestamps), camera stream names,
    joint names ('joint1'…'joint7','gripper'), and metadata.yaml fields follow
    the documented layout exactly.
  * The *internal column convention* of state.parquet is not given in the public
    docs (it lives in openarm_dataset's loader, which "explodes" it into
    qpos/qvel/qtorque). We use an explicit, documented convention — a
    nanosecond-timestamp index plus '<attr>.<joint>' columns — and recommend
    running `openarm-dataset-validate` before any production use.
  * The mock has no separate teleop-leader command stream, so we write the
    measured qpos as the action qpos (clearly a placeholder). On real hardware
    the action is the leader arm's commanded position.
  * The 'head' stereo stream is written as a single `head` folder to match the
    loader's `load_camera("head")`; the structure diagram's head_left/head_right
    split is the alternative for split stereo.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import pandas as pd
import yaml
from mcap.reader import make_reader

from openarm_pipeline.config import ARMS, CAMERAS

JOINT_NAMES = [j.name for j in ARMS[0].joints]
ARM_SIDES = [a.name for a in ARMS]
CAMERA_NAMES = [c.name for c in CAMERAS]


def _empirical_hz(timestamps_ns: list[int]) -> float:
    if len(timestamps_ns) < 2:
        return 0.0
    span_s = (timestamps_ns[-1] - timestamps_ns[0]) / 1e9
    return round((len(timestamps_ns) - 1) / span_s, 3) if span_s > 0 else 0.0


def export_episode(
    mcap_path: str | Path,
    output_dir: str | Path,
    episode_id: str = "0",
    operator: str = "mock",
    location: str = "software-mock",
    task_prompt: str = "Synthetic teleoperation episode.",
    success: bool = True,
) -> Path:
    """Read one MCAP episode and write the OpenArm dataset tree under output_dir."""
    mcap_path = Path(mcap_path)
    out = Path(output_dir)
    ep_dir = out / "episodes" / episode_id
    (ep_dir / "obs" / "arms").mkdir(parents=True, exist_ok=True)
    (ep_dir / "action" / "arms").mkdir(parents=True, exist_ok=True)
    (ep_dir / "cameras").mkdir(parents=True, exist_ok=True)

    # Accumulators
    joint_rows: dict[str, list[dict]] = {side: [] for side in ARM_SIDES}
    cam_ts: dict[str, list[int]] = {name: [] for name in CAMERA_NAMES}

    with open(mcap_path, "rb") as fh:
        reader = make_reader(fh)
        for _schema, channel, message in reader.iter_messages():
            topic = channel.topic
            t_ns = message.log_time
            if topic.startswith("/joint_states/"):
                side = topic.rsplit("/", 1)[1]
                payload = json.loads(message.data)
                row = {"timestamp_ns": t_ns}
                for jname, fb in payload["joints"].items():
                    row[f"qpos.{jname}"] = fb["position"]
                    row[f"qvel.{jname}"] = fb["velocity"]
                    row[f"qtorque.{jname}"] = fb["torque"]
                joint_rows.setdefault(side, []).append(row)
            elif topic.startswith("/camera/"):
                name = topic.rsplit("/", 1)[1]
                cam_dir = ep_dir / "cameras" / name
                cam_dir.mkdir(parents=True, exist_ok=True)
                (cam_dir / f"{t_ns}.jpeg").write_bytes(message.data)
                cam_ts.setdefault(name, []).append(t_ns)
            # '/sync' is recording diagnostics; not part of the OpenArm format.

    # Write per-arm parquet (obs state + action qpos).
    obs_hz, action_hz = {}, {}
    for side, rows in joint_rows.items():
        if not rows:
            continue
        df = pd.DataFrame(rows).sort_values("timestamp_ns").reset_index(drop=True)

        (ep_dir / "obs" / "arms" / side).mkdir(parents=True, exist_ok=True)
        df.to_parquet(ep_dir / "obs" / "arms" / side / "state.parquet", index=False)

        # action = commanded qpos (placeholder: measured qpos in the mock)
        qpos_cols = ["timestamp_ns"] + [f"qpos.{j}" for j in JOINT_NAMES]
        action_df = df[[c for c in qpos_cols if c in df.columns]].copy()
        action_df.columns = ["timestamp_ns"] + JOINT_NAMES  # bare joint names
        (ep_dir / "action" / "arms" / side).mkdir(parents=True, exist_ok=True)
        action_df.to_parquet(ep_dir / "action" / "arms" / side / "qpos.parquet", index=False)

        hz = _empirical_hz(df["timestamp_ns"].tolist())
        obs_hz[side] = hz
        action_hz[side] = hz

    # metadata.yaml
    metadata = {
        "version": "0.3.0",
        "operator": operator,
        "operation_type": "teleop",
        "location": location,
        "tasks": [{"prompt": task_prompt, "description": "Generated by the mock pipeline."}],
        "episodes": [{"id": episode_id, "success": bool(success), "task_index": 0}],
        "equipment": {
            "id": "OpenArm-2.0-mock",
            "version": "2.0",
            "embodiments": {
                "arms": {"id": "OpenArm", "version": "2.0"},
            },
            "perceptions": {
                "cameras": {name: {"stereo": bool(_is_stereo(name))} for name in CAMERA_NAMES},
            },
        },
        "frequencies": {
            "obs": {"arms": obs_hz},
            "action": {"arms": action_hz},
            "cameras": {name: _empirical_hz(ts) for name, ts in cam_ts.items() if ts},
        },
    }
    (out / "metadata.yaml").write_text(yaml.safe_dump(metadata, sort_keys=False))
    return out


def _is_stereo(name: str) -> bool:
    return any(c.name == name and c.stereo for c in CAMERAS)


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Export a recorded MCAP episode to OpenArm dataset format."
    )
    ap.add_argument("mcap", help="path to a recorded .mcap episode")
    ap.add_argument("output", help="output dataset directory")
    ap.add_argument("--episode-id", default="0")
    args = ap.parse_args()
    out = export_episode(args.mcap, args.output, episode_id=args.episode_id)
    print(f"wrote OpenArm dataset to {out}")
    for p in sorted(out.rglob("*")):
        if p.is_file():
            rel = p.relative_to(out)
            depth = len(rel.parts) - 1
            print("  " * depth + ("└─ " if depth else "") + rel.parts[-1])


if __name__ == "__main__":
    main()

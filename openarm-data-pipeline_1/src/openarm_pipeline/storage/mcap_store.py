"""Episode storage on MCAP (task 4).

Why MCAP (over HDF5 / zarr / custom):

  * Heterogeneous, time-indexed streams are exactly its job. An episode is N
    sensor streams at different rates sharing one timeline; MCAP stores each as
    a channel of timestamped messages and indexes by log time, so "give me
    everything between t0 and t1" is a seek, not a full scan. HDF5/zarr shine
    for dense homogeneous arrays (great for the joint matrix) but make you
    hand-roll the multi-rate, mixed-type indexing that MCAP gives for free.
  * Self-describing: schemas travel inside the file, so an episode recorded
    today still parses years later without this codebase.
  * Ecosystem: drops straight into Foxglove for visual inspection and is the
    de-facto rosbag2 format, so these episodes are portable into a real robot
    stack with zero conversion.
  * Append-friendly streaming writes — important when recording long
    teleop sessions you can't buffer in RAM.

Trade-off we accept: random access to a single far-in numeric column is less
direct than zarr. For training we'd likely transcode selected episodes to a
columnar/array layout offline; MCAP stays the source-of-truth log.

Layout per episode (one .mcap file):
    /joint_states/{arm}   json    JointState snapshots
    /camera/{name}        jpeg    raw JPEG frames
    /sync                 json    per-step skew + degraded flag
  + an MCAP metadata record "episode" with session-level fields.
"""

from __future__ import annotations

import json
import time
from pathlib import Path

from mcap.reader import make_reader
from mcap.writer import Writer

from openarm_pipeline.cameras.sync import SyncedBundle

DATA_DIR = Path(__file__).resolve().parents[3] / "data" / "episodes"

_JOINT_SCHEMA = json.dumps(
    {
        "type": "object",
        "properties": {
            "arm": {"type": "string"},
            "t_wall": {"type": "number"},
            "joints": {"type": "object"},
        },
    }
).encode()

_SYNC_SCHEMA = json.dumps(
    {
        "type": "object",
        "properties": {
            "step": {"type": "integer"},
            "max_skew_ms": {"type": "number"},
            "degraded": {"type": "boolean"},
        },
    }
).encode()


def _ns(t_wall: float) -> int:
    return int(t_wall * 1e9)


class EpisodeRecorder:
    """Streams synchronized bundles into a single MCAP file."""

    def __init__(self, episode_id: str, notes: str = "", data_dir: Path | None = None):
        self.episode_id = episode_id
        self.notes = notes
        self.dir = data_dir or DATA_DIR
        self.dir.mkdir(parents=True, exist_ok=True)
        self.path = self.dir / f"{episode_id}.mcap"

        self._fh = open(self.path, "wb")
        self._writer = Writer(self._fh)
        self._writer.start()

        self._joint_schema = self._writer.register_schema(
            name="openarm.JointState", encoding="jsonschema", data=_JOINT_SCHEMA
        )
        self._sync_schema = self._writer.register_schema(
            name="openarm.SyncStep", encoding="jsonschema", data=_SYNC_SCHEMA
        )
        self._channels: dict[str, int] = {}
        self._seq: dict[str, int] = {}

        self.t_start = time.time()
        self.steps = 0
        self.degraded_steps = 0

    def _channel(self, topic: str, encoding: str, schema_id: int) -> int:
        if topic not in self._channels:
            self._channels[topic] = self._writer.register_channel(
                topic=topic, message_encoding=encoding, schema_id=schema_id
            )
            self._seq[topic] = 0
        return self._channels[topic]

    def _emit(self, topic: str, encoding: str, schema_id: int, data: bytes, t_wall: float):
        cid = self._channel(topic, encoding, schema_id)
        seq = self._seq[topic]
        self._seq[topic] = seq + 1
        self._writer.add_message(
            channel_id=cid,
            log_time=_ns(t_wall),
            publish_time=_ns(t_wall),
            sequence=seq,
            data=data,
        )

    def write(self, bundle: SyncedBundle) -> None:
        for arm, state in bundle.joints.items():
            self._emit(
                f"/joint_states/{arm}", "json", self._joint_schema,
                json.dumps(state.as_dict()).encode(), state.t_wall,
            )
        for cam, frame in bundle.frames.items():
            self._emit(
                f"/camera/{cam}", "jpeg", 0, frame.jpeg, frame.t_wall,
            )
        self._emit(
            "/sync", "json", self._sync_schema,
            json.dumps(
                {
                    "step": bundle.step,
                    "max_skew_ms": round(bundle.max_skew_ms, 3),
                    "degraded": bundle.degraded,
                    "per_sensor_skew_ms": {
                        k: round(v, 3) for k, v in bundle.per_sensor_skew_ms.items()
                    },
                }
            ).encode(),
            bundle.t_tick if bundle.t_tick > 1e6 else time.time(),
        )
        self.steps += 1
        if bundle.degraded:
            self.degraded_steps += 1

    def close(self) -> dict:
        t_end = time.time()
        meta = {
            "episode_id": self.episode_id,
            "notes": self.notes,
            "t_start": str(self.t_start),
            "t_end": str(t_end),
            "duration_s": str(round(t_end - self.t_start, 3)),
            "steps": str(self.steps),
            "degraded_steps": str(self.degraded_steps),
            "source": "mock",
        }
        self._writer.add_metadata(name="episode", data=meta)
        self._writer.finish()
        self._fh.close()
        return {k: _coerce(v) for k, v in meta.items()}


def _coerce(v: str):
    for cast in (int, float):
        try:
            return cast(v)
        except (ValueError, TypeError):
            continue
    return v


class EpisodeStore:
    """Read-side: list episodes and pull metadata for the REST API."""

    def __init__(self, data_dir: Path | None = None):
        self.dir = data_dir or DATA_DIR
        self.dir.mkdir(parents=True, exist_ok=True)

    def path(self, episode_id: str) -> Path | None:
        p = self.dir / f"{episode_id}.mcap"
        return p if p.exists() else None

    def list_ids(self) -> list[str]:
        return sorted(p.stem for p in self.dir.glob("*.mcap"))

    def metadata(self, episode_id: str) -> dict | None:
        p = self.path(episode_id)
        if p is None:
            return None
        try:
            return self._read_metadata(p, episode_id)
        except Exception:
            # File is mid-write (currently recording, no index/summary yet) or
            # otherwise unreadable. Report what we can rather than 500-ing the API.
            return {
                "episode_id": episode_id,
                "status": "incomplete",
                "size_bytes": p.stat().st_size,
                "topics": {},
            }

    def _read_metadata(self, p: Path, episode_id: str) -> dict:
        with open(p, "rb") as fh:
            reader = make_reader(fh)
            summary = reader.get_summary()
            meta: dict = {"episode_id": episode_id, "status": "complete"}
            for record in reader.iter_metadata():
                if record.name == "episode":
                    meta.update({k: _coerce(v) for k, v in record.metadata.items()})

            channels = {}
            if summary and summary.statistics:
                stats = summary.statistics
                meta["message_count"] = stats.message_count
                if stats.message_start_time and stats.message_end_time:
                    meta["log_duration_s"] = round(
                        (stats.message_end_time - stats.message_start_time) / 1e9, 3
                    )
                for chan_id, count in stats.channel_message_counts.items():
                    chan = summary.channels.get(chan_id)
                    if chan:
                        channels[chan.topic] = count
            meta["topics"] = channels
            meta["size_bytes"] = p.stat().st_size
        return meta

    def list_all(self) -> list[dict]:
        return [m for eid in self.list_ids() if (m := self.metadata(eid))]

"""End-to-end demo (tasks 2-4, no server).

Spins up the mock arm + cameras, records a short synchronized episode to MCAP,
then reads it back and prints what landed on disk. This is the fastest way to
verify the whole pipeline without the web UI:

    python scripts/demo_record.py --seconds 3
"""

from __future__ import annotations

import argparse
import asyncio
from pathlib import Path

from openarm_pipeline.recorder import Recorder
from openarm_pipeline.storage.mcap_store import EpisodeStore


async def run(seconds: float, export: bool = False) -> None:
    rec = Recorder(mock=True)
    await rec.start()
    print(f"recording {seconds:.1f}s of synchronized data...")

    res = rec.start_recording(notes=f"demo {seconds}s")
    episode_id = res["episode_id"]
    await asyncio.sleep(seconds)
    meta = rec.stop_recording()
    await rec.stop()

    print(f"\nepisode  : {episode_id}")
    print(f"steps    : {meta['steps']}  ({meta['degraded_steps']} degraded)")
    print(f"duration : {meta['duration_s']}s")

    store = EpisodeStore()
    full = store.metadata(episode_id)
    print(f"\non disk   : {store.path(episode_id)}")
    print(f"size      : {full['size_bytes']/1024:.1f} KB")
    print(f"messages  : {full.get('message_count')}")
    print("topics    :")
    for topic, count in sorted(full["topics"].items()):
        print(f"    {topic:<28} {count:>5} msgs")

    if export:
        from openarm_pipeline.storage.openarm_export import export_episode

        out = Path("data") / "openarm_export" / episode_id
        export_episode(store.path(episode_id), out, episode_id=episode_id)
        print(f"\nexported to OpenArm dataset format -> {out}")
        for p in sorted(out.rglob("*")):
            if p.is_file():
                print(f"    {p.relative_to(out)}")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=3.0)
    ap.add_argument("--export", action="store_true",
                    help="also export to OpenArm native dataset format")
    args = ap.parse_args()
    asyncio.run(run(args.seconds, export=args.export))


if __name__ == "__main__":
    main()

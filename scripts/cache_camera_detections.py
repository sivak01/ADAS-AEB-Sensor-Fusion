"""
scripts/cache_camera_detections.py — STEP 04c §2: pre-computes 2D camera
detections in bulk (native-rate CAM_FRONT frames, every sweep not just
keyframes) and caches them to disk, keyed by (model name+version,
sample_data_token) -- outputs/cache/camera_dets/<model>/<token>.json.

Inference is slow (~2,400 frames across 10 scenes per step04c §3.1); the
camera adapter itself (adapters/camera.py) NEVER runs live inference --
it only reads this cache, so the slow step happens once, here, not on
every adapter call.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

from ttcf import config  # noqa: E402
from ttcf.adapters.camera import _cache_path  # noqa: E402
from ttcf.data.event_stream import scene_channel_events  # noqa: E402


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--split", choices=["tune", "eval", "all"], default="tune")
    parser.add_argument("--force", action="store_true", help="Re-run inference even if already cached")
    args = parser.parse_args()

    from nuscenes.nuscenes import NuScenes
    from ultralytics import YOLO

    if not config._has_nuscenes_layout(config.DATAROOT):
        print(f"STOP: {config.DATAROOT} does not have the expected nuScenes layout.")
        sys.exit(1)

    nusc = NuScenes(version=config.NUSCENES_VERSION, dataroot=str(config.DATAROOT), verbose=False)
    split = json.loads((Path(__file__).resolve().parent.parent / "configs" / "split.json").read_text())
    if args.split == "all":
        scene_entries = split["tune"] + split["eval"]
    else:
        scene_entries = split[args.split]

    model_key = config.CAMERA_DETECTOR_MODEL.value
    weights_path = Path("outputs/models") / f"{model_key}.pt"
    weights_path.parent.mkdir(parents=True, exist_ok=True)
    model = YOLO(str(weights_path))  # auto-downloads to this exact path if missing

    conf = config.CAMERA_DETECTOR_CONF_THRESH.value
    iou = config.CAMERA_DETECTOR_IOU_THRESH.value

    n_done = n_skipped = 0
    for scene_entry in scene_entries:
        scene_token = scene_entry["token"]
        for event in scene_channel_events(nusc, scene_token, "CAM_FRONT"):
            out_path = _cache_path(model_key, event.sample_data_token)
            if out_path.exists() and not args.force:
                n_skipped += 1
                continue

            img_path = config.DATAROOT / event.filename
            result = model.predict(str(img_path), conf=conf, iou=iou, verbose=False)[0]
            boxes = []
            for box in result.boxes:
                xyxy = box.xyxy[0].tolist()
                boxes.append(
                    {
                        "bbox": [float(x) for x in xyxy],
                        "cls_id": int(box.cls[0].item()),
                        "conf": float(box.conf[0].item()),
                    }
                )
            out_path.parent.mkdir(parents=True, exist_ok=True)
            out_path.write_text(json.dumps(boxes), encoding="utf-8")
            n_done += 1
            if n_done % 200 == 0:
                print(f"  ...{n_done} frames newly cached so far")

    print(f"Done. {n_done} frames newly cached, {n_skipped} already cached (skipped). "
          f"Cache: outputs/cache/camera_dets/{model_key}/")


if __name__ == "__main__":
    main()

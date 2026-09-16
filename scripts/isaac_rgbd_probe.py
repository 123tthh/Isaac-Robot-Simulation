#!/usr/bin/env python3
"""Capture one RGB and depth frame from the portable R1 scene without ROS."""

from pathlib import Path
import json

from isaacsim.simulation_app import SimulationApp


ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / "assets/scenes/scene_portable.usda"
OUT = ROOT / "outputs/rgbd_probe"
COLOR = "/World/Robot/camera_H_link/Realsense_H/RSD455/Camera_OmniVision_OV9782_Color"
DEPTH = "/World/Robot/camera_H_link/Realsense_H/RSD455/Camera_Pseudo_Depth"


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "phase.txt").write_text("start\n", encoding="utf-8")
    app = SimulationApp({"headless": True})
    try:
        import numpy as np
        from PIL import Image
        import omni.replicator.core as rep
        import omni.usd

        context = omni.usd.get_context()
        (OUT / "phase.txt").write_text("app ready\n", encoding="utf-8")
        if not context.open_stage(str(SCENE)):
            raise RuntimeError(f"Could not open {SCENE}")
        for _ in range(60):
            app.update()
        (OUT / "phase.txt").write_text("stage updated\n", encoding="utf-8")
        stage = context.get_stage()
        for path in (COLOR, DEPTH):
            if not stage.GetPrimAtPath(path).IsValid():
                raise RuntimeError(f"Missing camera: {path}")

        color_product = rep.create.render_product(COLOR, (320, 240))
        (OUT / "phase.txt").write_text("rgb render product\n", encoding="utf-8")
        depth_product = rep.create.render_product(DEPTH, (320, 240))
        (OUT / "phase.txt").write_text("depth render product\n", encoding="utf-8")
        rgb = rep.AnnotatorRegistry.get_annotator("rgb")
        distance = rep.AnnotatorRegistry.get_annotator("distance_to_image_plane")
        rgb.attach([color_product])
        distance.attach([depth_product])
        (OUT / "phase.txt").write_text("annotators attached\n", encoding="utf-8")
        rep.orchestrator.step(rt_subframes=16)
        (OUT / "phase.txt").write_text("orchestrator stepped\n", encoding="utf-8")

        rgb_data = np.asarray(rgb.get_data())
        depth_data = np.asarray(distance.get_data())
        (OUT / "phase.txt").write_text(f"arrays {rgb_data.shape} {depth_data.shape}\n", encoding="utf-8")
        if rgb_data.size == 0 or depth_data.size == 0:
            raise RuntimeError("RGB or depth annotator returned no pixels")
        OUT.mkdir(parents=True, exist_ok=True)
        Image.fromarray(rgb_data[:, :, :3].astype(np.uint8)).save(OUT / "head_rgb.png")
        np.save(OUT / "head_depth.npy", depth_data)
        valid_depth = np.isfinite(depth_data) & (depth_data > 0)
        report = {
            "scene": str(SCENE),
            "rgb_shape": list(rgb_data.shape),
            "depth_shape": list(depth_data.shape),
            "rgb_range": [int(rgb_data.min()), int(rgb_data.max())],
            "valid_depth_pixels": int(valid_depth.sum()),
            "depth_range_m": [float(depth_data[valid_depth].min()), float(depth_data[valid_depth].max())] if valid_depth.any() else None,
        }
        (OUT / "report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(report, indent=2))
        return 0 if valid_depth.any() else 1
    finally:
        app.close()


if __name__ == "__main__":
    raise SystemExit(main())

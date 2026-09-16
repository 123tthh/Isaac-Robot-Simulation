#!/usr/bin/env python3
"""List USD asset paths that cannot resolve from the project scene."""

from pathlib import Path
import json
import os

from isaacsim.simulation_app import SimulationApp


ROOT = Path(__file__).resolve().parents[1]
SCENE = Path(os.environ.get("ISAAC_USD_PATH", str(ROOT / "assets/scenes/scene.usd"))).resolve()


def main() -> int:
    app = SimulationApp({"headless": True})
    try:
        from pxr import Sdf, Usd

        stage = Usd.Stage.Open(str(SCENE))
        if stage is None:
            raise RuntimeError(f"Could not open {SCENE}")
        assets = []
        for prim in stage.Traverse():
            for attr in prim.GetAttributes():
                value = attr.Get()
                values = value if isinstance(value, (list, tuple)) else [value]
                for item in values:
                    if isinstance(item, Sdf.AssetPath):
                        assets.append({
                            "prim": str(prim.GetPath()),
                            "attribute": attr.GetName(),
                            "path": item.path,
                            "resolved": item.resolvedPath,
                        })
        missing = [x for x in assets if x["path"] and not x["resolved"]]
        absolute = [x for x in assets if x["path"].startswith("/")]
        shader_details = []
        for item in missing:
            prim = stage.GetPrimAtPath(item["prim"])
            shader_details.append({
                "prim": item["prim"],
                "attributes": {attr.GetName(): str(attr.Get()) for attr in prim.GetAttributes() if attr.GetName().startswith(("info:mdl", "inputs:"))},
            })
        report = {"asset_count": len(assets), "missing": missing, "absolute": absolute, "shader_details": shader_details}
        out = ROOT / "outputs/isaac_asset_audit.json"
        out.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        print(f"USD asset paths: {len(assets)}; unresolved: {len(missing)}; absolute: {len(absolute)}")
        return 1 if missing or absolute else 0
    finally:
        app.close()


if __name__ == "__main__":
    raise SystemExit(main())

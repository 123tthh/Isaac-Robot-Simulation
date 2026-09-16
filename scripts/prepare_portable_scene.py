#!/usr/bin/env python3
"""Create a small USD layer that replaces missing camera MDL modules."""

from pathlib import Path

from isaacsim.simulation_app import SimulationApp


ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / "assets/scenes/scene.usd"
OUTPUT = SCENE.with_name("scene_portable.usda")


def main() -> int:
    app = SimulationApp({"headless": True})
    try:
        from pxr import Sdf, Usd

        source_stage = Usd.Stage.Open(str(SCENE))
        if source_stage is None:
            raise RuntimeError(f"Could not open {SCENE}")
        stage = Usd.Stage.CreateNew(str(OUTPUT))
        stage.GetRootLayer().subLayerPaths.append("scene.usd")
        # Stage-wide metadata is taken from the root layer, not its sublayers.
        # Omitting these silently changes Z-up/metres to USD's Y-up/centimetres.
        for key in ("upAxis", "metersPerUnit", "timeCodesPerSecond", "framesPerSecond",
                    "startTimeCode", "endTimeCode", "defaultPrim", "customLayerData"):
            if source_stage.GetRootLayer().pseudoRoot.HasInfo(key):
                stage.GetRootLayer().pseudoRoot.SetInfo(
                    key, source_stage.GetRootLayer().pseudoRoot.GetInfo(key)
                )
        replaced = []
        for prim in source_stage.Traverse():
            if prim.GetTypeName() != "Shader":
                continue
            source_attr = prim.GetAttribute("info:mdl:sourceAsset")
            if not source_attr:
                continue
            asset = source_attr.Get()
            if not isinstance(asset, Sdf.AssetPath) or not asset.path.endswith(("Plastic_ABS.mdl", "Aluminum_Cast.mdl", "Aluminum_Anodized.mdl")):
                continue
            if asset.resolvedPath:
                continue
            override = stage.OverridePrim(str(prim.GetPath()))
            override.CreateAttribute("info:mdl:sourceAsset", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath("OmniPBR.mdl"))
            override.CreateAttribute("info:mdl:sourceAsset:subIdentifier", Sdf.ValueTypeNames.Token).Set("OmniPBR")
            replaced.append(str(prim.GetPath()))
        stage.GetRootLayer().Save()
        print(f"Created {OUTPUT} with {len(replaced)} camera material overrides")
        return 0 if len(replaced) == 9 else 1
    finally:
        app.close()


if __name__ == "__main__":
    raise SystemExit(main())

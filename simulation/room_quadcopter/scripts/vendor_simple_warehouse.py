#!/usr/bin/env python3
"""Vendor every dependency needed to run NVIDIA's Simple Warehouse offline."""

from __future__ import annotations

import argparse
import os
import shutil
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path
from urllib.parse import quote
from urllib.request import urlopen

from isaacsim import SimulationApp


REMOTE_ISAAC_ROOT = (
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/"
    "Assets/Isaac/6.0/Isaac"
)
REMOTE_WAREHOUSE_ROOT = f"{REMOTE_ISAAC_ROOT}/Environments/Simple_Warehouse"
REMOTE_WAREHOUSE = f"{REMOTE_WAREHOUSE_ROOT}/warehouse.usd"
REQUIRED_MDL_MODULES = ("OmniUe4Base.mdl", "OmniUe4Function.mdl")
REQUIRED_MDL_TEXTURES = ("T_SignsA_D.png", "AisleSign_Text_01.png")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    return parser.parse_args()


def download(url: str, destination: Path) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    partial = destination.with_name(f"{destination.name}.part")
    print(f"Downloading {url}", flush=True)
    with urlopen(url, timeout=120) as response, partial.open("wb") as stream:
        shutil.copyfileobj(response, stream)
    os.replace(partial, destination)


def dependency_url(path: Path, warehouse_dir: Path, room_root: Path) -> str:
    try:
        relative = path.relative_to(warehouse_dir)
        return f"{REMOTE_WAREHOUSE_ROOT}/{quote(relative.as_posix(), safe='/')}"
    except ValueError:
        pass
    try:
        relative = path.relative_to(room_root)
        return f"{REMOTE_ISAAC_ROOT}/{quote(relative.as_posix(), safe='/')}"
    except ValueError as exc:
        raise RuntimeError(f"Dependency escaped the local warehouse asset tree: {path}") from exc


def local_dependency_path(path: object, room_root: Path) -> Path:
    raw = Path(str(path))
    if raw.is_absolute():
        return raw.resolve()
    # This leaf is authored relative to the KLT collision USD, but USD's
    # dependency API reports only its basename when the anchor is unresolved.
    if raw.as_posix() == "small_KLT_visual.usd":
        return room_root / "Props" / "KLT_Bin" / raw
    if raw.parts[:2] == ("Materials", "Textures"):
        return room_root / "Props" / "KLT_Bin" / raw
    raise RuntimeError(f"Unanchored warehouse dependency needs an explicit mapping: {raw}")


def main() -> None:
    parse_args()
    room_root = Path(__file__).resolve().parents[1]
    warehouse_dir = room_root / "assets" / "simple_warehouse"
    local_root = warehouse_dir / "warehouse.usd"
    warehouse_dir.mkdir(parents=True, exist_ok=True)

    app = SimulationApp({"headless": True})
    try:
        from pxr import Sdf, Usd, UsdGeom, UsdUtils

        if not local_root.is_file():
            download(REMOTE_WAREHOUSE, local_root)
        for module in REQUIRED_MDL_MODULES:
            module_path = warehouse_dir / "Materials" / module
            if not module_path.is_file():
                download(f"{REMOTE_WAREHOUSE_ROOT}/Materials/{module}", module_path)
        # MDL texture defaults are not reported by ComputeAllDependencies, so
        # vendor these explicitly to keep the warehouse fully offline.
        for texture in REQUIRED_MDL_TEXTURES:
            texture_path = warehouse_dir / "Materials" / "Textures" / texture
            if not texture_path.is_file():
                download(
                    f"{REMOTE_WAREHOUSE_ROOT}/Materials/Textures/{texture}",
                    texture_path,
                )

        for round_number in range(1, 25):
            _layers, _assets, unresolved = UsdUtils.ComputeAllDependencies(Sdf.AssetPath(str(local_root)))
            missing = sorted({local_dependency_path(path, room_root) for path in unresolved})
            if not missing:
                break
            print(f"Dependency round {round_number}: downloading {len(missing)} file(s)", flush=True)
            with ThreadPoolExecutor(max_workers=8) as pool:
                futures = {
                    pool.submit(download, dependency_url(path, warehouse_dir, room_root), path): path
                    for path in missing
                }
                for future in as_completed(futures):
                    future.result()
        else:
            raise RuntimeError("Warehouse dependency resolution did not converge")

        _layers, _assets, unresolved = UsdUtils.ComputeAllDependencies(Sdf.AssetPath(str(local_root)))
        if unresolved:
            raise RuntimeError(f"Warehouse still has {len(unresolved)} unresolved dependencies")

        stage = Usd.Stage.Open(str(local_root))
        if stage is None or not stage.GetDefaultPrim().IsValid():
            raise RuntimeError("Vendored warehouse has no valid default prim")

        mesh_count = sum(1 for prim in stage.Traverse() if prim.IsA(UsdGeom.Mesh))
        if mesh_count < 100:
            raise RuntimeError(f"Vendored warehouse is incomplete: only {mesh_count} renderable meshes")
        print(f"Offline warehouse folder ready ({mesh_count} meshes): {warehouse_dir}", flush=True)
    finally:
        app.close()


if __name__ == "__main__":
    main()

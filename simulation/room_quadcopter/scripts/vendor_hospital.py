#!/usr/bin/env python3
"""Create and verify a complete, project-local Isaac Sim Hospital asset tree."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import shutil
import subprocess
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path
from urllib.parse import quote


REMOTE_ROOT = (
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/"
    "Assets/Isaac/6.0"
)
MATERIALS_REMOTE_ROOT = (
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Materials"
)
HOSPITAL_RELATIVE = Path("Isaac/Environments/Hospital")
HDR_RELATIVE = Path("NVIDIA/Assets/Skies/Cloudy/abandoned_parking_4k.hdr")
LIDAR_USD_RELATIVE = Path("Isaac/Sensors/Ouster/OS1/OS1.usd")
LIDAR_DEPENDENCIES = tuple(
    Path("Isaac/Sensors/Ouster/OS1") / name
    for name in (
        "OS1_REV6_128ch10hz1024res.usda",
        "OS1_REV6_128ch10hz2048res.usda",
        "OS1_REV6_128ch10hz512res.usda",
        "OS1_REV6_128ch20hz1024res.usda",
        "OS1_REV6_128ch20hz512res.usda",
        "OS1_REV6_32ch10hz1024res.usda",
        "OS1_REV6_32ch10hz2048res.usda",
        "OS1_REV6_32ch10hz512res.usda",
        "OS1_REV6_32ch20hz1024res.usda",
        "OS1_REV6_32ch20hz512res.usda",
        "OS1_REV7_128ch10hz1024res.usda",
        "OS1_REV7_128ch10hz2048res.usda",
        "OS1_REV7_128ch10hz512res.usda",
        "OS1_REV7_128ch20hz1024res.usda",
        "OS1_REV7_128ch20hz512res.usda",
    )
) + (Path("Isaac/Sensors/Ouster/meshes/os1_mesh.usd"),)
REQUIRED_TEXTURES = (
    "Tex_CeilingTile2x2_basecolor.png",
    "Tex_CeilingTile2x2_basecolor_5.png",
    "Tex_CeilingTile2x2_normal.png",
    "Tex_Checkered_RAM.png",
    "Tex_Wall_Plaster_Merge_RAM.png",
    "Tex_Wall_Plaster_basecolor.png",
    "Tex_Wall_Plaster_normal.png",
    "Tex_Wall_Plaster_normal1.png",
    "Tex_Wood_Floor_MergeRAM.png",
    "Tex_Wood_Floor_basecolor.png",
    "Tex_Wood_Floor_normal.png",
)
DRONE_MATERIAL_FILES = tuple(
    Path(path)
    for path in (
        "Base/Carpet/Carpet_Gray.mdl",
        "Base/Carpet/Carpet_Gray/Carpet_Gray_BaseColor.png",
        "Base/Carpet/Carpet_Gray/Carpet_Gray_ORM.png",
        "Base/Carpet/Carpet_Gray/Carpet_Gray_Normal.png",
        "Base/Textiles/Linen_Blue.mdl",
        "Base/Textiles/Linen_Blue/Linen_Blue_BaseColor.png",
        "Base/Textiles/Linen_Blue/Linen_Blue_ORM.png",
        "Base/Textiles/Linen_Blue/Linen_Blue_N.png",
        "Base/Textiles/Linen_Blue/Linen_Blue_DN.png",
    )
)
TEXTURE_PATTERN = re.compile(r'"(\./Textures/[^"\n]+\.(?:png|jpe?g|tga|exr|hdr))"', re.I)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Vendor the Hospital environment and verify every USD/MDL dependency."
    )
    parser.add_argument(
        "--source",
        type=Path,
        default=Path.home() / "Downloads" / "Hospital",
        help="Existing Hospital download to copy into the project",
    )
    parser.add_argument(
        "--drone-source",
        type=Path,
        default=Path.home() / "Downloads" / "iris.usd",
        help="Existing Iris USD to copy into the project",
    )
    parser.add_argument(
        "--verify-only",
        action="store_true",
        help="Do not copy or download; only validate the current offline package",
    )
    return parser.parse_args()


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def download_url(url: str, destination: Path) -> Path:
    destination.parent.mkdir(parents=True, exist_ok=True)
    partial = destination.with_suffix(f"{destination.suffix}.part")
    print(f"Downloading {url}", flush=True)
    subprocess.run(
        [
            "curl",
            "--fail",
            "--location",
            "--retry",
            "5",
            "--retry-delay",
            "2",
            "--retry-all-errors",
            "--speed-limit",
            "100000",
            "--speed-time",
            "15",
            "--continue-at",
            "-",
            "--silent",
            "--show-error",
            "--output",
            str(partial),
            url,
        ],
        check=True,
    )
    os.replace(partial, destination)
    return destination


def download(relative: Path, local_root: Path) -> Path:
    url = f"{REMOTE_ROOT}/{quote(relative.as_posix(), safe='/')}"
    return download_url(url, local_root / relative)


def missing_mdl_textures(hospital_dir: Path) -> list[Path]:
    missing: set[Path] = set()
    for mdl in hospital_dir.rglob("*.mdl"):
        contents = mdl.read_text(encoding="utf-8", errors="replace")
        for match in TEXTURE_PATTERN.finditer(contents):
            dependency = (mdl.parent / match.group(1)).resolve()
            if not dependency.is_file():
                missing.add(dependency)
    return sorted(missing)


def validate_usd(
    hospital_usd: Path,
    lidar_usd: Path,
    drone_usd: Path,
    materials_root: Path,
    rewrite_drone: bool,
) -> tuple[object, int, int, int, int, int]:
    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True})
    from pxr import Sdf, Usd, UsdGeom, UsdUtils

    drone_stage = Usd.Stage.Open(str(drone_usd))
    if drone_stage is None:
        raise RuntimeError(f"Could not open vendored Iris USD: {drone_usd}")
    drone_asset_count = 0
    for prim in drone_stage.Traverse():
        for attribute in prim.GetAttributes():
            if attribute.GetTypeName() != Sdf.ValueTypeNames.Asset:
                continue
            value = attribute.Get()
            if not isinstance(value, Sdf.AssetPath) or not value.path.startswith(("http://", "https://")):
                continue
            marker = "/Materials/"
            if marker not in value.path:
                raise RuntimeError(f"Unsupported remote Iris dependency: {value.path}")
            relative_material = Path("Materials") / value.path.split(marker, 1)[1]
            local_material = materials_root.parent / relative_material
            if not local_material.is_file():
                raise RuntimeError(f"Missing vendored Iris material: {local_material}")
            if rewrite_drone:
                relative_to_drone = os.path.relpath(local_material, drone_usd.parent)
                attribute.Set(Sdf.AssetPath(relative_to_drone))
            drone_asset_count += 1
    if rewrite_drone and drone_asset_count:
        drone_stage.GetRootLayer().Save()

    # A verification-only pass must find no CDN references in the persisted USD.
    remaining_remote = []
    persisted_drone = Usd.Stage.Open(str(drone_usd))
    for prim in persisted_drone.Traverse():
        for attribute in prim.GetAttributes():
            value = attribute.Get()
            if isinstance(value, Sdf.AssetPath) and value.path.startswith(("http://", "https://")):
                remaining_remote.append(value.path)
    if remaining_remote:
        raise RuntimeError(f"Iris USD still contains {len(remaining_remote)} remote asset references")

    layers, assets, unresolved = UsdUtils.ComputeAllDependencies(
        Sdf.AssetPath(str(hospital_usd))
    )
    if unresolved:
        unresolved_text = "\n".join(f"  - {path}" for path in unresolved)
        raise RuntimeError(f"Unresolved Hospital USD dependencies:\n{unresolved_text}")
    stage = Usd.Stage.Open(str(hospital_usd))
    if stage is None or not stage.GetDefaultPrim().IsValid():
        raise RuntimeError("Hospital USD has no valid default prim")
    mesh_count = sum(1 for prim in stage.Traverse() if prim.IsA(UsdGeom.Mesh))
    # Most hospital geometry is instanced from hundreds of referenced layers,
    # so the root stage intentionally exposes only a small mesh count.
    if mesh_count < 20 or len(layers) < 250:
        raise RuntimeError(
            f"Hospital composition is incomplete: {mesh_count} meshes, {len(layers)} layers"
        )
    lidar_layers, _lidar_assets, lidar_unresolved = UsdUtils.ComputeAllDependencies(
        Sdf.AssetPath(str(lidar_usd))
    )
    if lidar_unresolved:
        unresolved_text = "\n".join(f"  - {path}" for path in lidar_unresolved)
        raise RuntimeError(f"Unresolved OS1 LiDAR dependencies:\n{unresolved_text}")
    return app, len(layers), len(assets), mesh_count, len(lidar_layers), drone_asset_count


def main() -> None:
    args = parse_args()
    room_root = Path(__file__).resolve().parents[1]
    local_root = room_root / "assets" / "isaac" / "6.0"
    hospital_dir = local_root / HOSPITAL_RELATIVE
    hospital_usd = hospital_dir / "hospital.usd"
    lidar_usd = local_root / LIDAR_USD_RELATIVE
    drone_usd = room_root / "assets" / "iris" / "iris.usd"
    materials_root = room_root / "assets" / "materials" / "Materials"

    if not args.verify_only:
        source = args.source.expanduser().resolve()
        if (source / "hospital.usd").is_file():
            print(f"Copying the existing Hospital package from {source}", flush=True)
            shutil.copytree(source, hospital_dir, dirs_exist_ok=True)
        else:
            downloader = room_root.parents[1] / "scripts" / "download_assets.py"
            print(
                "No existing Hospital download was supplied; downloading the "
                "checksummed public asset bundle.",
                flush=True,
            )
            subprocess.run(
                ["python3", str(downloader), "--profile", "core", "--profile", "hospital"],
                check=True,
            )
        drone_source = args.drone_source.expanduser().resolve()
        if drone_source.is_file():
            drone_usd.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(drone_source, drone_usd)
        elif not drone_usd.is_file():
            raise FileNotFoundError(
                f"Iris source is absent and no project copy is installed: {drone_source}"
            )

        required = [
            HOSPITAL_RELATIVE / "Materials" / "Textures" / name
            for name in REQUIRED_TEXTURES
        ]
        required.extend((HDR_RELATIVE, LIDAR_USD_RELATIVE, *LIDAR_DEPENDENCIES))
        absent = [relative for relative in required if not (local_root / relative).is_file()]
        if absent:
            with ThreadPoolExecutor(max_workers=4) as pool:
                futures = [pool.submit(download, relative, local_root) for relative in absent]
                for future in as_completed(futures):
                    future.result()
        absent_materials = [
            relative for relative in DRONE_MATERIAL_FILES
            if not (materials_root / relative).is_file()
        ]
        if absent_materials:
            with ThreadPoolExecutor(max_workers=4) as pool:
                futures = [
                    pool.submit(
                        download_url,
                        f"{MATERIALS_REMOTE_ROOT}/{quote(relative.as_posix(), safe='/')}",
                        materials_root / relative,
                    )
                    for relative in absent_materials
                ]
                for future in as_completed(futures):
                    future.result()

    if not hospital_usd.is_file():
        raise FileNotFoundError(
            f"Offline Hospital is not installed: {hospital_usd}. Run without --verify-only first."
        )

    missing_textures = missing_mdl_textures(hospital_dir)
    if missing_textures:
        details = "\n".join(f"  - {path}" for path in missing_textures)
        raise RuntimeError(f"Missing MDL texture dependencies:\n{details}")

    app, layers, assets, meshes, lidar_layers, drone_assets_rewritten = validate_usd(
        hospital_usd,
        lidar_usd,
        drone_usd,
        materials_root,
        rewrite_drone=not args.verify_only,
    )
    downloaded_files = [
        local_root / HOSPITAL_RELATIVE / "Materials" / "Textures" / name
        for name in REQUIRED_TEXTURES
    ] + [
        local_root / HDR_RELATIVE,
        local_root / LIDAR_USD_RELATIVE,
        *(local_root / relative for relative in LIDAR_DEPENDENCIES),
        *(materials_root / relative for relative in DRONE_MATERIAL_FILES),
    ]
    manifest = {
        "asset_source": REMOTE_ROOT,
        "hospital_usd": str(hospital_usd.relative_to(room_root)),
        "usd_layers": layers,
        "non_layer_assets": assets,
        "composed_meshes": meshes,
        "lidar_usd": str(lidar_usd.relative_to(room_root)),
        "lidar_usd_layers": lidar_layers,
        "drone_usd": str(drone_usd.relative_to(room_root)),
        "offline_materials_root": str(materials_root.parent.relative_to(room_root)),
        "drone_remote_assets_rewritten": drone_assets_rewritten,
        "package_files": sum(1 for path in hospital_dir.rglob("*") if path.is_file()),
        "completed_dependencies": {
            str(path.relative_to(room_root)): {
                "bytes": path.stat().st_size,
                "sha256": sha256(path),
            }
            for path in downloaded_files
        },
    }
    manifest_path = local_root / "OFFLINE_HOSPITAL_MANIFEST.json"
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(
        f"Offline Hospital verified: {meshes} meshes, {layers} USD layers, "
        f"{assets} non-layer assets, OS1={lidar_layers} layers, "
        "0 unresolved dependencies",
        flush=True,
    )
    print(f"Hospital USD: {hospital_usd}", flush=True)
    print(f"Manifest: {manifest_path}", flush=True)
    app.close()


if __name__ == "__main__":
    main()

import json
import random
from pathlib import Path
from PIL import Image
from .map_loader import RoadSpec
from .renderer import render_camera_view_png, render_top_down_png
from .usd_builder import build_usd_stage
from .validator import validate_scenario


def _save_gif(png_paths: list[Path], gif_path: Path, duration_ms: int = 250) -> Path:
    frames = [Image.open(p).convert("RGBA") for p in png_paths]
    if not frames:
        raise ValueError("No PNG frames available to build GIF.")
    first, rest = frames[0], frames[1:]
    first.save(
        gif_path,
        save_all=True,
        append_images=rest,
        duration=duration_ms,
        loop=0,
    )
    for im in frames:
        im.close()
    return gif_path


def run_sweep(
    out_dir: Path,
    road: RoadSpec,
    obstacle_kind: str,
    base_lane_index: int,
    base_s_m: float,
    base_d_m: float,
    obs_lwh: tuple[float, float, float],
    cam_xyz: tuple[float, float, float],
    fov_deg: float,
    img_w: int,
    img_h: int,
    num_samples: int = 8,
    seed: int = 42,
    s_min_m: float = 10.0,
    s_max_m: float = 70.0,
) -> dict:
    out_dir.mkdir(parents=True, exist_ok=True)
    rng = random.Random(seed)

    top_pngs: list[Path] = []
    cam_pngs: list[Path] = []
    records: list[dict] = []

    s_values = (
        [base_s_m]
        if num_samples == 1
        else [s_min_m + (s_max_m - s_min_m) * i / (num_samples - 1) for i in range(num_samples)]
    )

    for idx, s_m in enumerate(s_values):
        lane_idx = base_lane_index if num_samples == 1 else rng.randrange(0, road.num_lanes)
        d_m = base_d_m if num_samples == 1 else rng.uniform(-0.4, 0.4)
        obs_xyz = road.lane_to_world(lane_idx, s_m, d_m)

        tag = f"{idx:03d}"
        usda_path = build_usd_stage(
            output_usda_path=out_dir / f"scenario_{tag}.usda",
            road_length_m=road.length_m,
            road_width_m=road.total_width_m(),
            obstacle_kind=obstacle_kind,
            obstacle_center_xyz=obs_xyz,
            obstacle_dims_lwh=obs_lwh,
            lane_index=lane_idx,
            s_distance_m=s_m,
        )
        top_png = render_top_down_png(
            out_path=out_dir / f"top_down_{tag}.png",
            road=road,
            lane_index=lane_idx,
            s_m=s_m,
            d_m=d_m,
            obs_length=obs_lwh[0],
            obs_width=obs_lwh[1],
            cam_xy=(cam_xyz[0], cam_xyz[1]),
        )
        cam_png, bbox_px = render_camera_view_png(
            out_path=out_dir / f"camera_{tag}.png",
            road=road,
            lane_index=lane_idx,
            s_m=s_m,
            d_m=d_m,
            obs_length=obs_lwh[0],
            obs_width=obs_lwh[1],
            obs_height=obs_lwh[2],
            cam_xyz=cam_xyz,
            fov_deg=fov_deg,
            img_w=img_w,
            img_h=img_h,
        )

        report = validate_scenario(
            road=road,
            lane_index=lane_idx,
            s_distance_m=s_m,
            d_offset_m=d_m,
            obs_lwh=obs_lwh,
            cam_xyz=cam_xyz,
            fov_deg=fov_deg,
            img_w=img_w,
            img_h=img_h,
            bbox_px=bbox_px,
        )

        top_pngs.append(top_png)
        cam_pngs.append(cam_png)
        records.append(
            {
                "index": idx,
                "usda": str(usda_path),
                "top_down_png": str(top_png),
                "camera_png": str(cam_png),
                "lane_index": lane_idx,
                "s_distance_m": round(s_m, 3),
                "d_offset_m": round(d_m, 3),
                "bbox_px": bbox_px,
                "valid_ok": report.ok,
                "warnings": report.warnings,
                "errors": report.errors,
            }
        )

    cam_gif = _save_gif(cam_pngs, out_dir / "sweep_camera.gif", duration_ms=300)
    top_gif = _save_gif(top_pngs, out_dir / "sweep_top_down.gif", duration_ms=300)

    manifest = {
        "seed": seed,
        "num_samples": num_samples,
        "camera_gif": str(cam_gif),
        "top_down_gif": str(top_gif),
        "samples": records,
    }
    (out_dir / "manifest.json").write_text(json.dumps(manifest, indent=2))
    return manifest
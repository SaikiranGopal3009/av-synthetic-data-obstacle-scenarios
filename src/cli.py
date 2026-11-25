import argparse
from pathlib import Path
import yaml
from .map_loader import RoadSpec
from .renderer import render_camera_view_png, render_top_down_png
from .sweep import run_sweep
from .usd_builder import build_usd_stage
from .validator import validate_scenario


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate map-grounded OpenUSD obstacle sample.")
    parser.add_argument("--config", type=Path, default=Path("configs/workzone_demo.yaml"))
    parser.add_argument("--num-samples", type=int, default=1)
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()

    cfg = yaml.safe_load(args.config.read_text())
    road = RoadSpec(
        length_m=float(cfg["road"]["length_m"]),
        num_lanes=int(cfg["road"]["num_lanes"]),
        lane_width_m=float(cfg["road"]["lane_width_m"]),
    )
    obs = cfg["obstacle"]
    cam = cfg["camera"]
    out_dir = Path(cfg["output"]["dir"])

    lane_idx = int(obs["lane_index"])
    s_m = float(obs["s_distance_m"])
    d_m = float(obs["d_offset_m"])
    obs_lwh = (float(obs["length_m"]), float(obs["width_m"]), float(obs["height_m"]))

    cam_xyz = (float(cam["x"]), float(cam["y"]), float(cam["z"]))
    fov_deg = float(cam["fov_deg"])
    img_w = int(cam["img_width"])
    img_h = int(cam["img_height"])

    if args.num_samples > 1:
        manifest = run_sweep(
            out_dir=out_dir,
            road=road,
            obstacle_kind=str(obs["kind"]),
            base_lane_index=lane_idx,
            base_s_m=s_m,
            base_d_m=d_m,
            obs_lwh=obs_lwh,
            cam_xyz=cam_xyz,
            fov_deg=fov_deg,
            img_w=img_w,
            img_h=img_h,
            num_samples=args.num_samples,
            seed=args.seed,
        )
        print(f"Wrote camera GIF:   {manifest['camera_gif']}")
        print(f"Wrote top-down GIF: {manifest['top_down_gif']}")
        print(f"Wrote manifest:     {out_dir / 'manifest.json'}")
        return

    obs_xyz = road.lane_to_world(lane_idx, s_m, d_m)
    usda_path = build_usd_stage(
        output_usda_path=out_dir / "scenario_000.usda",
        road_length_m=road.length_m,
        road_width_m=road.total_width_m(),
        obstacle_kind=str(obs["kind"]),
        obstacle_center_xyz=obs_xyz,
        obstacle_dims_lwh=obs_lwh,
        lane_index=lane_idx,
        s_distance_m=s_m,
    )
    top_png = render_top_down_png(
        out_path=out_dir / "top_down_000.png",
        road=road,
        lane_index=lane_idx,
        s_m=s_m,
        d_m=d_m,
        obs_length=obs_lwh[0],
        obs_width=obs_lwh[1],
        cam_xy=(cam_xyz[0], cam_xyz[1]),
    )
    cam_png, bbox_px = render_camera_view_png(
        out_path=out_dir / "camera_000.png",
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
    for warn in report.warnings:
        print(f"WARNING: {warn}")
    if not report.ok:
        for err in report.errors:
            print(f"ERROR: {err}")
        raise ValueError("Scenario validation failed.")

    print(f"Wrote USD:        {usda_path}")
    print(f"Wrote top-down:   {top_png}")
    print(f"Wrote camera PNG: {cam_png}")
    print(f"2D bbox (px):     {bbox_px}")


if __name__ == "__main__":
    main()
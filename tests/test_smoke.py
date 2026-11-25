from pathlib import Path
from src.map_loader import RoadSpec
from src.renderer import render_camera_view_png, render_top_down_png
from src.sweep import run_sweep
from src.usd_builder import build_usd_stage
from src.validator import validate_scenario


def test_single_sample_end_to_end(tmp_path: Path) -> None:
    road = RoadSpec(length_m=100.0, num_lanes=3, lane_width_m=3.6)
    obs_lwh = (4.2, 1.8, 1.5)
    cam_xyz = (0.0, 0.0, 1.5)

    obs_xyz = road.lane_to_world(0, 28.0, 0.0)
    usda_path = build_usd_stage(
        output_usda_path=tmp_path / "scenario_000.usda",
        road_length_m=road.length_m,
        road_width_m=road.total_width_m(),
        obstacle_kind="workzone_barrier",
        obstacle_center_xyz=obs_xyz,
        obstacle_dims_lwh=obs_lwh,
        lane_index=0,
        s_distance_m=28.0,
    )
    top_png = render_top_down_png(
        out_path=tmp_path / "top_down_000.png",
        road=road,
        lane_index=0,
        s_m=28.0,
        d_m=0.0,
        obs_length=obs_lwh[0],
        obs_width=obs_lwh[1],
        cam_xy=(cam_xyz[0], cam_xyz[1]),
    )
    cam_png, bbox_px = render_camera_view_png(
        out_path=tmp_path / "camera_000.png",
        road=road,
        lane_index=0,
        s_m=28.0,
        d_m=0.0,
        obs_length=obs_lwh[0],
        obs_width=obs_lwh[1],
        obs_height=obs_lwh[2],
        cam_xyz=cam_xyz,
        fov_deg=60.0,
        img_w=800,
        img_h=450,
    )

    report = validate_scenario(
        road=road,
        lane_index=0,
        s_distance_m=28.0,
        d_offset_m=0.0,
        obs_lwh=obs_lwh,
        cam_xyz=cam_xyz,
        fov_deg=60.0,
        img_w=800,
        img_h=450,
        bbox_px=bbox_px,
    )

    assert usda_path.exists()
    assert top_png.exists()
    assert cam_png.exists()
    assert bbox_px is not None
    assert report.ok
    assert report.errors == []


def test_validator_catches_invalid_lane() -> None:
    road = RoadSpec(length_m=100.0, num_lanes=3, lane_width_m=3.6)
    report = validate_scenario(
        road=road,
        lane_index=5,
        s_distance_m=28.0,
        d_offset_m=0.0,
        obs_lwh=(4.2, 1.8, 1.5),
        cam_xyz=(0.0, 0.0, 1.5),
        fov_deg=60.0,
        img_w=800,
        img_h=450,
        bbox_px=None,
    )
    assert not report.ok
    assert len(report.errors) >= 1


def test_sweep_creates_gifs_and_manifest(tmp_path: Path) -> None:
    road = RoadSpec(length_m=100.0, num_lanes=3, lane_width_m=3.6)
    manifest = run_sweep(
        out_dir=tmp_path / "sweep",
        road=road,
        obstacle_kind="workzone_barrier",
        base_lane_index=0,
        base_s_m=28.0,
        base_d_m=0.0,
        obs_lwh=(4.2, 1.8, 1.5),
        cam_xyz=(0.0, 0.0, 1.5),
        fov_deg=60.0,
        img_w=800,
        img_h=450,
        num_samples=3,
        seed=42,
    )
    assert Path(manifest["camera_gif"]).exists()
    assert Path(manifest["top_down_gif"]).exists()
    assert (tmp_path / "sweep" / "manifest.json").exists()
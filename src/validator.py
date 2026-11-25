from dataclasses import dataclass
from .map_loader import RoadSpec


@dataclass
class ValidationResult:
    ok: bool
    warnings: list[str]
    errors: list[str]


def validate_scenario(
    road: RoadSpec,
    lane_index: int,
    s_distance_m: float,
    d_offset_m: float,
    obs_lwh: tuple[float, float, float],
    cam_xyz: tuple[float, float, float],
    fov_deg: float,
    img_w: int,
    img_h: int,
    bbox_px: tuple[float, float, float, float] | None = None,
) -> ValidationResult:
    errors: list[str] = []
    warnings: list[str] = []

    length_m, width_m, height_m = obs_lwh
    if length_m <= 0.0 or width_m <= 0.0 or height_m <= 0.0:
        errors.append(f"Obstacle dimensions must be positive, got L={length_m}, W={width_m}, H={height_m}.")

    if lane_index < 0 or lane_index >= road.num_lanes:
        errors.append(f"lane_index={lane_index} out of range for num_lanes={road.num_lanes}.")

    if s_distance_m < 0.0 or s_distance_m > road.length_m:
        errors.append(f"s_distance_m={s_distance_m} outside road length [0.0, {road.length_m}].")

    if road.num_lanes > 0 and 0 <= lane_index < road.num_lanes:
        cx, _, _ = road.lane_to_world(lane_index, s_distance_m, d_offset_m)
        half_w = width_m / 2.0
        if cx - half_w < 0.0 or cx + half_w > road.total_width_m():
            warnings.append(
                f"Obstacle lateral footprint [{cx - half_w:.2f}, {cx + half_w:.2f}] crosses road edges [0.0, {road.total_width_m():.2f}]."
            )

    if img_w <= 0 or img_h <= 0:
        errors.append(f"Invalid camera image size: {img_w}x{img_h}.")
    if not (10.0 <= fov_deg <= 150.0):
        warnings.append(f"Unusual camera fov_deg={fov_deg}; expected roughly 10 to 150 deg.")

    cam_x, cam_y, cam_z = cam_xyz
    if cam_z <= 0.0:
        warnings.append(f"Camera height z={cam_z} is at or below ground plane z=0.0.")
    if s_distance_m <= cam_y:
        warnings.append(f"Obstacle s_distance_m={s_distance_m} is at or behind camera y={cam_y}.")

    if bbox_px is not None:
        u0, v0, u1, v1 = bbox_px
        bw = u1 - u0
        bh = v1 - v0
        if bw <= 0.0 or bh <= 0.0:
            warnings.append("Projected 2D bounding box is empty or outside camera frame.")
        elif bw < 4.0 or bh < 4.0:
            warnings.append(f"Projected 2D bounding box is very small ({bw:.1f}x{bh:.1f} px).")

    return ValidationResult(ok=len(errors) == 0, warnings=warnings, errors=errors)
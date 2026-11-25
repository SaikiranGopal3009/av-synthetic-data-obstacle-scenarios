from pathlib import Path
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np
from .map_loader import RoadSpec


def render_top_down_png(
    out_path: Path,
    road: RoadSpec,
    lane_index: int,
    s_m: float,
    d_m: float,
    obs_length: float,
    obs_width: float,
    cam_xy: tuple[float, float],
    ego_label: str = "Ego Vehicle Camera",
) -> Path:
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(6, 8))

    # Draw full road rectangle from x=0 to total_width_m, y=0 to length_m
    ax.add_patch(
        Rectangle(
            (0.0, 0.0),
            road.total_width_m(),
            road.length_m,
            facecolor="#e5e7eb",
            edgecolor="#374151",
            lw=1.5,
        )
    )

    # Draw dashed centerline for each lane
    for idx in range(road.num_lanes):
        pts = road.centerline_points(idx)
        ax.plot(pts[:, 0], pts[:, 1], "--", color="#6b7280", lw=1.2, label="Lane Centerline" if idx == 0 else "")

    # Compute obstacle center in world XY and draw its footprint rectangle
    cx, cy, _ = road.lane_to_world(lane_index, s_m, d_m)
    ax.add_patch(
        Rectangle(
            (cx - obs_width / 2.0, cy - obs_length / 2.0),
            obs_width,
            obs_length,
            facecolor="#ef4444",
            edgecolor="#7f1d1d",
            lw=1.5,
            label="Obstacle",
        )
    )

    # Plot ego-mounted camera position on the map
    ax.scatter([cam_xy[0]], [cam_xy[1]], c="#2563eb", s=80, zorder=5, label=ego_label)

    ax.set_xlim(-2.0, road.total_width_m() + 2.0)
    ax.set_ylim(min(-4.0, cam_xy[1] - 2.0), road.length_m + 2.0)
    ax.set_aspect("equal")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y / Downrange s (m)")
    ax.set_title("Top-Down Lane & Obstacle Layout")
    ax.legend(bbox_to_anchor=(1.02, 1.0), loc="upper left", borderaxespad=0.0)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    return out_path


def render_camera_view_png(
    out_path: Path,
    road: RoadSpec,
    lane_index: int,
    s_m: float,
    d_m: float,
    obs_length: float,
    obs_width: float,
    obs_height: float,
    cam_xyz: tuple[float, float, float],
    fov_deg: float = 60.0,
    img_w: int = 800,
    img_h: int = 450,
    ego_label: str = "Ego Vehicle Camera",
) -> tuple[Path, tuple[float, float, float, float] | None]:
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.set_facecolor("#cbd5e1")
    # Shade lower half of image as rough ground region
    ax.axhspan(img_h * 0.5, img_h, color="#475569", alpha=0.35)

    # Pinhole focal length in pixels from horizontal FOV and image width
    f = (img_w / 2.0) / np.tan(np.deg2rad(fov_deg / 2.0))
    cam_x, cam_y, cam_z = cam_xyz

    def project_point(px: float, py: float, pz: float) -> tuple[float, float] | None:
        dy = py - cam_y
        # Skip points behind or right at the camera plane
        if dy <= 0.5:
            return None
        u = img_w / 2.0 + f * ((px - cam_x) / dy)
        v = img_h / 2.0 - f * ((pz - cam_z) / dy)
        return (u, v)

    # Project longitudinal lane boundary lines onto the image plane
    for edge_idx in range(road.num_lanes + 1):
        x_edge = edge_idx * road.lane_width_m
        ys = np.linspace(max(cam_y + 1.0, 1.0), road.length_m, 40)
        pts_2d = [project_point(x_edge, float(y), 0.0) for y in ys]
        pts_2d = [p for p in pts_2d if p is not None]
        if len(pts_2d) > 1:
            xs_p, ys_p = zip(*pts_2d)
            ax.plot(xs_p, ys_p, color="#f8fafc", lw=1.5)

    # Project all 8 corners of the 3D obstacle box and take their 2D screen bounds
    cx, cy, _ = road.lane_to_world(lane_index, s_m, d_m)
    corners_3d = [
        (cx + sx * obs_width / 2.0, cy + sy * obs_length / 2.0, sz * obs_height)
        for sx in (-1.0, 1.0)
        for sy in (-1.0, 1.0)
        for sz in (0.0, 1.0)
    ]
    projected = [project_point(*c) for c in corners_3d]
    projected = [p for p in projected if p is not None]

    bbox_px: tuple[float, float, float, float] | None = None
    if projected:
        us, vs = zip(*projected)
        u0, u1 = max(0.0, min(us)), min(float(img_w), max(us))
        v0, v1 = max(0.0, min(vs)), min(float(img_h), max(vs))
        if u1 > u0 and v1 > v0:
            bbox_px = (float(u0), float(v0), float(u1), float(v1))
            ax.add_patch(
                Rectangle((u0, v0), u1 - u0, v1 - v0, fill=False, edgecolor="#ef4444", lw=2.0)
            )
            ax.text(
                u0,
                max(12.0, v0 - 6.0),
                f"obstacle ({s_m:.1f}m)",
                color="#fecaca",
                fontsize=9,
                weight="bold",
            )

    # Pixel coordinates have (0, 0) at top-left and v increasing downward
    ax.set_xlim(0, img_w)
    ax.set_ylim(img_h, 0)
    ax.set_xlabel("u (px)")
    ax.set_ylabel("v (px)")
    ax.set_title(f"{ego_label} — Projection + Ground-Truth 2D Box")
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    return out_path, bbox_px
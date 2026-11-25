from pathlib import Path
from pxr import Gf, Sdf, Usd, UsdGeom


def build_usd_stage(
    output_usda_path: Path,
    road_length_m: float,
    road_width_m: float,
    obstacle_kind: str,
    obstacle_center_xyz: tuple[float, float, float],
    obstacle_dims_lwh: tuple[float, float, float],
    lane_index: int,
    s_distance_m: float,
) -> Path:
    output_usda_path.parent.mkdir(parents=True, exist_ok=True)
    stage = Usd.Stage.CreateNew(str(output_usda_path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    world = UsdGeom.Xform.Define(stage, "/World").GetPrim()
    stage.SetDefaultPrim(world)

    # Road surface cube positioned below z=0 so top sits flush on road plane
    road_prim = UsdGeom.Cube.Define(stage, "/World/Map/RoadSurface")
    road_xform = UsdGeom.Xformable(road_prim)
    road_xform.AddTranslateOp().Set(Gf.Vec3d(road_width_m / 2.0, road_length_m / 2.0, -0.05))
    road_xform.AddScaleOp().Set(Gf.Vec3f(road_width_m / 2.0, road_length_m / 2.0, 0.05))

    # Obstacle cube sitting on z=0 (bottom at oz, center lifted by height/2)
    ox, oy, oz = obstacle_center_xyz
    length, width, height = obstacle_dims_lwh
    obs_path = f"/World/Obstacles/{obstacle_kind}_0"
    obs_cube = UsdGeom.Cube.Define(stage, obs_path)
    obs_xform = UsdGeom.Xformable(obs_cube)
    obs_xform.AddTranslateOp().Set(Gf.Vec3d(ox, oy, oz + height / 2.0))
    obs_xform.AddScaleOp().Set(Gf.Vec3f(width / 2.0, length / 2.0, height / 2.0))

    # Custom domain attributes for scenario traceability
    obs_prim = obs_cube.GetPrim()
    obs_prim.CreateAttribute("sdg:obstacleKind", Sdf.ValueTypeNames.String).Set(obstacle_kind)
    obs_prim.CreateAttribute("sdg:laneIndex", Sdf.ValueTypeNames.Int).Set(int(lane_index))
    obs_prim.CreateAttribute("sdg:sDistanceM", Sdf.ValueTypeNames.Double).Set(float(s_distance_m))

    stage.GetRootLayer().Save()
    return output_usda_path
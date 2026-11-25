from dataclasses import dataclass
import numpy as np


@dataclass
class RoadSpec:
    length_m: float
    num_lanes: int
    lane_width_m: float

    def lane_center_x(self, lane_index: int) -> float:
        return (lane_index + 0.5) * self.lane_width_m

    def total_width_m(self) -> float:
        return self.num_lanes * self.lane_width_m

    def lane_to_world(self, lane_index: int, s_m: float, d_m: float = 0.0) -> tuple[float, float, float]:
        x = self.lane_center_x(lane_index) + d_m
        y = s_m
        z = 0.0
        return (x, y, z)

    def centerline_points(self, lane_index: int, num_pts: int = 50) -> np.ndarray:
        ys = np.linspace(0.0, self.length_m, num_pts)
        xs = np.full_like(ys, self.lane_center_x(lane_index))
        zs = np.zeros_like(ys)
        return np.column_stack([xs, ys, zs])
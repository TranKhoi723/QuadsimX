"""
tests/test_pathfinding.py
============================
Kiem tra thuat toan A* ne vat can (pathfinding.py) - phan MOI, chua co test
nao truoc do.
"""

import numpy as np
import pytest

from quadsim.pathfinding import (
    rasterize_no_fly_zones, inflate, astar_grid, simplify_path_rdp, plan_path_pixels,
)


def test_rasterize_polygon_marks_correct_region():
    zones = [{"type": "polygon", "points_px": [[10, 10], [30, 10], [30, 30], [10, 30]]}]
    occ, step = rasterize_no_fly_zones((50, 50, 3), zones, grid_step_px=1)
    assert occ[20, 20]      # tam vung cam -> phai la vat can
    assert not occ[45, 45]  # xa vung cam -> khong phai vat can


def test_rasterize_circle_marks_correct_region():
    zones = [{"type": "circle", "center_px": [25, 25], "radius_px": 10}]
    occ, step = rasterize_no_fly_zones((50, 50, 3), zones, grid_step_px=1)
    assert occ[25, 25]        # tam hinh tron
    assert not occ[49, 49]    # goc xa, ngoai hinh tron


def test_inflate_expands_obstacle():
    occ = np.zeros((20, 20), dtype=bool)
    occ[10, 10] = True
    inflated = inflate(occ, radius_cells=2)
    assert inflated.sum() > 1                 # phai lon hon vung goc (1 pixel)
    assert inflated[10, 10]                    # van chua diem goc
    assert inflated[10, 12] or inflated[12, 10]  # da phinh ra xung quanh


def test_astar_finds_straight_path_when_no_obstacle():
    occ = np.zeros((30, 30), dtype=bool)
    path = astar_grid(occ, (0, 0), (10, 10))
    assert path is not None
    assert path[0] == (0, 0)
    assert path[-1] == (10, 10)


def test_astar_returns_none_when_goal_unreachable():
    """Vat can bao KIN hoan toan quanh dich -> A* PHAI tra ve None, khong
    duoc crash hay tra ve duong di xuyen qua tuong."""
    occ = np.zeros((20, 20), dtype=bool)
    occ[5, 3:8] = True   # 4 buc tuong bao kin 1 o (6,5)
    occ[9, 3:8] = True
    occ[5:10, 3] = True
    occ[5:10, 7] = True
    path = astar_grid(occ, (0, 0), (6, 5))
    assert path is None


def test_astar_returns_none_when_start_inside_obstacle():
    occ = np.zeros((10, 10), dtype=bool)
    occ[2, 2] = True
    path = astar_grid(occ, (2, 2), (8, 8))
    assert path is None


def test_simplify_rdp_reduces_point_count_on_straight_line():
    """Duong THANG (nhieu diem thang hang) phai duoc rut gon ve gan nhu 2 dau."""
    pts = np.column_stack([np.linspace(0, 100, 50), np.zeros(50)])
    simplified = simplify_path_rdp(pts, epsilon=1.0)
    assert len(simplified) <= 3   # duong thang tuyet doi -> chi can 2 diem (dau/cuoi)


def test_simplify_rdp_keeps_sharp_corner():
    """Duong gap khuc VUONG GOC ro ret phai GIU LAI diem gay, khong duoc xoa mat."""
    pts = np.array([[0, 0], [50, 0], [50, 50]], dtype=float)
    simplified = simplify_path_rdp(pts, epsilon=1.0)
    assert len(simplified) == 3


def test_plan_path_pixels_end_to_end_avoids_obstacle():
    """Kiem tra tich hop toan bo pipeline: co 1 vat can chan giua duong
    thang noi start-goal (nhung CHUA het chieu cao anh, con khe ho phia
    duoi) -> duong di tra ve PHAI di VONG qua, khong duoc cat xuyen qua
    vung cam (kiem tra bang rasterize lai va so voi occ)."""
    image_shape = (200, 200, 3)
    no_fly = [{"type": "polygon", "points_px": [[80, 0], [120, 0], [120, 150], [80, 150]]}]
    path = plan_path_pixels(image_shape, no_fly, start_px=(20, 100), goal_px=(180, 100),
                             grid_step_px=4, safety_margin_px=6, simplify_eps_px=4.0)
    assert path is not None
    assert len(path) >= 2

    occ, step = rasterize_no_fly_zones(image_shape, no_fly, grid_step_px=2)
    occ_inflated = inflate(occ, radius_cells=1)
    for u, v in path:
        col, row = int(round(u / step)), int(round(v / step))
        row = min(max(row, 0), occ_inflated.shape[0] - 1)
        col = min(max(col, 0), occ_inflated.shape[1] - 1)
        assert not occ_inflated[row, col], f"Waypoint ({u:.0f},{v:.0f}) roi VAO trong vung cam bay!"


def test_plan_path_pixels_returns_none_when_fully_blocked():
    image_shape = (100, 100, 3)
    no_fly = [{"type": "polygon", "points_px": [[0, 40], [100, 40], [100, 60], [0, 60]]}]
    path = plan_path_pixels(image_shape, no_fly, start_px=(50, 10), goal_px=(50, 90),
                             grid_step_px=4, safety_margin_px=25, simplify_eps_px=4.0)
    assert path is None

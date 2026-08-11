"""
quadsim.pathfinding
======================
LAP DUONG DI TU DONG NE VAT CAN tren anh 2D (grid-based A*), dung cho tinh
nang "click 2 diem tren anh -> tu sinh waypoint ne vat can" cua GUI.

Luong xu ly:
    1. Rasterize cac no_fly_zones (polygon/circle, toa do PIXEL) thanh 1
       ma tran occupancy grid (mac dinh downsample de A* chay nhanh).
    2. "Phinh" (inflate) vat can them 1 ban kinh an toan (so pixel) de
       quadcopter khong bay sat mep vung cam.
    3. A* tim duong ngan nhat (8-huong, chi phi bang khoang cach Euclid)
       tu diem bat dau toi diem ket thuc tren grid da phinh.
    4. Rut gon duong di (loai bot diem thang hang) bang thuat toan
       Ramer-Douglas-Peucker de KHONG sinh qua nhieu waypoint vun vat.

Neu A* khong tim duoc duong (vi du diem nam trong vung cam, hoac vung cam
bao kin hoan toan) -> tra ve None, GUI se bao loi cho nguoi dung chon lai.
"""

import heapq
import numpy as np


def rasterize_no_fly_zones(image_shape, no_fly_zones, grid_step_px=4):
    """
    Tao occupancy grid (True = vat can) tu danh sach no_fly_zones (toa do
    PIXEL goc, xem dinh dang trong waypoint_io.py). Grid duoc downsample
    theo grid_step_px de A* chay nhanh tren anh lon.

    Tra ve: (occ_grid (H',W') bool, grid_step_px)
    """
    h, w = image_shape[0], image_shape[1]
    gh, gw = max(int(np.ceil(h / grid_step_px)), 1), max(int(np.ceil(w / grid_step_px)), 1)
    occ = np.zeros((gh, gw), dtype=bool)

    # Toa do tam moi o grid, quy ve pixel goc de test nam-trong-vung-cam
    gy, gx = np.mgrid[0:gh, 0:gw]
    px_x = (gx + 0.5) * grid_step_px
    px_y = (gy + 0.5) * grid_step_px

    for zone in no_fly_zones:
        ztype = zone.get("type", "polygon")
        if ztype == "circle":
            cx, cy = zone["center_px"]
            r = zone["radius_px"]
            occ |= (px_x - cx) ** 2 + (px_y - cy) ** 2 <= r ** 2
        else:  # polygon
            pts = np.asarray(zone["points_px"], dtype=float)
            occ |= _points_in_polygon(px_x, px_y, pts)

    return occ, grid_step_px


def _points_in_polygon(px_x, px_y, poly):
    """Ray-casting vector hoa: True neu (px_x,px_y) nam trong da giac poly."""
    n = len(poly)
    inside = np.zeros(px_x.shape, dtype=bool)
    x, y = px_x, px_y
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        intersect = ((yi > y) != (yj > y)) & (
            x < (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi
        )
        inside ^= intersect
        j = i
    return inside


def inflate(occ, radius_cells):
    """Phinh vat can them radius_cells o grid (an toan bay), dung max-filter don gian."""
    if radius_cells <= 0:
        return occ
    try:
        from scipy import ndimage
        struct = ndimage.generate_binary_structure(2, 2)
        return ndimage.binary_dilation(occ, structure=struct, iterations=int(radius_cells))
    except ImportError:
        out = occ.copy()
        h, w = occ.shape
        r = int(radius_cells)
        ys, xs = np.where(occ)
        for y, x in zip(ys, xs):
            y0, y1 = max(0, y - r), min(h, y + r + 1)
            x0, x1 = max(0, x - r), min(w, x + r + 1)
            out[y0:y1, x0:x1] = True
        return out


def astar_grid(occ, start_cell, goal_cell):
    """
    A* 8-huong tren luoi occupancy occ (True=vat can). start_cell/goal_cell
    la (row, col). Tra ve list[(row,col)] tu start toi goal, hoac None neu
    khong co duong (bi chan hoan toan, hoac start/goal nam trong vat can).
    """
    h, w = occ.shape
    sr, sc = start_cell
    gr, gc = goal_cell
    if not (0 <= sr < h and 0 <= sc < w and 0 <= gr < h and 0 <= gc < w):
        return None
    if occ[sr, sc] or occ[gr, gc]:
        return None

    neighbors = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
                 (-1, -1, np.sqrt(2)), (-1, 1, np.sqrt(2)),
                 (1, -1, np.sqrt(2)), (1, 1, np.sqrt(2))]

    def heuristic(r, c):
        return np.hypot(r - gr, c - gc)

    open_heap = [(heuristic(sr, sc), 0.0, (sr, sc))]
    came_from = {}
    g_score = {(sr, sc): 0.0}
    visited = set()

    while open_heap:
        _, g, current = heapq.heappop(open_heap)
        if current in visited:
            continue
        visited.add(current)
        if current == (gr, gc):
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path

        cr, cc = current
        for dr, dc, cost in neighbors:
            nr, nc = cr + dr, cc + dc
            if not (0 <= nr < h and 0 <= nc < w) or occ[nr, nc]:
                continue
            tentative = g + cost
            if tentative < g_score.get((nr, nc), np.inf):
                g_score[(nr, nc)] = tentative
                came_from[(nr, nc)] = current
                heapq.heappush(open_heap, (tentative + heuristic(nr, nc), tentative, (nr, nc)))

    return None


def simplify_path_rdp(points, epsilon=2.0):
    """Ramer-Douglas-Peucker: rut gon duong gap khuc, giu sai so <= epsilon (don vi cung voi points)."""
    points = np.asarray(points, dtype=float)
    if len(points) < 3:
        return points

    def _rdp(pts):
        if len(pts) < 3:
            return pts
        start, end = pts[0], pts[-1]
        line_vec = end - start
        line_len = np.linalg.norm(line_vec)
        if line_len < 1e-9:
            dists = np.linalg.norm(pts - start, axis=1)
        else:
            line_unit = line_vec / line_len
            proj = np.outer(np.dot(pts - start, line_unit), line_unit)
            dists = np.linalg.norm((pts - start) - proj, axis=1)
        idx = int(np.argmax(dists))
        if dists[idx] > epsilon:
            left = _rdp(pts[: idx + 1])
            right = _rdp(pts[idx:])
            return np.vstack([left[:-1], right])
        return np.array([start, end])

    return _rdp(points)


def plan_path_pixels(image_shape, no_fly_zones, start_px, goal_px,
                      grid_step_px=4, safety_margin_px=10, simplify_eps_px=6.0):
    """
    HAM CHINH goi tu GUI: (anh, vung cam bay, diem bat dau, diem ket thuc,
    toa do PIXEL) -> list[(u, v)] waypoint pixel da ne vat can + rut gon.

    Tra ve None neu khong tim duoc duong (bao nguoi dung tren GUI).
    """
    occ, step = rasterize_no_fly_zones(image_shape, no_fly_zones, grid_step_px)
    occ = inflate(occ, radius_cells=max(1, int(round(safety_margin_px / step))))

    def to_cell(px):
        u, v = px
        return int(round(v / step)), int(round(u / step))

    start_cell = to_cell(start_px)
    goal_cell = to_cell(goal_px)

    path_cells = astar_grid(occ, start_cell, goal_cell)
    if path_cells is None:
        return None

    path_px = np.array([[c * step, r * step] for r, c in path_cells], dtype=float)
    # Dam bao diem dau/cuoi CHINH XAC nhu nguoi dung click (khong bi lech do grid)
    path_px[0] = start_px
    path_px[-1] = goal_px

    simplified = simplify_path_rdp(path_px, epsilon=simplify_eps_px)
    return simplified.tolist()

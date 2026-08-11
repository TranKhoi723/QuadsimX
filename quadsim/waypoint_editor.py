#!/usr/bin/env python3
"""
waypoint_editor.py
=====================
SCRIPT DOC LAP (chay ngoai terminal, KHONG phai module import trong package) -
mo 1 anh 2D (so do mat bang / ban do top-down), cho phep:

    - CLICK CHUOT TRAI de dat waypoint tai vi tri do.
    - Nhan 'u' de UNDO waypoint vua dat.
    - Nhan 'd' de TU DONG PHAT HIEN vung cam bay (chuong ngai vat) tren anh
      bang threshold do sang/toi - CHUA tu dong ne, chi PHAT HIEN va luu lai
      toa do vung do vao file JSON de dung cho buoc phat trien sau.
    - Nhan 's' de LUU waypoint + vung cam bay ra file .json (dung duoc thang
      voi waypoint_io.load_waypoints_json()).
    - Nhan 'q' de THOAT.

CHAY:
    python waypoint_editor.py <duong_dan_anh> [--scale 0.05] [--altitude 1.5]

    --scale     : so met tren 1 pixel (BAT BUOC hieu chinh dung voi ban do
                  thuc te cua ban - vd anh 800x600px ung voi can phong 40m x
                  30m thi scale = 40/800 = 0.05 m/px).
    --altitude  : do cao mac dinh (m) gan cho moi waypoint vua click (co the
                  sua tay sau trong file JSON xuat ra).
    --origin    : toa do pixel (u,v) duoc coi la goc (0,0) cua the gioi. Mac
                  dinh la GIUA anh. Vi du: --origin 0,0 neu muon goc toa do
                  o goc tren-trai anh.

Sau khi luu file .json, nap vao mo phong bang:
    from quadsim.waypoint_io import load_waypoints_json
    from quadsim.controllers import WaypointManager
    waypoints, calib, no_fly_zones = load_waypoints_json("my_waypoints.json")
    wm = WaypointManager(waypoints, epsilon=0.15)
"""

import argparse
import sys

import numpy as np
import matplotlib
matplotlib.use("TkAgg")  # backend co cua so tuong tac; doi sang khac neu may ban khong ho tro
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from matplotlib.patches import Circle

from quadsim.waypoint_io import ImageCalibration, save_waypoints_json


def detect_no_fly_zones(image_gray, dark_threshold=60, min_area_px=200):
    """
    Phat hien vung cam bay DON GIAN: coi cac vung PIXEL TOI (gia tri < nguong)
    la chuong ngai vat/vung khong bay duoc - phu hop voi so do mat bang kieu
    "tuong den tren nen trang". Dung scipy neu co (ket qua tot hon, gom nhom
    connected-component) hoac fallback tu viet bang numpy neu khong co scipy.

    Tra ve: list[dict] dang {"type": "polygon", "points_px": [[u,v], ...]}
    - moi phan tu la BAO LOI (convex hull) don gian cua 1 vung toi lien thong.

    Day CHI la phat hien - chua dung de tu dong ne trong controller. Ket qua
    duoc luu vao file JSON de danh cho buoc phat trien path-planning sau nay.
    """
    mask = image_gray < dark_threshold

    try:
        from scipy import ndimage
        labeled, n_labels = ndimage.label(mask)
    except ImportError:
        print("  [canh bao] khong co scipy - dung thuat toan flood-fill don gian "
              "(cham hon, ket qua tuong duong voi anh nho).")
        labeled, n_labels = _flood_fill_label(mask)

    zones = []
    for label_id in range(1, n_labels + 1):
        ys, xs = np.where(labeled == label_id)
        if len(xs) < min_area_px:
            continue
        points = np.column_stack([xs, ys])
        hull_points = _convex_hull(points)
        zones.append({"type": "polygon", "points_px": hull_points.tolist()})

    return zones


def _flood_fill_label(mask):
    """Fallback tu viet (khong can scipy) de gom nhom vung True lien thong 4-huong."""
    labeled = np.zeros(mask.shape, dtype=int)
    current_label = 0
    h, w = mask.shape
    for i0 in range(h):
        for j0 in range(w):
            if mask[i0, j0] and labeled[i0, j0] == 0:
                current_label += 1
                stack = [(i0, j0)]
                while stack:
                    i, j = stack.pop()
                    if i < 0 or i >= h or j < 0 or j >= w:
                        continue
                    if not mask[i, j] or labeled[i, j] != 0:
                        continue
                    labeled[i, j] = current_label
                    stack.extend([(i + 1, j), (i - 1, j), (i, j + 1), (i, j - 1)])
    return labeled, current_label


def _convex_hull(points):
    """Convex hull 2D bang Andrew's monotone chain (khong can scipy)."""
    points = np.unique(points, axis=0)
    if len(points) < 3:
        return points

    points = points[np.lexsort((points[:, 1], points[:, 0]))]

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower = []
    for p in points:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)

    upper = []
    for p in points[::-1]:
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)

    return np.array(lower[:-1] + upper[:-1])


class WaypointEditor:
    """Trang thai + xu ly su kien cho phien lam viec click-chon-waypoint."""

    def __init__(self, image_path, calib, default_altitude_m):
        self.image_path = image_path
        self.image = mpimg.imread(image_path)
        self.calib = calib
        self.default_altitude_m = default_altitude_m

        self.pixel_waypoints = []   # list[dict {'pixel':[u,v]}]
        self.no_fly_zones = []      # list[dict] (xem detect_no_fly_zones)
        self.markers = []           # cac artist matplotlib da ve, de undo duoc
        self.zone_patches = []

        self.fig, self.ax = plt.subplots(figsize=(10, 8))
        self.ax.imshow(self.image)
        self.ax.set_title(self._title_text())
        self.fig.canvas.mpl_connect("button_press_event", self._on_click)
        self.fig.canvas.mpl_connect("key_press_event", self._on_key)

    def _title_text(self):
        return (
            f"Waypoints: {len(self.pixel_waypoints)}  |  "
            f"No-fly zones: {len(self.no_fly_zones)}\n"
            "Click trai: dat diem | u: undo | d: phat hien vung cam bay | "
            "s: luu JSON | q: thoat"
        )

    def _on_click(self, event):
        if event.inaxes != self.ax or event.button != 1:
            return
        u, v = event.xdata, event.ydata
        if u is None or v is None:
            return

        idx = len(self.pixel_waypoints)
        self.pixel_waypoints.append({"pixel": [u, v], "alt_m": self.default_altitude_m})

        marker, = self.ax.plot(u, v, marker="o", color="lime", markersize=9,
                                markeredgecolor="black")
        label = self.ax.annotate(str(idx), (u, v), color="white", fontsize=9,
                                  fontweight="bold", ha="center", va="center")
        self.markers.append((marker, label))

        x_world, y_world = self.calib.pixel_to_world((u, v))
        print(f"  [+] Waypoint {idx}: pixel=({u:.1f}, {v:.1f}) "
              f"-> world=({x_world:.2f} m, {y_world:.2f} m), "
              f"alt={self.default_altitude_m} m")

        self._refresh()

    def _on_key(self, event):
        if event.key == "u":
            self._undo()
        elif event.key == "d":
            self._detect_zones()
        elif event.key == "s":
            self._save()
        elif event.key == "q":
            plt.close(self.fig)

    def _undo(self):
        if not self.pixel_waypoints:
            print("  (khong con waypoint nao de undo)")
            return
        self.pixel_waypoints.pop()
        marker, label = self.markers.pop()
        marker.remove()
        label.remove()
        print("  [-] Da xoa waypoint cuoi.")
        self._refresh()

    def _detect_zones(self):
        print("  Dang phat hien vung cam bay (pixel toi)...")
        if self.image.ndim == 3:
            gray = (0.299 * self.image[..., 0] + 0.587 * self.image[..., 1]
                    + 0.114 * self.image[..., 2])
            if gray.max() <= 1.0:  # anh doc bang matplotlib co the o thang [0,1]
                gray = gray * 255.0
        else:
            gray = self.image.astype(float)
            if gray.max() <= 1.0:
                gray = gray * 255.0

        zones = detect_no_fly_zones(gray)
        self.no_fly_zones = zones

        for patch in self.zone_patches:
            patch.remove()
        self.zone_patches = []

        for zone in zones:
            pts = np.array(zone["points_px"])
            patch = plt.Polygon(pts, closed=True, facecolor="red", alpha=0.3,
                                 edgecolor="red", linewidth=1.5)
            self.ax.add_patch(patch)
            self.zone_patches.append(patch)

        print(f"  -> Phat hien {len(zones)} vung cam bay (chi de luu lai, "
              f"CHUA tu dong ne trong mo phong).")
        self._refresh()

    def _save(self):
        if not self.pixel_waypoints:
            print("  (chua co waypoint nao, khong luu)")
            return
        out_path = input("  Nhap ten file JSON de luu (vd waypoints.json): ").strip()
        if not out_path:
            out_path = "waypoints.json"
        if not out_path.endswith(".json"):
            out_path += ".json"

        save_waypoints_json(
            out_path,
            pixel_waypoints=self.pixel_waypoints,
            calibration=self.calib,
            default_altitude_m=self.default_altitude_m,
            no_fly_zones=self.no_fly_zones,
            image_path=self.image_path,
        )
        print(f"  [OK] Da luu {len(self.pixel_waypoints)} waypoint va "
              f"{len(self.no_fly_zones)} vung cam bay vao '{out_path}'.")

    def _refresh(self):
        self.ax.set_title(self._title_text())
        self.fig.canvas.draw_idle()

    def run(self):
        print("Huong dan: click trai de dat diem, 'u' undo, 'd' phat hien vung "
              "cam bay, 's' luu file JSON, 'q' thoat.")
        plt.show()


def _parse_args():
    parser = argparse.ArgumentParser(
        description="Chon waypoint tren anh 2D bang click chuot + phat hien vung cam bay."
    )
    parser.add_argument("image", help="Duong dan anh (bam do mat bang / ban do top-down).")
    parser.add_argument("--scale", type=float, required=True,
                         help="So met tren 1 pixel, vd 0.05 (BAT BUOC).")
    parser.add_argument("--altitude", type=float, default=1.5,
                         help="Do cao mac dinh (m) cho waypoint vua click (mac dinh 1.5).")
    parser.add_argument("--origin", type=str, default=None,
                         help="Pixel goc toa do 'u,v', vd '400,300'. "
                              "Mac dinh: giua anh.")
    parser.add_argument("--no-flip-y", action="store_true",
                         help="Tat dao truc Y (mac dinh CO dao vi truc pixel "
                              "huong xuong con truc world huong len).")
    return parser.parse_args()


def main():
    args = _parse_args()

    try:
        image = mpimg.imread(args.image)
    except FileNotFoundError:
        print(f"[LOI] Khong tim thay anh: {args.image}")
        sys.exit(1)

    h, w = image.shape[0], image.shape[1]

    if args.origin is not None:
        u0, v0 = (float(v) for v in args.origin.split(","))
    else:
        u0, v0 = w / 2.0, h / 2.0

    calib = ImageCalibration(
        origin_px=(u0, v0),
        scale_m_per_px=args.scale,
        flip_y=not args.no_flip_y,
        image_size_px=(w, h),
        image_path=args.image,
    )

    print(f"Anh: {args.image} ({w}x{h} px)")
    print(f"Hieu chinh: goc toa do tai pixel ({u0:.1f}, {v0:.1f}), "
          f"scale={args.scale} m/px, flip_y={calib.flip_y}")

    editor = WaypointEditor(args.image, calib, args.altitude)
    editor.run()


if __name__ == "__main__":
    main()

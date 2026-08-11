"""
quadsim.color_obstacles
==========================
NHAN DIEN VAT CAN TU ANH VE TINH/HANG KHONG bang PHAN LOAI MAU SAC (HSV) -
KHONG can huan luyen model, chay ngay lap tuc, phu hop lam BUOC 1 truoc khi
nang cap len model AI that (xem docstring cuoi file de biet huong nang cap).

Nguyen tac (kinh nghiem xu ly anh vien tham co ban - "chi so mau don gian"):
    - THUC VAT (cay, co)  : mau XANH LA chiem uu the -> heuristic giong NDVI
      don gian tren anh RGB thuong (khong co kenh hong ngoai NIR).
    - MAT NUOC            : mau XANH DUONG dam, do bao hoa cao, do sang thap-vua.
    - NHA/MAI CONG TRINH  : vung TOI DEU MAU (do bao hoa mau thap) - mai ton,
      mai ngoi sam, bong do nha cao tang.
    - DUONG/BAI DAT TRONG : mau XAM/BE nhat, sang - KHONG coi la vat can.

Day la heuristic (co the sai voi anh dac thu - vd sa mac, tuyet) - dung
threshold (nguong) co the chinh tay qua slider tren GUI. Ket qua tra ve
CUNG DINH DANG "no_fly_zones" (list[dict] polygon toa do PIXEL) nhu
waypoint_editor.detect_no_fly_zones(), nen CAM THANG duoc vao
pathfinding.plan_path_pixels() va GUI hien co, khong can sua gi them.
"""

import numpy as np


def _rgb_to_hsv(rgb):
    """RGB (H,W,3) trong [0,255] -> HSV (H,W,3), H trong [0,360), S,V trong [0,1]."""
    arr = rgb.astype(float) / 255.0
    r, g, b = arr[..., 0], arr[..., 1], arr[..., 2]
    maxc = np.max(arr, axis=-1)
    minc = np.min(arr, axis=-1)
    v = maxc
    delta = maxc - minc
    s = np.where(maxc > 1e-9, delta / np.where(maxc > 1e-9, maxc, 1), 0.0)

    h = np.zeros_like(maxc)
    mask = delta > 1e-9
    rc = np.zeros_like(maxc); gc = np.zeros_like(maxc); bc = np.zeros_like(maxc)
    d = np.where(mask, delta, 1)
    rc = (maxc - r) / d
    gc = (maxc - g) / d
    bc = (maxc - b) / d

    h = np.where(mask & (maxc == r), (bc - gc), h)
    h = np.where(mask & (maxc == g), 2.0 + rc - bc, h)
    h = np.where(mask & (maxc == b), 4.0 + gc - rc, h)
    h = (h / 6.0) % 1.0
    h = h * 360.0
    return np.stack([h, s, v], axis=-1)


def classify_obstacle_mask(image_rgb, classes=("vegetation", "water", "dark_building"),
                            veg_hue_range=(60, 170), veg_min_sat=0.15,
                            water_hue_range=(170, 260), water_min_sat=0.2, water_max_val=0.75,
                            dark_max_val=0.35, dark_max_sat=0.35):
    """
    Phan loai TUNG PIXEL cua anh RGB thanh vat can/khong-vat-can theo mau.

    Tra ve: mask (H,W) bool - True = pixel duoc coi la VAT CAN (khong bay qua duoc).
    """
    hsv = _rgb_to_hsv(np.asarray(image_rgb))
    hue, sat, val = hsv[..., 0], hsv[..., 1], hsv[..., 2]

    mask = np.zeros(hue.shape, dtype=bool)

    if "vegetation" in classes:
        veg = (hue >= veg_hue_range[0]) & (hue <= veg_hue_range[1]) & (sat >= veg_min_sat)
        mask |= veg

    if "water" in classes:
        water = (hue >= water_hue_range[0]) & (hue <= water_hue_range[1]) & \
                (sat >= water_min_sat) & (val <= water_max_val)
        mask |= water

    if "dark_building" in classes:
        dark = (val <= dark_max_val) & (sat <= dark_max_sat)
        mask |= dark

    return mask


def detect_no_fly_zones_from_color(image_rgb, classes=("vegetation", "water", "dark_building"),
                                    min_area_px=200, **kwargs):
    """
    HAM CHINH goi tu GUI: anh RGB (vd anh ve tinh) -> list[no_fly_zone] dang
    polygon, CUNG DINH DANG voi waypoint_editor.detect_no_fly_zones() nen
    dung duoc thang trong pathfinding.plan_path_pixels() va GUI hien co.

    kwargs duoc chuyen tiep cho classify_obstacle_mask() (nguong mau tuy chinh).
    """
    from .waypoint_editor import _flood_fill_label, _convex_hull

    mask = classify_obstacle_mask(image_rgb, classes=classes, **kwargs)

    try:
        from scipy import ndimage
        labeled, n_labels = ndimage.label(mask)
    except ImportError:
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


# --------------------------------------------------------------------------
# HUONG NANG CAP LEN MODEL AI THAT (khi heuristic mau khong du chinh xac)
# --------------------------------------------------------------------------
# Heuristic HSV o tren la buoc "co ngay khong can huan luyen". Khi can do
# chinh xac cao hon (anh phuc tap, nhieu bong, nhieu loai vat can), co the
# thay the classify_obstacle_mask() bang 1 model segmentation that, tham
# khao cac repo mo (đã kiem tra con hoat dong):
#
#   - mapbox/robosat (github.com/mapbox/robosat)
#         Segmentation anh ve tinh/hang khong theo tile kieu Slippy Map,
#         tach duoc buildings/roads/water - PHU HOP NHAT neu ban co anh
#         ve tinh thuc su (khong phai so do ve tay).
#
#   - ayushdabra/drone-images-semantic-segmentation
#     santurini/aerial-view-segmentation
#         UNet huan luyen tren "Semantic Drone Dataset" (anh nadir tu drone,
#         24 lop bao gom nguoi/xe/cay/nuoc/mai nha) - PHU HOP neu ban co
#         anh chup THUC TE tu drone (khong phai anh ve tinh do phan giai
#         thap tu Google Maps).
#
# Cach tich hop: viet 1 ham segment_obstacles_ml(image_rgb) -> mask (H,W)
# bool, roi thay classify_obstacle_mask() trong detect_no_fly_zones_from_color()
# bang ham nay - phan con lai (gom nhom vung, tinh convex hull, xuat JSON,
# nap vao A*) GIU NGUYEN, khong phai sua GUI hay pathfinding.py.

"""
quadsim.waypoint_io
======================
DOC/GHI waypoint dang file JSON + CHUYEN DOI toa do pixel (tren anh 2D)
sang toa do the gioi (World frame, don vi met, quy uoc NED z am = do cao
duong) ma cac module con lai (controllers.WaypointManager, simulate) dang
dung.

Muc tieu: waypoint KHONG con hard-code trong code Python - co the:
    1. Viet tay 1 file .json roi load_waypoints_json() nap vao.
    2. Dung waypoint_editor.py de CLICK CHON DIEM tren anh, luu ra .json.
    3. Sinh waypoint tu bat ky nguon nao khac (script rieng, GUI khac...)
       mien la ra dung dinh dang JSON quy dinh o duoi.

------------------------------------------------------------------------
DINH DANG FILE JSON
------------------------------------------------------------------------
{
  "calibration": {
      "image_path": "map.png",       // (tuy chon, chi de tham khao)
      "image_size_px": [800, 600],   // [width, height] pixel luc hieu chinh
      "origin_px": [400, 300],       // pixel duoc coi la goc toa do (0,0)
      "scale_m_per_px": 0.05,        // 1 pixel = bao nhieu met
      "flip_y": true                 // true: truc Y anh (xuong duoi) bi
                                      // DAO NGUOC so voi truc Y the gioi
                                      // (anh: y tang xuong duoi; the gioi
                                      //  thuong ve: y tang len tren)
  },
  "default_altitude_m": 1.5,          // do cao mac dinh (m, SO DUONG) neu
                                       // 1 waypoint khong ghi rieng "alt_m"
  "waypoints": [
      {"pixel": [400, 300], "alt_m": 1.5, "yaw_deg": 0},
      {"pixel": [520, 220], "alt_m": 2.0},
      {"world": [3.0, 1.5, -2.0], "yaw_deg": 90}   // co the ghi thang world
  ],
  "no_fly_zones": [                    // (tuy chon) vung cam bay phat hien
      {"type": "circle", "center_px": [450, 350], "radius_px": 40},
      {"type": "polygon", "points_px": [[100,100],[150,100],[150,150],[100,150]]}
  ]
}

Waypoint co the khai bao bang "pixel" (se duoc quy doi qua "calibration")
HOAC bang "world" (dung thang, khong quy doi) - tien cho truong hop mix
diem lay tu anh voi diem nhap tay bang toa do that.
"""

import json
import numpy as np


class ImageCalibration:
    """
    Luu thong tin quy doi pixel anh (u, v) -> toa do the gioi (x, y) [met].

    Quy uoc:
        - origin_px  : pixel duoc coi la (x=0, y=0) trong the gioi.
        - scale_m_per_px : so met tren 1 pixel (gia dinh ty le DONG DEU 2
          truc - du cho ban do/so do mat bang don gian; neu can ty le khac
          nhau cho x/y, sua truc tiep pixel_to_world ben duoi).
        - flip_y     : anh co truc v (hang pixel) tang dan XUONG DUOI, trong
          khi the gioi thuong quy uoc y tang len - flip_y=True se dao dau
          truc y khi quy doi.
    """

    def __init__(self, origin_px, scale_m_per_px, flip_y=True, image_size_px=None,
                 image_path=None):
        if scale_m_per_px <= 0:
            raise ValueError(f"scale_m_per_px ({scale_m_per_px}) phai > 0.")
        self.origin_px = np.asarray(origin_px, dtype=float)
        self.scale = float(scale_m_per_px)
        self.flip_y = bool(flip_y)
        self.image_size_px = tuple(image_size_px) if image_size_px is not None else None
        self.image_path = image_path

    def pixel_to_world(self, pixel_xy):
        """(u, v) pixel -> (x, y) met, theo goc toa do va ty le da hieu chinh."""
        u, v = pixel_xy
        dx = (u - self.origin_px[0]) * self.scale
        dv = (v - self.origin_px[1]) * self.scale
        dy = -dv if self.flip_y else dv
        return float(dx), float(dy)

    def world_to_pixel(self, world_xy):
        """Chieu nguoc - huu ich khi ve lai waypoint (world) len anh de kiem tra."""
        x, y = world_xy
        dv = -y if self.flip_y else y
        u = self.origin_px[0] + x / self.scale
        v = self.origin_px[1] + dv / self.scale
        return float(u), float(v)

    def to_dict(self):
        d = {
            "origin_px": list(self.origin_px),
            "scale_m_per_px": self.scale,
            "flip_y": self.flip_y,
        }
        if self.image_size_px is not None:
            d["image_size_px"] = list(self.image_size_px)
        if self.image_path is not None:
            d["image_path"] = self.image_path
        return d

    @classmethod
    def from_dict(cls, d):
        return cls(
            origin_px=d["origin_px"],
            scale_m_per_px=d["scale_m_per_px"],
            flip_y=d.get("flip_y", True),
            image_size_px=d.get("image_size_px"),
            image_path=d.get("image_path"),
        )


def _resolve_waypoint(entry, calib, default_altitude_m):
    """1 phan tu trong 'waypoints' (dict co 'pixel' hoac 'world') -> dict chuan
    {'pos': [x,y,z], 'yaw': rad} dung duoc thang cho WaypointManager."""
    if "world" in entry:
        x, y, z = entry["world"]
    elif "pixel" in entry:
        if calib is None:
            raise ValueError(
                "Waypoint dung 'pixel' nhung file JSON khong co muc 'calibration'."
            )
        x, y = calib.pixel_to_world(entry["pixel"])
        alt = entry.get("alt_m", default_altitude_m)
        z = -alt  # NED: do cao duong -> z am
    else:
        raise ValueError(f"Waypoint {entry} phai co 'pixel' hoac 'world'.")

    yaw_deg = entry.get("yaw_deg", 0.0)
    result = {"pos": [float(x), float(y), float(z)], "yaw": float(np.deg2rad(yaw_deg))}
    if "epsilon" in entry and entry["epsilon"] is not None:
        result["epsilon"] = float(entry["epsilon"])
    return result


def load_waypoints_json(path):
    """
    Doc file JSON (dinh dang o dau module) -> tra ve
    (waypoints, calibration, no_fly_zones):
        waypoints    : list[dict] dang {'pos':[x,y,z], 'yaw': rad} - dua
                       THANG vao controllers.WaypointManager(waypoints).
        calibration  : ImageCalibration hoac None (neu file khong co muc
                       'calibration', chi dung duoc waypoint kieu 'world').
        no_fly_zones : list[dict] nguyen dang trong JSON (chua xu ly gi -
                       de danh cho buoc phat trien ne chuong ngai sau nay).
    """
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)

    if "waypoints" not in data or not data["waypoints"]:
        raise ValueError(f"File '{path}' khong co waypoint nao trong muc 'waypoints'.")

    calib = ImageCalibration.from_dict(data["calibration"]) if "calibration" in data else None
    default_altitude_m = data.get("default_altitude_m", 1.5)

    waypoints = [_resolve_waypoint(e, calib, default_altitude_m) for e in data["waypoints"]]
    no_fly_zones = data.get("no_fly_zones", [])

    return waypoints, calib, no_fly_zones


def save_waypoints_json(path, pixel_waypoints, calibration, default_altitude_m=1.5,
                         no_fly_zones=None, image_path=None):
    """
    Ghi file JSON tu danh sach diem PIXEL vua chon (vd tu waypoint_editor.py).

    Tham so:
        path             : duong dan file .json se ghi ra.
        pixel_waypoints  : list[dict] dang {'pixel':[u,v], 'alt_m':.., 'yaw_deg':..}
                           (alt_m/yaw_deg co the bo qua de dung mac dinh).
        calibration      : ImageCalibration da dung de quy doi cac diem tren.
        default_altitude_m : do cao mac dinh (m) ghi vao file cho cac diem
                              khong chi dinh rieng 'alt_m'.
        no_fly_zones     : list[dict] vung cam bay (tuy chon), xem dinh dang
                            o dau module.
        image_path       : duong dan anh goc (chi de ghi chu, khong bat buoc).
    """
    calib_dict = calibration.to_dict()
    if image_path is not None:
        calib_dict["image_path"] = image_path

    data = {
        "calibration": calib_dict,
        "default_altitude_m": default_altitude_m,
        "waypoints": pixel_waypoints,
        "no_fly_zones": no_fly_zones or [],
    }
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)
    return path

"""
quadsim.osm_obstacles
========================
LAY VAT CAN THAT (nha, cay, nuoc) tu OpenStreetMap qua Overpass API - CHINH
XAC HON NHIEU so voi doan mau/do sang tren anh (khong bi nham vi du bai co
sang mau xanh nhat, hay san thuong toi mau), MIEN LA ban biet TOA DO GPS
(kinh do/vi do - longitude/latitude) cua vung anh dang xet.

Yeu cau: ket noi Internet (Overpass API cong khai, KHONG can API key) va
thu vien `requests` (co san hoac `pip install requests`).

Dung khi nao:
    - Ban co anh chup tu Google Maps/vien tham VOI toa do GPS cua 2 goc anh
      (lay bang cach zoom Google Maps, chuot phai -> "Toa do nay" tai goc
      tren-trai va goc duoi-phai cua anh).
    - Neu KHONG biet toa do GPS, dung quadsim.color_obstacles (doan theo
      mau) hoac click tay tren GUI thay the.

Luong xu ly:
    1. fetch_osm_obstacles(bbox) : goi Overpass API, tra ve danh sach
       building/vat can dang toa do GPS (lat, lon).
    2. GeoCalibration : quy doi (lat, lon) <-> pixel anh, GIONG VAI TRO cua
       waypoint_io.ImageCalibration nhung dua tren GPS thay vi scale m/px
       don gian - dung khi anh CO gan voi thuc te dia ly (anh ve tinh),
       khac voi so do ve tay (dung ImageCalibration binh thuong).
    3. osm_to_pixel_zones(...) : ket hop 2 buoc tren -> list[no_fly_zone]
       CUNG DINH DANG voi waypoint_editor.detect_no_fly_zones(), dung
       THANG duoc trong pathfinding.plan_path_pixels() va GUI hien co.
"""

import math
import numpy as np

OVERPASS_URL = "https://overpass-api.de/api/interpreter"

# Cac loai vat the OpenStreetMap duoc coi la "vat can" mac dinh - co the
# them/bot tuy nhu cau (xem danh sach the OSM tai wiki.openstreetmap.org).
DEFAULT_OSM_FILTERS = [
    'way["building"]',            # nha, cong trinh
    'way["natural"="water"]',     # ao ho
    'way["natural"="wood"]',      # rung cay
    'way["landuse"="forest"]',    # dat rung
]


class GeoCalibration:
    """
    Quy doi (lat, lon) <-> pixel anh, gia dinh anh la 1 khung CHU NHAT theo
    kinh/vi do (KHONG xoay, KHONG bien dang phoi canh - dung cho anh ve
    tinh/Google Maps chup thang tu tren xuong, sai so nho voi vung nho).

    Tham so:
        top_left     : (lat, lon) cua GOC TREN-TRAI anh (pixel (0,0)).
        bottom_right : (lat, lon) cua GOC DUOI-PHAI anh (pixel (W,H)).
        image_size_px: (W, H) kich thuoc anh, pixel.
    """

    def __init__(self, top_left, bottom_right, image_size_px):
        self.lat0, self.lon0 = top_left
        self.lat1, self.lon1 = bottom_right
        self.w, self.h = image_size_px

    def latlon_to_pixel(self, lat, lon):
        u = (lon - self.lon0) / (self.lon1 - self.lon0) * self.w
        v = (lat - self.lat0) / (self.lat1 - self.lat0) * self.h
        return float(u), float(v)

    def pixel_to_latlon(self, u, v):
        lon = self.lon0 + (u / self.w) * (self.lon1 - self.lon0)
        lat = self.lat0 + (v / self.h) * (self.lat1 - self.lat0)
        return float(lat), float(lon)

    def bbox(self):
        """(south, west, north, east) - dung cho query Overpass."""
        south = min(self.lat0, self.lat1)
        north = max(self.lat0, self.lat1)
        west = min(self.lon0, self.lon1)
        east = max(self.lon0, self.lon1)
        return south, west, north, east

    def meters_per_pixel(self):
        """Uoc luong ty le m/px trung binh (dung Haversine) - de dong bo voi
        ImageCalibration.scale_m_per_px khi can chuyen sang toa do the gioi
        cua bo mo phong (x,y met)."""
        R = 6371000.0

        def hav(lat1, lon1, lat2, lon2):
            p1, p2 = math.radians(lat1), math.radians(lat2)
            dphi = math.radians(lat2 - lat1)
            dlmb = math.radians(lon2 - lon1)
            a = math.sin(dphi / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dlmb / 2) ** 2
            return 2 * R * math.asin(math.sqrt(a))

        width_m = hav(self.lat0, self.lon0, self.lat0, self.lon1)
        height_m = hav(self.lat0, self.lon0, self.lat1, self.lon0)
        return float((width_m / self.w + height_m / self.h) / 2.0)


def fetch_osm_obstacles(bbox, filters=None, timeout=60):
    """
    Goi Overpass API lay cac 'way' (duong bao da giac) trong bbox
    (south, west, north, east) khop voi filters (mac dinh: nha/nuoc/rung).

    Tra ve: list[dict] {"tags": {...}, "points_latlon": [[lat,lon], ...]}

    CAN INTERNET - chi chay tren MAY CUA BAN (khong chay duoc trong moi
    truong sandbox cua Claude), va can `pip install requests`.
    """
    try:
        import requests
    except ImportError as exc:
        raise ImportError(
            "Can thu vien 'requests' de goi Overpass API: pip install requests"
        ) from exc

    filters = filters or DEFAULT_OSM_FILTERS
    south, west, north, east = bbox
    bbox_str = f"{south},{west},{north},{east}"

    query_parts = "\n".join(f"  {f}({bbox_str});" for f in filters)
    query = f"""
    [out:json][timeout:{timeout}];
    (
{query_parts}
    );
    out body;
    >;
    out skel qt;
    """

    resp = requests.post(OVERPASS_URL, data={"data": query}, timeout=timeout)
    resp.raise_for_status()
    data = resp.json()

    nodes = {el["id"]: (el["lat"], el["lon"]) for el in data["elements"] if el["type"] == "node"}
    ways = []
    for el in data["elements"]:
        if el["type"] != "way" or "nodes" not in el:
            continue
        pts = [nodes[n] for n in el["nodes"] if n in nodes]
        if len(pts) < 3:
            continue
        ways.append({"tags": el.get("tags", {}), "points_latlon": pts})

    return ways


def osm_to_pixel_zones(osm_ways, geo_calib):
    """Chuyen list tra ve tu fetch_osm_obstacles() (toa do lat/lon) sang
    dinh dang no_fly_zones (toa do PIXEL) dung duoc voi pathfinding.py."""
    zones = []
    for way in osm_ways:
        pts_px = [geo_calib.latlon_to_pixel(lat, lon) for lat, lon in way["points_latlon"]]
        zones.append({"type": "polygon", "points_px": [[float(u), float(v)] for u, v in pts_px]})
    return zones


def detect_no_fly_zones_from_osm(geo_calib, filters=None, timeout=60):
    """
    HAM CHINH goi tu GUI/script: GeoCalibration (da biet toa do GPS 2 goc
    anh) -> list[no_fly_zone] dang pixel, dung THANG duoc trong
    pathfinding.plan_path_pixels() va GUI hien co.
    """
    ways = fetch_osm_obstacles(geo_calib.bbox(), filters=filters, timeout=timeout)
    return osm_to_pixel_zones(ways, geo_calib)

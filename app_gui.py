#!/usr/bin/env python3
"""
app_gui.py
============
GIAO DIEN WEB (Streamlit) cho QUADSIM - cho phep:

    1. Tai anh (so do mat bang / anh ve tinh) len.
    2. Hieu chinh ty le pixel<->met.
    3. BON nguon nhan dien vung cam bay:
         a) Nguong do sang (so do ve tay).
         b) Phan loai mau HSV (anh ve tinh: cay/nuoc/nha).
         c) OpenStreetMap that (can toa do GPS).
         d) SAM click-to-segment (MobileSAM) - click chuot khoanh vung BAT KY
            vat the nao, khong can GPS, khong can dieu kien mau sac dac thu.
    4. Hai che do dat waypoint: thu cong (click tung diem) hoac tu dong
       (click diem dau/cuoi -> A* tu ne vat can).
    5. Bang chinh sua epsilon / do cao / yaw RIENG cho tung waypoint.
    6. Mo phong vong kin (Cascade PID + WaypointManager) va truc quan hoa
       ket qua: quy dao ve tren anh goc, do thi vi tri/goc/dong co, cac chi
       so bay (quang duong, toc do, thoi diem toi tung waypoint).
    7. Xuat/nhap file waypoints.json.

Chay:
    streamlit run app_gui.py
"""

import io
import json
import os

import numpy as np
import streamlit as st
from PIL import Image, ImageDraw, ImageFont

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from quadsim.params import get_preset, PRESET_NAMES
from quadsim.controllers import CascadePIDController, WaypointManager
from quadsim.simulate import simulate_waypoints
from quadsim.waypoint_io import ImageCalibration
from quadsim.pathfinding import plan_path_pixels
from quadsim import sam_obstacles
from quadsim.metrics import (
    compute_run_metrics, score_config, compare_epsilon_values,
)
from quadsim.plotting import plot_waypoint_radius_xy

_FONT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "assets", "fonts")


@st.cache_resource(show_spinner=False)
def _load_fonts():
    """Nap font TTF dong goi san (DejaVu Sans) de ve nhan tren anh - PIL
    dung font bitmap mac dinh se hien SAI dau tieng Viet (vd 'BẮT ĐẦU' bi
    vo chu). Dong goi san font trong assets/fonts/ de chay dung tren MOI
    may, khong phu thuoc font co san cua he dieu hanh nguoi dung."""
    try:
        regular = ImageFont.truetype(os.path.join(_FONT_DIR, "DejaVuSans.ttf"), 13)
        bold = ImageFont.truetype(os.path.join(_FONT_DIR, "DejaVuSans-Bold.ttf"), 13)
        return regular, bold
    except Exception:
        return ImageFont.load_default(), ImageFont.load_default()

try:
    from streamlit_image_coordinates import streamlit_image_coordinates
    HAS_CLICK = True
except ImportError:
    HAS_CLICK = False


# ============================================================
# CAU HINH TRANG + CSS TUY CHINH (giao dien chuyen nghiep hon)
# ============================================================
st.set_page_config(page_title="QuadSim — Waypoint Planner", page_icon="🚁", layout="wide")

st.markdown("""
<style>
    #MainMenu {visibility: hidden;}
    footer {visibility: hidden;}

    .qs-hero {
        background: linear-gradient(120deg, #1e3a8a 0%, #2563EB 55%, #0891b2 100%);
        padding: 1.4rem 1.8rem; border-radius: 14px; margin-bottom: 1.1rem;
        color: white; display: flex; align-items: center; justify-content: space-between;
    }
    .qs-hero h1 { margin: 0; font-size: 1.55rem; font-weight: 700; }
    .qs-hero p { margin: 0.15rem 0 0 0; font-size: 0.92rem; opacity: 0.9; }
    .qs-badge {
        background: rgba(255,255,255,0.18); border: 1px solid rgba(255,255,255,0.35);
        padding: 0.25rem 0.7rem; border-radius: 999px; font-size: 0.78rem; font-weight: 600;
        white-space: nowrap;
    }

    .qs-status-row { display: flex; gap: 0.6rem; margin-bottom: 1rem; flex-wrap: wrap; }
    .qs-status-chip {
        flex: 1; min-width: 150px; background: #F8FAFC; border: 1px solid #E2E8F0;
        border-radius: 10px; padding: 0.55rem 0.9rem;
    }
    .qs-status-chip .label { font-size: 0.72rem; color: #64748B; font-weight: 600; text-transform: uppercase; letter-spacing: 0.03em; }
    .qs-status-chip .value { font-size: 1.05rem; color: #0F172A; font-weight: 700; margin-top: 0.1rem; }
    .qs-status-chip.ok { border-color: #86EFAC; background: #F0FDF4; }
    .qs-status-chip.warn { border-color: #FDE68A; background: #FFFBEB; }

    .qs-mode-banner {
        border-radius: 10px; padding: 0.55rem 0.9rem; margin-bottom: 0.6rem;
        font-size: 0.88rem; font-weight: 600; border: 1px solid;
    }
    .qs-mode-manual { background: #EFF6FF; border-color: #BFDBFE; color: #1D4ED8; }
    .qs-mode-auto { background: #F5F3FF; border-color: #DDD6FE; color: #6D28D9; }
    .qs-mode-sam { background: #FFF7ED; border-color: #FED7AA; color: #C2410C; }
    .qs-mode-disabled { background: #F1F5F9; border-color: #E2E8F0; color: #64748B; }

    section[data-testid="stSidebar"] .stButton button { width: 100%; }
    div[data-testid="stMetricValue"] { font-size: 1.4rem; }
</style>
""", unsafe_allow_html=True)


@st.cache_resource(show_spinner=False)
def _load_sam_segmenter():
    """Load model MobileSAM 1 LAN DUY NHAT, dung chung cho moi lan rerun cua
    Streamlit (tranh load lai model ~1s + phai ma hoa lai anh moi lan bam nut)."""
    return sam_obstacles.SAMObstacleSegmenter()


# ============================================================
# SESSION STATE
# ============================================================
def _init_state():
    defaults = {
        "image": None,
        "image_bytes_key": None,
        "waypoints": [],
        "no_fly_zones": [],
        "mode": "manual",
        "auto_start_px": None,
        "auto_goal_px": None,
        "last_click": None,
        "sim_result": None,
        "sam_points": [],
        "sam_labels": [],
        "sam_preview_mask": None,
        "sam_click_label": 1,
        "draw_points": [],
    }
    for k, v in defaults.items():
        if k not in st.session_state:
            st.session_state[k] = v


_init_state()
sam_ready = sam_obstacles.is_available() and sam_obstacles.checkpoint_exists()

# Gia tri mac dinh cho cac nut bam (mot so nut CHI duoc dinh nghia co dieu
# kien trong sidebar tuy theo detect_source/mode dang chon - khoi tao False
# truoc de phan xu ly ben duoi luon tham chieu an toan, khong bi NameError).
detect_zones_btn = False
clear_zones_btn = False
undo_btn = False
clear_wp_btn = False
reset_auto_btn = False
sam_new_zone_btn = False
sam_confirm_btn = False
draw_confirm_btn = False
draw_undo_pt_btn = False


# ============================================================
# HERO HEADER
# ============================================================
st.markdown("""
<div class="qs-hero">
    <div>
        <h1>🚁 QuadSim — Waypoint Planner</h1>
        <p>Mô phỏng động lực học quadcopter · lập kế hoạch bay né vật cản · giao diện tương tác bằng chuột</p>
    </div>
    <div class="qs-badge">v1.1 · Research Prototype</div>
</div>
""", unsafe_allow_html=True)


# ============================================================
# SIDEBAR
# ============================================================
with st.sidebar:
    st.markdown("### ⚙️ Bảng điều khiển")

    with st.expander("📁 1. Dữ liệu đầu vào", expanded=True):
        uploaded = st.file_uploader("Tải ảnh sơ đồ / bản đồ", type=["png", "jpg", "jpeg", "bmp"])
        if uploaded is not None:
            key = uploaded.name + str(uploaded.size)
            if st.session_state["image_bytes_key"] != key:
                st.session_state["image"] = Image.open(uploaded).convert("RGB")
                st.session_state["image_bytes_key"] = key
                st.session_state["waypoints"] = []
                st.session_state["no_fly_zones"] = []
                st.session_state["auto_start_px"] = None
                st.session_state["auto_goal_px"] = None
                st.session_state["sim_result"] = None
                st.session_state["sam_points"] = []
                st.session_state["sam_labels"] = []
                st.session_state["sam_preview_mask"] = None
                st.session_state["draw_points"] = []

        if not HAS_CLICK:
            st.error(
                "Thiếu gói `streamlit-image-coordinates`. Cài bằng:\n\n"
                "`pip install streamlit-image-coordinates`"
            )

        scale_m_per_px = st.number_input("Tỉ lệ (m / pixel)", min_value=0.0001,
                                          value=0.05, step=0.01, format="%.4f",
                                          help="Ví dụ ảnh rộng 800px ứng với 40m thực tế → 40/800 = 0.05")
        default_alt = st.number_input("Độ cao mặc định (m)", min_value=0.1, value=1.5, step=0.1)
        default_epsilon = st.number_input("Epsilon mặc định (m)", min_value=0.05, value=0.2, step=0.05,
                                           help="Bán kính coi như 'đã tới' 1 waypoint. Có thể ghi đè riêng cho từng điểm ở bảng waypoint.")

    with st.expander("🚧 2. Vùng cấm bay (vật cản)", expanded=True):
        detect_source = st.selectbox(
            "Nguồn phát hiện vật cản",
            ["Sơ đồ đơn giản (ngưỡng tối)", "Ảnh vệ tinh (màu: cây/nước/nhà)",
             "OpenStreetMap (cần toạ độ GPS)", "✂️ SAM click-to-segment",
             "✏️ Tự vẽ (click từng đỉnh)"],
        )

        if detect_source == "Sơ đồ đơn giản (ngưỡng tối)":
            dark_threshold = st.slider("Ngưỡng tối (0-255)", 0, 255, 60)
            min_area_px = st.number_input("Diện tích tối thiểu (px²)", min_value=10, value=200, step=50)

        elif detect_source == "Ảnh vệ tinh (màu: cây/nước/nhà)":
            veg_on = st.checkbox("🌳 Cây / thảm thực vật (xanh lá)", value=True)
            water_on = st.checkbox("💧 Mặt nước (xanh dương)", value=True)
            dark_on = st.checkbox("🏠 Mái nhà / công trình tối màu", value=True)
            min_area_px = st.number_input("Diện tích tối thiểu (px²)", min_value=10, value=200, step=50)

        elif detect_source == "OpenStreetMap (cần toạ độ GPS)":
            st.caption("Toạ độ GPS 2 góc ảnh (lấy từ Google Maps: chuột phải → xem toạ độ).")
            tl_lat = st.number_input("Vĩ độ góc trên-trái", value=10.7800, format="%.6f")
            tl_lon = st.number_input("Kinh độ góc trên-trái", value=106.6950, format="%.6f")
            br_lat = st.number_input("Vĩ độ góc dưới-phải", value=10.7750, format="%.6f")
            br_lon = st.number_input("Kinh độ góc dưới-phải", value=106.7050, format="%.6f")

        else:  # SAM
            if not sam_obstacles.is_available():
                st.error(
                    "Thiếu thư viện cho SAM. Cài bằng:\n\n"
                    "`pip install torch torchvision opencv-python-headless timm`\n\n"
                    "`pip install \"git+https://github.com/ChaoningZhang/MobileSAM.git\"`"
                )
            elif not sam_obstacles.checkpoint_exists():
                st.warning("Chưa có model MobileSAM (~40MB).")
                if st.button("⬇️ Tải model MobileSAM (1 lần)"):
                    with st.spinner("Đang tải MobileSAM (~40MB)..."):
                        try:
                            sam_obstacles.download_checkpoint()
                            st.success("Đã tải xong — chọn lại nguồn SAM để dùng.")
                            st.rerun()
                        except Exception as exc:
                            st.error(f"Tải thất bại: {exc}")
            else:
                st.caption(
                    "Click lên ảnh để khoanh vùng: điểm ➕ = THUỘC vật cản, "
                    "điểm ➖ = LOẠI TRỪ (sửa khi vùng tô lan sai)."
                )
                label_choice = st.radio(
                    "Nhãn cho lần click tiếp theo", ["➕ Thuộc vật cản", "➖ Loại trừ"], horizontal=True,
                )
                st.session_state["sam_click_label"] = 1 if label_choice.startswith("➕") else 0
                n_pts = len(st.session_state["sam_points"])
                st.caption(f"Số điểm đã đặt cho vùng hiện tại: **{n_pts}**")
                c_sam1, c_sam2 = st.columns(2)
                sam_new_zone_btn = c_sam1.button("🆕 Vùng mới", use_container_width=True)
                sam_confirm_btn = c_sam2.button(
                    "✅ Xác nhận", use_container_width=True,
                    disabled=st.session_state["sam_preview_mask"] is None,
                )

        if detect_source.startswith("✏️"):
            st.caption(
                "Click lần lượt từng đỉnh của vùng cấm bay trên ảnh (ít nhất 3 điểm), "
                "rồi bấm **Chốt vùng** để đóng đa giác. Dùng khi vật cản không nhận ra "
                "được bằng màu/ngưỡng tối, hoặc bạn muốn khoanh tay cho chính xác."
            )
            n_draw_pts = len(st.session_state["draw_points"])
            st.caption(f"Số đỉnh đã đặt cho vùng hiện tại: **{n_draw_pts}**")
            c_dr1, c_dr2, c_dr3 = st.columns(3)
            draw_undo_pt_btn = c_dr1.button("↩️ Bỏ đỉnh cuối", use_container_width=True,
                                             disabled=n_draw_pts == 0)
            draw_confirm_btn = c_dr2.button("✅ Chốt vùng", use_container_width=True,
                                             disabled=n_draw_pts < 3)
            draw_clear_pts_btn = c_dr3.button("🆕 Vùng mới", use_container_width=True,
                                               disabled=n_draw_pts == 0)
            if draw_clear_pts_btn:
                st.session_state["draw_points"] = []

        c_det1, c_det2 = st.columns(2)
        detect_zones_btn = c_det1.button("🔍 Phát hiện", use_container_width=True,
                                          disabled=detect_source.startswith("✂️") or detect_source.startswith("✏️"))
        clear_zones_btn = c_det2.button("🧹 Xoá hết", use_container_width=True)

    with st.expander("📍 3. Waypoint", expanded=True):
        mode_label = st.radio(
            "Chế độ đặt điểm",
            ["🖱️ Thủ công: click từng waypoint", "🤖 Tự động: click điểm đầu/cuối, né vật cản"],
        )
        st.session_state["mode"] = "manual" if mode_label.startswith("🖱️") else "auto"

        if st.session_state["mode"] == "auto":
            safety_margin_px = st.slider("Biên an toàn quanh vật cản (px)", 0, 60, 12)
            simplify_eps_px = st.slider("Đơn giản hoá đường đi (px)", 1, 30, 6)
            grid_step_px = st.slider("Độ phân giải lưới A* (px/ô)", 2, 16, 4)
            if st.session_state["auto_start_px"] is None:
                st.info("👉 Click điểm BẮT ĐẦU trên ảnh.")
            elif st.session_state["auto_goal_px"] is None:
                st.info("👉 Click điểm ĐÍCH trên ảnh.")
            reset_auto_btn = st.button("↺ Chọn lại điểm đầu/cuối", use_container_width=True)
        else:
            reset_auto_btn = False

        c_wp1, c_wp2 = st.columns(2)
        undo_btn = c_wp1.button("↩️ Undo", use_container_width=True)
        clear_wp_btn = c_wp2.button("🗑️ Xoá hết", use_container_width=True)

    with st.expander("💾 4. Xuất / nhập JSON", expanded=False):
        export_name = st.text_input("Tên file xuất", value="waypoints.json")
        uploaded_json = st.file_uploader("Nhập lại từ file .json", type=["json"], key="json_up")

    st.markdown("---")
    st.markdown("### 🚁 5. Mô phỏng")
    preset_name = st.selectbox("Preset drone", PRESET_NAMES)
    sim_dt = st.number_input("Bước tích phân dt (s)", min_value=0.001, value=0.01, step=0.005)
    sim_t_final = st.number_input("Thời gian mô phỏng tối đa (s)", min_value=1.0, value=40.0, step=5.0)
    run_sim_btn = st.button("▶️  Chạy mô phỏng", type="primary", use_container_width=True)


# ============================================================
# TRANG CHINH
# ============================================================
if st.session_state["image"] is None:
    st.info(
        "👈 **Bắt đầu:** tải một ảnh sơ đồ mặt bằng hoặc ảnh vệ tinh ở mục "
        "**1. Dữ liệu đầu vào** trên thanh bên trái."
    )
    with st.expander("ℹ️ Hướng dẫn nhanh", expanded=True):
        st.markdown("""
1. **Tải ảnh** bản đồ / mặt bằng khu vực bay, hiệu chỉnh tỉ lệ mét/pixel.
2. **Phát hiện vật cản** — chọn 1 trong 4 nguồn (ngưỡng tối, màu sắc, OpenStreetMap, hoặc SAM click-to-segment).
3. **Đặt waypoint** — click thủ công từng điểm, hoặc click điểm đầu/cuối để hệ thống tự sinh đường né vật cản.
4. **Chỉnh từng waypoint** (độ cao, hướng, epsilon) ở bảng bên phải.
5. **Chạy mô phỏng** để xem quỹ đạo bay thực tế bám theo waypoint.
        """)
    st.stop()

img = st.session_state["image"]
w, h = img.size
calib = ImageCalibration(origin_px=(w / 2.0, h / 2.0), scale_m_per_px=scale_m_per_px,
                          flip_y=True, image_size_px=(w, h))


# ----------------------------------------------------------------------
# XU LY NUT BAM (LOGIC)
# ----------------------------------------------------------------------
if detect_zones_btn:
    if detect_source == "Sơ đồ đơn giản (ngưỡng tối)":
        gray = np.array(img.convert("L"), dtype=float)
        from quadsim.waypoint_editor import detect_no_fly_zones
        zones = detect_no_fly_zones(gray, dark_threshold=dark_threshold, min_area_px=int(min_area_px))
        st.session_state["no_fly_zones"] = zones
        st.toast(f"Phát hiện {len(zones)} vùng cấm bay (ngưỡng tối).", icon="🔍")

    elif detect_source == "Ảnh vệ tinh (màu: cây/nước/nhà)":
        from quadsim.color_obstacles import detect_no_fly_zones_from_color
        classes = tuple(c for c, on in [("vegetation", veg_on), ("water", water_on),
                                         ("dark_building", dark_on)] if on)
        if not classes:
            st.sidebar.warning("Chưa chọn loại vật cản nào để nhận diện.")
        else:
            rgb = np.array(img.convert("RGB"))
            zones = detect_no_fly_zones_from_color(rgb, classes=classes, min_area_px=int(min_area_px))
            st.session_state["no_fly_zones"] = zones
            st.toast(f"Phát hiện {len(zones)} vùng cấm bay (phân loại màu).", icon="🎨")

    elif detect_source == "OpenStreetMap (cần toạ độ GPS)":
        try:
            from quadsim.osm_obstacles import GeoCalibration, detect_no_fly_zones_from_osm
            geo_calib = GeoCalibration((tl_lat, tl_lon), (br_lat, br_lon), (w, h))
            with st.spinner("Đang tải dữ liệu vật cản từ OpenStreetMap..."):
                zones = detect_no_fly_zones_from_osm(geo_calib)
            st.session_state["no_fly_zones"] = zones
            st.toast(f"Tải được {len(zones)} vùng cấm bay từ OpenStreetMap.", icon="🌍")
        except ImportError as exc:
            st.sidebar.error(f"Thiếu thư viện: {exc}\nCài bằng: pip install requests")
        except Exception as exc:
            st.sidebar.error(
                f"Lỗi khi tải dữ liệu OSM: {exc}\n"
                "Kiểm tra: (1) máy có Internet, (2) toạ độ GPS đã nhập đúng, "
                "(3) Overpass API có thể đang quá tải, thử lại sau vài giây."
            )

if clear_zones_btn:
    st.session_state["no_fly_zones"] = []

if clear_wp_btn:
    st.session_state["waypoints"] = []
    st.session_state["sim_result"] = None

if undo_btn and st.session_state["waypoints"]:
    st.session_state["waypoints"].pop()
    st.session_state["sim_result"] = None

if reset_auto_btn:
    st.session_state["auto_start_px"] = None
    st.session_state["auto_goal_px"] = None

if sam_new_zone_btn:
    st.session_state["sam_points"] = []
    st.session_state["sam_labels"] = []
    st.session_state["sam_preview_mask"] = None

if sam_confirm_btn and st.session_state["sam_preview_mask"] is not None:
    new_zones = sam_obstacles.SAMObstacleSegmenter.mask_to_zone(st.session_state["sam_preview_mask"])
    st.session_state["no_fly_zones"].extend(new_zones)
    st.session_state["sam_points"] = []
    st.session_state["sam_labels"] = []
    st.session_state["sam_preview_mask"] = None
    st.toast("Đã thêm vùng vật cản mới từ SAM.", icon="✂️")

if draw_undo_pt_btn and st.session_state["draw_points"]:
    st.session_state["draw_points"].pop()

if draw_confirm_btn and len(st.session_state["draw_points"]) >= 3:
    st.session_state["no_fly_zones"].append({
        "type": "polygon",
        "points_px": [list(p) for p in st.session_state["draw_points"]],
        "source": "manual_draw",
    })
    st.session_state["draw_points"] = []
    st.toast("Đã thêm vùng vật cản tự vẽ.", icon="✏️")

if uploaded_json is not None:
    try:
        data = json.load(uploaded_json)
        wps = []
        for e in data.get("waypoints", []):
            if "pixel" in e:
                u, v = e["pixel"]
            else:
                u, v = calib.world_to_pixel((e["world"][0], e["world"][1]))
            wps.append({
                "pixel": [float(u), float(v)],
                "alt_m": float(e.get("alt_m", data.get("default_altitude_m", default_alt))),
                "yaw_deg": float(e.get("yaw_deg", 0.0)),
                "epsilon_m": float(e.get("epsilon", default_epsilon)),
            })
        st.session_state["waypoints"] = wps
        st.session_state["no_fly_zones"] = data.get("no_fly_zones", [])
        st.sidebar.success(f"Đã nhập {len(wps)} waypoint từ file.")
    except Exception as exc:
        st.sidebar.error(f"Lỗi đọc file JSON: {exc}")


# ----------------------------------------------------------------------
# THANH TRANG THAI NHANH
# ----------------------------------------------------------------------
n_zones = len(st.session_state["no_fly_zones"])
n_wps = len(st.session_state["waypoints"])
st.markdown(f"""
<div class="qs-status-row">
    <div class="qs-status-chip ok"><div class="label">Ảnh</div><div class="value">{w}×{h} px</div></div>
    <div class="qs-status-chip {'ok' if n_zones else 'warn'}"><div class="label">Vùng cấm bay</div><div class="value">{n_zones}</div></div>
    <div class="qs-status-chip {'ok' if n_wps else 'warn'}"><div class="label">Waypoint</div><div class="value">{n_wps}</div></div>
    <div class="qs-status-chip"><div class="label">Tỉ lệ</div><div class="value">{scale_m_per_px:.3f} m/px</div></div>
</div>
""", unsafe_allow_html=True)


# ----------------------------------------------------------------------
# VE ANH + OVERLAY (waypoint, vung cam bay, duong di, preview SAM)
# ----------------------------------------------------------------------
def render_overlay_image(img, waypoints, no_fly_zones, auto_start_px=None, auto_goal_px=None,
                          sam_mask=None, sam_points=None, sam_labels=None, draw_points=None):
    canvas = img.copy().convert("RGBA")
    font_regular, font_bold = _load_fonts()

    if sam_mask is not None:
        overlay_arr = np.zeros((canvas.size[1], canvas.size[0], 4), dtype=np.uint8)
        overlay_arr[sam_mask] = [255, 140, 0, 120]
        mask_img = Image.fromarray(overlay_arr, mode="RGBA")
        canvas = Image.alpha_composite(canvas, mask_img)

    draw = ImageDraw.Draw(canvas, "RGBA")

    def label_with_bg(xy, text, fill, font=font_bold, pad=2):
        """Ve chu co nen mo phia sau de luon doc duoc du anh nen sang hay toi."""
        bbox = draw.textbbox(xy, text, font=font)
        draw.rectangle([bbox[0] - pad, bbox[1] - pad, bbox[2] + pad, bbox[3] + pad],
                        fill=(0, 0, 0, 140))
        draw.text(xy, text, fill=fill, font=font)

    for zone in no_fly_zones:
        if zone.get("type") == "circle":
            cx, cy = zone["center_px"]
            r = zone["radius_px"]
            draw.ellipse([cx - r, cy - r, cx + r, cy + r], fill=(255, 0, 0, 70), outline=(255, 0, 0, 200), width=2)
        else:
            pts = [tuple(p) for p in zone["points_px"]]
            if len(pts) >= 3:
                draw.polygon(pts, fill=(255, 0, 0, 70), outline=(255, 0, 0, 200))

    if sam_points:
        for (px, py), lbl in zip(sam_points, sam_labels or []):
            if lbl == 1:
                draw.line([(px - 7, py), (px + 7, py)], fill=(0, 160, 0, 255), width=3)
                draw.line([(px, py - 7), (px, py + 7)], fill=(0, 160, 0, 255), width=3)
            else:
                draw.line([(px - 7, py - 7), (px + 7, py + 7)], fill=(220, 0, 0, 255), width=3)
                draw.line([(px - 7, py + 7), (px + 7, py - 7)], fill=(220, 0, 0, 255), width=3)

    if draw_points:
        pts = [tuple(p) for p in draw_points]
        if len(pts) >= 2:
            draw.line(pts, fill=(255, 165, 0, 255), width=2)
        for i, (px, py) in enumerate(pts):
            r = 5
            draw.ellipse([px - r, py - r, px + r, py + r],
                         fill=(255, 165, 0, 230), outline=(0, 0, 0, 255), width=1)
            label_with_bg((px + 7, py - 7), str(i + 1), fill=(255, 200, 0, 255))
        if len(pts) >= 3:
            draw.line([pts[-1], pts[0]], fill=(255, 165, 0, 140), width=1)

    for i in range(len(waypoints) - 1):
        p1 = tuple(waypoints[i]["pixel"])
        p2 = tuple(waypoints[i + 1]["pixel"])
        draw.line([p1, p2], fill=(30, 144, 255, 220), width=3)

    for i, wp in enumerate(waypoints):
        u, v = wp["pixel"]
        eps_px = wp.get("epsilon_m", default_epsilon) / max(scale_m_per_px, 1e-9)
        if eps_px > 3:
            draw.ellipse([u - eps_px, v - eps_px, u + eps_px, v + eps_px],
                          outline=(50, 220, 50, 200), width=2)
        r = 9
        draw.ellipse([u - r, v - r, u + r, v + r], fill=(50, 200, 50, 235), outline=(10, 90, 10, 255), width=2)
        label_with_bg((u + 11, v - 9), str(i), fill=(255, 255, 255, 255))

    if auto_start_px is not None:
        u, v = auto_start_px
        draw.ellipse([u - 9, v - 9, u + 9, v + 9], fill=(0, 200, 255, 230), outline=(0, 60, 90, 255), width=2)
        label_with_bg((u + 11, v - 9), "BẮT ĐẦU", fill=(0, 220, 255, 255))
    if auto_goal_px is not None:
        u, v = auto_goal_px
        draw.ellipse([u - 9, v - 9, u + 9, v + 9], fill=(255, 80, 80, 230), outline=(90, 0, 0, 255), width=2)
        label_with_bg((u + 11, v - 9), "ĐÍCH", fill=(255, 100, 100, 255))

    return canvas.convert("RGB")


# ----------------------------------------------------------------------
# TABS
# ----------------------------------------------------------------------
tab_plan, tab_result = st.tabs(["🗺️  Thiết lập chuyến bay", "📊  Kết quả mô phỏng"])

with tab_plan:
    col_img, col_table = st.columns([2, 1])

    with col_img:
        sam_active = detect_source.startswith("✂️") and sam_ready
        draw_active = detect_source.startswith("✏️")
        if sam_active:
            label_txt = "➕ Thuộc vật cản" if st.session_state["sam_click_label"] == 1 else "➖ Loại trừ"
            st.markdown(f'<div class="qs-mode-banner qs-mode-sam">✂️ Chế độ SAM — click để khoanh vùng '
                        f'(nhãn hiện tại: {label_txt})</div>', unsafe_allow_html=True)
        elif detect_source.startswith("✂️") and not sam_ready:
            st.markdown('<div class="qs-mode-banner qs-mode-disabled">✂️ SAM chưa sẵn sàng — cài thư viện '
                        'hoặc tải model ở sidebar để dùng chế độ này.</div>', unsafe_allow_html=True)
        elif draw_active:
            st.markdown('<div class="qs-mode-banner qs-mode-manual">✏️ Chế độ tự vẽ — click từng đỉnh của '
                        'vùng cấm bay, rồi bấm "Chốt vùng" ở sidebar.</div>', unsafe_allow_html=True)
        elif st.session_state["mode"] == "manual":
            st.markdown('<div class="qs-mode-banner qs-mode-manual">🖱️ Chế độ thủ công — mỗi click thêm '
                        '1 waypoint theo thứ tự.</div>', unsafe_allow_html=True)
        else:
            stage = "điểm BẮT ĐẦU" if st.session_state["auto_start_px"] is None else "điểm ĐÍCH"
            st.markdown(f'<div class="qs-mode-banner qs-mode-auto">🤖 Chế độ tự động — click {stage} '
                        'trên ảnh.</div>', unsafe_allow_html=True)

        overlay = render_overlay_image(
            img, st.session_state["waypoints"], st.session_state["no_fly_zones"],
            st.session_state["auto_start_px"], st.session_state["auto_goal_px"],
            sam_mask=st.session_state["sam_preview_mask"],
            sam_points=st.session_state["sam_points"], sam_labels=st.session_state["sam_labels"],
            draw_points=st.session_state["draw_points"],
        )

        if HAS_CLICK:
            click = streamlit_image_coordinates(overlay, key="main_click")
        else:
            st.image(overlay, use_container_width=True)
            click = None

        if click is not None and click != st.session_state["last_click"]:
            st.session_state["last_click"] = click
            u, v = float(click["x"]), float(click["y"])

            if draw_active:
                st.session_state["draw_points"].append([u, v])
                st.rerun()

            elif sam_active:
                segmenter = _load_sam_segmenter()
                if not segmenter.is_image_ready(st.session_state["image_bytes_key"]):
                    with st.spinner("Đang mã hoá ảnh cho SAM (một lần, vài giây)..."):
                        segmenter.set_image(np.array(img.convert("RGB")),
                                             image_key=st.session_state["image_bytes_key"])
                st.session_state["sam_points"].append([u, v])
                st.session_state["sam_labels"].append(st.session_state["sam_click_label"])
                mask, score = segmenter.predict(
                    points=st.session_state["sam_points"], labels=st.session_state["sam_labels"])
                st.session_state["sam_preview_mask"] = mask
                st.rerun()

            elif detect_source.startswith("✂️") and not sam_ready:
                st.warning("SAM chưa sẵn sàng — xem hướng dẫn cài đặt ở sidebar.")

            elif st.session_state["mode"] == "manual":
                st.session_state["waypoints"].append({
                    "pixel": [u, v], "alt_m": default_alt, "yaw_deg": 0.0, "epsilon_m": default_epsilon,
                })
                st.session_state["sim_result"] = None
                st.rerun()

            else:
                if st.session_state["auto_start_px"] is None:
                    st.session_state["auto_start_px"] = [u, v]
                    st.rerun()
                elif st.session_state["auto_goal_px"] is None:
                    st.session_state["auto_goal_px"] = [u, v]
                    with st.spinner("Đang tìm đường né vật cản (A*)..."):
                        img_arr_shape = (h, w, 3)
                        path_px = plan_path_pixels(
                            img_arr_shape, st.session_state["no_fly_zones"],
                            start_px=tuple(st.session_state["auto_start_px"]),
                            goal_px=tuple(st.session_state["auto_goal_px"]),
                            grid_step_px=grid_step_px, safety_margin_px=safety_margin_px,
                            simplify_eps_px=simplify_eps_px,
                        )
                    if path_px is None:
                        st.error("❌ Không tìm được đường đi (điểm nằm trong vùng cấm bay, hoặc "
                                  "vùng cấm bay chặn kín hoàn toàn). Hãy chọn lại điểm hoặc giảm biên an toàn.")
                        st.session_state["auto_goal_px"] = None
                    else:
                        new_wps = [{"pixel": p, "alt_m": default_alt, "yaw_deg": 0.0,
                                    "epsilon_m": default_epsilon} for p in path_px]
                        st.session_state["waypoints"].extend(new_wps)
                        st.session_state["sim_result"] = None
                        st.toast(f"Đã sinh {len(new_wps)} waypoint né vật cản.", icon="✅")
                    st.rerun()

    with col_table:
        st.markdown("#### 📋 Danh sách waypoint")
        wps = st.session_state["waypoints"]
        if not wps:
            st.caption("_(chưa có waypoint nào — click lên ảnh bên trái để bắt đầu)_")
        else:
            rows = []
            for i, wp in enumerate(wps):
                xw, yw = calib.pixel_to_world(wp["pixel"])
                rows.append({
                    "#": i, "x (m)": round(xw, 2), "y (m)": round(yw, 2),
                    "Độ cao (m)": wp["alt_m"], "Yaw (deg)": wp["yaw_deg"],
                    "Epsilon (m)": wp["epsilon_m"],
                })
            edited = st.data_editor(
                rows, key="wp_editor", use_container_width=True, num_rows="fixed",
                disabled=["#", "x (m)", "y (m)"], hide_index=True,
            )
            for i, row in enumerate(edited):
                wps[i]["alt_m"] = float(row["Độ cao (m)"])
                wps[i]["yaw_deg"] = float(row["Yaw (deg)"])
                wps[i]["epsilon_m"] = float(row["Epsilon (m)"])

        st.markdown("---")
        if st.button("💾 Lưu waypoints ra JSON", use_container_width=True):
            if not wps:
                st.warning("Chưa có waypoint để lưu.")
            else:
                pixel_wps = [{"pixel": wp["pixel"], "alt_m": wp["alt_m"], "yaw_deg": wp["yaw_deg"],
                              "epsilon": wp["epsilon_m"]} for wp in wps]
                buf = io.StringIO()
                data = {
                    "calibration": calib.to_dict(),
                    "default_altitude_m": default_alt,
                    "waypoints": pixel_wps,
                    "no_fly_zones": st.session_state["no_fly_zones"],
                }
                json.dump(data, buf, indent=2, ensure_ascii=False)
                st.download_button("⬇️ Tải file JSON", data=buf.getvalue(),
                                    file_name=export_name or "waypoints.json", mime="application/json",
                                    use_container_width=True)


# ----------------------------------------------------------------------
# CHAY MO PHONG
# ----------------------------------------------------------------------
if run_sim_btn:
    wps = st.session_state["waypoints"]
    if len(wps) < 1:
        st.error("Cần ít nhất 1 waypoint để mô phỏng.")
    else:
        world_wps = []
        for wp in wps:
            xw, yw = calib.pixel_to_world(wp["pixel"])
            world_wps.append({
                "pos": [xw, yw, -wp["alt_m"]],
                "yaw": float(np.deg2rad(wp["yaw_deg"])),
                "epsilon": float(wp["epsilon_m"]),
            })

        params = get_preset(preset_name)
        ctrl = CascadePIDController(params)
        wm = WaypointManager(world_wps, epsilon=default_epsilon)

        state0 = np.zeros(12)
        state0[0:3] = world_wps[0]["pos"]

        with st.spinner("Đang chạy mô phỏng vòng kín (Cascade PID)..."):
            t, state, omega, saturated, wp_idx = simulate_waypoints(
                ctrl, wm, params, state0=state0, dt=sim_dt, t_final=sim_t_final,
                stop_on_completion=True, settle_time=1.0,
            )
        st.session_state["sim_result"] = {
            "t": t, "state": state, "omega": omega, "saturated": saturated,
            "wp_idx": wp_idx, "switch_log": wm.switch_log, "world_wps": world_wps,
            "params": params, "dt": sim_dt, "preset_name": preset_name,
        }
        st.toast("Mô phỏng hoàn tất — xem tab 'Kết quả mô phỏng'.", icon="🚁")


with tab_result:
    res = st.session_state["sim_result"]
    if res is None:
        st.info("Chưa có kết quả — thiết lập waypoint ở tab bên cạnh rồi bấm **▶️ Chạy mô phỏng** ở sidebar.")
    else:
        t, state = res["t"], res["state"]
        completed = len(res["switch_log"]) >= len(res["world_wps"]) - 1

        dpos = np.diff(state[:, 0:3], axis=0)
        dt_arr = np.diff(t)
        dt_arr[dt_arr == 0] = np.nan
        dist_total = float(np.sum(np.linalg.norm(dpos, axis=1)))
        speed = np.linalg.norm(dpos, axis=1) / dt_arr
        speed = speed[np.isfinite(speed)]
        peak_tilt_deg = float(np.rad2deg(np.abs(state[:, 6:8])).max())

        c1, c2, c3, c4 = st.columns(4)
        c1.metric("Thời gian bay", f"{t[-1]:.1f} s")
        c2.metric("Hoàn thành waypoint", "✅ Có" if completed else "⚠️ Chưa xong")
        c3.metric("Quãng đường bay", f"{dist_total:.1f} m")
        c4.metric("Bão hoà động cơ", f"{res['saturated'].mean()*100:.1f} %")

        c5, c6, c7, _ = st.columns(4)
        c5.metric("Tốc độ trung bình", f"{speed.mean():.2f} m/s" if len(speed) else "—")
        c6.metric("Tốc độ đỉnh", f"{speed.max():.2f} m/s" if len(speed) else "—")
        c7.metric("Góc nghiêng đỉnh", f"{peak_tilt_deg:.1f}°")

        st.markdown("<br>", unsafe_allow_html=True)
        col_a, col_b = st.columns([1, 1])

        with col_a:
            fig1, ax1 = plt.subplots(figsize=(6, 6 * h / w))
            ax1.imshow(img)
            px_traj = np.array([calib.world_to_pixel((x, y)) for x, y in zip(state[:, 0], state[:, 1])])
            ax1.plot(px_traj[:, 0], px_traj[:, 1], color="#00cfff", linewidth=2, label="Quỹ đạo bay")
            for i, wp in enumerate(st.session_state["waypoints"]):
                ax1.scatter(*wp["pixel"], color="lime", edgecolor="black", zorder=5)
            ax1.set_title("Quỹ đạo trên bản đồ")
            ax1.axis("off")
            ax1.legend()
            st.pyplot(fig1)

        with col_b:
            fig2, axes2 = plt.subplots(2, 2, figsize=(9, 6))
            axes2[0, 0].plot(t, state[:, 0], label="x")
            axes2[0, 0].plot(t, state[:, 1], label="y")
            axes2[0, 0].plot(t, -state[:, 2], label="độ cao")
            axes2[0, 0].legend(); axes2[0, 0].set_title("Vị trí (m)"); axes2[0, 0].grid(alpha=0.3)

            axes2[0, 1].plot(t, np.rad2deg(state[:, 6]), label="roll")
            axes2[0, 1].plot(t, np.rad2deg(state[:, 7]), label="pitch")
            axes2[0, 1].plot(t, np.rad2deg(state[:, 8]), label="yaw")
            axes2[0, 1].legend(); axes2[0, 1].set_title("Góc nghiêng (deg)"); axes2[0, 1].grid(alpha=0.3)

            axes2[1, 0].plot(t, res["wp_idx"], color="purple")
            axes2[1, 0].set_title("Chỉ số waypoint đang hướng tới"); axes2[1, 0].grid(alpha=0.3)

            for i in range(4):
                axes2[1, 1].plot(t, res["omega"][:, i], linewidth=0.8, label=f"ω{i+1}")
            axes2[1, 1].legend(fontsize=7); axes2[1, 1].set_title("Tốc độ động cơ (rad/s)"); axes2[1, 1].grid(alpha=0.3)
            for ax in axes2.flat:
                ax.set_xlabel("t (s)")
            fig2.tight_layout()
            st.pyplot(fig2)

        if res["switch_log"]:
            st.markdown("#### 🕐 Thời điểm tới từng waypoint")
            st.dataframe(
                [{"Waypoint": idx, "Thời điểm (s)": round(ts, 2)} for idx, ts in res["switch_log"]],
                use_container_width=True, hide_index=True,
            )

        st.markdown("---")
        st.markdown("#### 🎯 Bán kính waypoint — kiểu hiển thị PX4 (Local Position)")
        st.caption(
            "Đường nét đứt nối các waypoint theo thứ tự bay (giống mission plan của PX4), "
            "vòng tròn cam quanh mỗi waypoint là bán kính chấp nhận (epsilon), và đường xanh "
            "là quỹ đạo bay thực tế — cho thấy trực quan việc bo góc/mượt hoá khi drone đi qua "
            "từng waypoint, thay vì chỉ xem qua bảng số liệu."
        )
        fig_radius, ax_radius = plt.subplots(figsize=(7.5, 7.5))
        plot_waypoint_radius_xy(res["state"], res["world_wps"], ax=ax_radius)
        st.pyplot(fig_radius)

        # ------------------------------------------------------------
        # TIÊU CHÍ ĐÁNH GIÁ CHO BÁO CÁO (metrics.py)
        # ------------------------------------------------------------
        st.markdown("---")
        st.markdown("### 📊 Tiêu chí đánh giá (dùng cho báo cáo thực tập)")

        run_m = compute_run_metrics(
            res["t"], res["state"], res["omega"], res["saturated"],
            res["wp_idx"], res["world_wps"], res["params"], res["dt"],
        )
        score = score_config(run_m)

        m1, m2, m3, m4 = st.columns(4)
        m1.metric("Độ mượt xung ω (Sw)", f"{run_m['motor_smoothness_S']:.3f}",
                   help="1 = ω gần như hằng số (mượt tuyệt đối); 0 = giật cục liên tục. "
                        "Công thức: Sw = exp(-α · RMS(dω/dt)/ω_max).")
        m2.metric("Hiệu suất động cơ (proxy)", f"{run_m['motor_efficiency_proxy']*100:.1f} %",
                   help="ƯỚC LƯỢNG dựa trên độ mượt + mức bão hoà + mức sử dụng quanh vùng hover "
                        "— KHÔNG PHẢI hiệu suất điện thực (mô hình không có động cơ điện/pin).")
        m3.metric("Chỉ số vọt số (TB)", f"{run_m['overshoot_mean_pct']:.1f} %",
                   help="Phần trăm vượt quá setpoint trên trục dịch chuyển chính của mỗi chặng bay, "
                        "trung bình trên các chặng có dữ liệu.")
        m4.metric("Điểm tổng hợp (0-1)", f"{score['total_score']:.3f}",
                   help="Điểm 'tiêu chí chọn' = 0.15·thời gian + 0.30·(1-vọt số) + "
                        "0.35·độ mượt + 0.20·(1-bão hoà). Dùng để so sánh nhiều cấu hình.")

        with st.expander("📐 Công thức & chi tiết từng tiêu chí", expanded=False):
            st.markdown(
                f"""
**1) Độ mượt xung ω / Hiệu suất động cơ (proxy)**

- `Sw = exp(-α · RMS(Δω/Δt) / ω_max)`, α = 20 (mặc định), Sw ∈ (0, 1].
- `η_proxy = 0.5·Sw + 0.3·U + 0.2·SAT`, trong đó U = mức bám vùng hover
  (1 − |ω̄/ω_max − ω_hover/ω_max|), SAT = 1 − tỉ lệ bước bão hoà động cơ.
- ⚠️ Đây là **chỉ số proxy theo cơ học** (không mô phỏng mạch điện/PWM/pin),
  nên khi viết báo cáo cần ghi rõ là "ước lượng hiệu suất theo độ mượt tín
  hiệu điều khiển", không phải hiệu suất điện η = P_cơ/P_điện đo thực tế.

**2) Chỉ số vọt số (overshoot)**

- Với mỗi chặng bay (waypoint i-1 → i), lấy trục có biên độ dịch chuyển lớn
  nhất làm trục chính, rồi tính `overshoot % = (đỉnh vượt − đích)/biên độ × 100`
  — cùng công thức đã dùng trong `tune_altitude_gain.py::step_metrics()`,
  chỉ tổng quát hoá từ 1 trục (độ cao) sang toàn bộ hành trình nhiều waypoint.
- Vọt số TB hiện tại: **{run_m['overshoot_mean_pct']:.1f}%**, lớn nhất:
  **{run_m['overshoot_max_pct']:.1f}%**, đánh giá trên **{run_m['_leg_table'].__len__()}**
  chặng có dịch chuyển đủ lớn để tính.

**3) Tiêu chí chọn (điểm tổng hợp)**

| Thành phần | Trọng số | Điểm (0-1) |
|---|---|---|
| Thời gian bay | 15% | {score['time_score']:.3f} |
| Vọt số | 30% | {score['overshoot_score']:.3f} |
| Độ mượt động cơ | 35% | {score['smoothness_score']:.3f} |
| Không bão hoà | 20% | {score['saturation_score']:.3f} |
| **Tổng** | 100% | **{score['total_score']:.3f}** |
"""
            )

        if run_m["_leg_table"]:
            st.markdown("##### Vọt số theo từng chặng bay")
            st.dataframe(
                [{"Chặng (→ waypoint)": r["leg"], "Trục chính": r["axis"],
                  "Biên độ (m)": round(r["amplitude_m"], 2),
                  "Vọt quá (m)": round(r["overshoot_m"], 3),
                  "Vọt số (%)": round(r["overshoot_pct"], 1)} for r in run_m["_leg_table"]],
                use_container_width=True, hide_index=True,
            )

        st.markdown("##### 🔬 Khảo sát ảnh hưởng bán kính waypoint (epsilon) đến độ mượt")
        st.caption(
            "Chạy lại mô phỏng với cùng waypoint + cùng gain PID nhưng nhiều giá trị "
            "epsilon khác nhau, để trả lời câu hỏi: bán kính waypoint lớn/nhỏ ảnh hưởng "
            "thế nào đến tốc độ mượt của động cơ, vọt số, và thời gian bay."
        )
        eps_text = st.text_input("Danh sách epsilon cần so sánh (m, cách nhau bằng dấu phẩy)",
                                  value="0.1, 0.2, 0.3, 0.5, 0.8, 1.2")
        if st.button("▶️ Chạy khảo sát epsilon"):
            try:
                eps_values = [float(v.strip()) for v in eps_text.split(",") if v.strip()]
            except ValueError:
                eps_values = []
                st.error("Danh sách epsilon không hợp lệ — dùng số cách nhau bằng dấu phẩy.")
            if eps_values:
                wps_no_eps = [{"pos": wp["pos"], "yaw": wp["yaw"]} for wp in res["world_wps"]]
                with st.spinner(f"Đang chạy {len(eps_values)} lần mô phỏng..."):
                    rows = compare_epsilon_values(
                        eps_values,
                        controller_factory=lambda: CascadePIDController(res["params"]),
                        waypoints_no_epsilon=wps_no_eps,
                        params=res["params"], dt=res["dt"], t_final=30.0,
                    )
                st.dataframe(
                    [{"Epsilon (m)": r["epsilon"], "Thời gian bay (s)": round(r["flight_time_s"], 2),
                      "Vọt số TB (%)": round(r["overshoot_mean_pct"], 1),
                      "Độ mượt Sw": round(r["motor_smoothness_S"], 3),
                      "Bão hoà (%)": round(r["saturation_pct"], 1),
                      "Điểm tổng hợp": round(r["total_score"], 3)} for r in rows],
                    use_container_width=True, hide_index=True,
                )
                best = rows[0]
                st.success(
                    f"🏆 Epsilon đề xuất: **{best['epsilon']} m** "
                    f"(điểm tổng hợp {best['total_score']:.3f}) — bán kính waypoint nhỏ "
                    f"thường cho vọt số thấp hơn nhưng dễ làm động cơ dao động (giảm độ "
                    f"mượt) khi bay qua nhiều điểm liên tiếp; bán kính lớn giúp mượt hơn "
                    f"nhưng có thể cắt góc quỹ đạo nhiều hơn — bảng trên là căn cứ định "
                    f"lượng để chọn thay vì ước lượng cảm tính."
                )

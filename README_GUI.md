# QUADSIM — Giao diện GUI (click chuột chọn waypoint)

File mới: **`app_gui.py`** — giao diện web chạy bằng [Streamlit], cho phép
tương tác hoàn toàn bằng chuột, không cần gõ lệnh terminal cho từng bước.

## 1. Cài đặt (chỉ làm 1 lần)

```bash
cd quadsim_project
pip install -r requirements_gui.txt
```

(hoặc cài tay: `pip install streamlit streamlit-image-coordinates matplotlib numpy pillow scipy`)

File này **CHỈ chứa gói cần thiết cho GUI cơ bản** (tải ảnh, click chọn
waypoint, mô phỏng, xem báo cáo chỉ số) — cố tình KHÔNG gồm `torch` /
`torchvision` / `opencv` / `timm` (rất nặng, chỉ cần cho tính năng
"✂️ SAM click-to-segment" tùy chọn), để tránh pip phải hạ cấp Pillow xuống
bản không có sẵn wheel cho Python mới. Nếu muốn dùng SAM, cài thêm:

```bash
pip install -r requirements_gui_sam.txt
pip install "git+https://github.com/ChaoningZhang/MobileSAM.git"
```

`scipy` không bắt buộc (code có fallback tự viết), nhưng có `scipy` thì
phát hiện vùng cấm bay + A* chạy nhanh hơn.

## 2. Chạy GUI

```bash
streamlit run app_gui.py
```

Trình duyệt sẽ tự mở `http://localhost:8501`.

## 3. Cách dùng

### Bước 1 — Tải ảnh
Ở thanh bên trái, mục **"1) Tải ảnh sơ đồ / bản đồ"**, chọn ảnh mặt bằng /
bản đồ top-down của bạn (PNG/JPG).

### Bước 2 — Hiệu chỉnh tỉ lệ
Nhập **"Tỉ lệ (m/pixel)"** — ví dụ ảnh 800px ứng với sảnh rộng 40m thì
tỉ lệ = 40/800 = 0.05. Toạ độ thế giới (x, y mét) sẽ được tự tính từ đây,
gốc toạ độ (0,0) mặc định là **giữa ảnh**.

### Bước 3 — (tuỳ chọn) Phát hiện vật cản
Nhấn **"🔍 Phát hiện vùng cấm bay tự động"** — thuật toán sẽ tô đỏ các
vùng **pixel tối** trên ảnh (ví dụ tường/vật thể vẽ đen trên nền trắng).
Chỉnh **"Ngưỡng tối"** nếu phát hiện sai (ảnh sáng/tối khác nhau).

Nếu ảnh của bạn không có vùng tối rõ ràng để tự phát hiện, bạn vẫn có thể
bỏ qua bước này và tự đặt waypoint né vật cản bằng tay (chế độ thủ công).

### Bước 4 — Chọn chế độ đặt waypoint

**🖱️ Thủ công**: mỗi cú click chuột trên ảnh = 1 waypoint mới, theo đúng
thứ tự bạn click. Dùng khi bạn muốn kiểm soát đường bay chính xác.

**🤖 Tự động (click 2 điểm)**: click lần 1 = điểm **bắt đầu**, click lần 2
= điểm **đích**. Ứng dụng sẽ tự chạy thuật toán **A\*** trên lưới ảnh để
tìm đường ngắn nhất **né các vùng đỏ (vật cản)**, sau đó rút gọn thành vài
waypoint (không sinh hàng trăm điểm vụn vặt). Có 3 thanh trượt:
- **Biên an toàn**: bay cách vật cản bao nhiêu pixel.
- **Độ đơn giản hoá**: đường đi càng đơn giản (ít điểm) thì thanh này càng lớn.
- **Độ phân giải lưới A\***: nhỏ hơn = chính xác hơn nhưng tính lâu hơn.

Nếu 2 điểm bị vật cản chặn kín hoàn toàn, ứng dụng sẽ báo lỗi và yêu cầu
chọn lại.

### Bước 5 — Chỉnh từng waypoint
Ở bảng bên phải, bạn có thể sửa trực tiếp cho **từng waypoint**:
- **Độ cao (m)**
- **Yaw (deg)** — hướng mũi drone
- **Epsilon (m)** — bán kính "coi như đã tới nơi" **RIÊNG cho từng điểm**
  (điểm cần bay chính xác thì để epsilon nhỏ, điểm chỉ đi ngang qua thì để
  epsilon lớn cho mượt).

Vòng tròn xanh lá quanh mỗi waypoint trên ảnh thể hiện đúng bán kính
epsilon đó (vẽ theo đúng tỉ lệ m/pixel).

### Bước 6 — Lưu / nạp lại
- **💾 Lưu waypoints ra JSON**: xuất file `.json` đúng định dạng cũ
  (tương thích `waypoint_editor.py`), có thể nạp lại bằng ô "Nhập lại từ
  file .json" ở sidebar.

### Bước 7 — Chạy mô phỏng
Chọn **preset drone**, `dt`, thời gian mô phỏng tối đa, rồi nhấn
**▶️ Chạy mô phỏng**. Ứng dụng chạy bộ điều khiển **Cascade PID** đầy đủ
(Position → Velocity → Attitude → Rate) bay lần lượt qua các waypoint theo
đúng epsilon đã đặt, rồi hiển thị:
- Quỹ đạo bay vẽ **đè lên chính ảnh bản đồ** của bạn.
- Đồ thị vị trí / góc nghiêng / waypoint đang hướng tới / tốc độ 4 động cơ
  theo thời gian.
- Cảnh báo nếu động cơ bão hoà quá 30% thời gian (có thể do drone quá tải
  hoặc waypoint đặt yêu cầu tăng tốc quá gấp).

## 4. File mới được thêm vào package

| File | Vai trò |
|---|---|
| `quadsim/pathfinding.py` | Thuật toán A* né vật cản trên lưới pixel + rút gọn đường đi (Douglas–Peucker) |
| `quadsim/color_obstacles.py` | Nhận diện vật cản theo màu sắc (cây/nước/nhà) trên ảnh vệ tinh, không cần AI |
| `quadsim/osm_obstacles.py` | Lấy vật cản THẬT từ OpenStreetMap qua Overpass API (cần toạ độ GPS) |
| `quadsim/sam_obstacles.py` | Khoanh vùng vật cản bằng click chuột dùng MobileSAM, không cần GPS/màu sắc |
| `assets/fonts/DejaVuSans*.ttf` | Font đóng gói sẵn để hiển thị đúng dấu tiếng Việt trên ảnh overlay (mọi máy) |
| `app_gui.py` | Giao diện Streamlit chính (đã nâng cấp: tab, banner trạng thái, theme riêng) |
| `quadsim/controllers.py` | (đã sửa) `WaypointManager` giờ hỗ trợ **epsilon riêng từng waypoint** qua key `"epsilon"` |
| `quadsim/waypoint_io.py` | (đã sửa) đọc thêm key `"epsilon"` khi nạp file JSON |

`WaypointManager`, `simulate_waypoints()`, `waypoint_editor.py` (bản click
chuột chạy rời qua matplotlib, không cần Streamlit) vốn đã có sẵn trong
gói bạn tải lên — GUI mới chỉ là lớp giao diện gọi lại đúng các hàm đó,
không viết lại logic tính toán.

## 6. Nhận diện vật cản từ ảnh vệ tinh / bản đồ (mới)

Ngoài kiểu "ngưỡng tối" cũ (chỉ hợp với sơ đồ vẽ tay trắng-đen), sidebar mục
**"3) Vùng cấm bay"** giờ có thêm 2 nguồn:

- **Ảnh vệ tinh (màu: cây/nước/nhà)** — `quadsim/color_obstacles.py`:
  phân loại từng pixel theo màu HSV (xanh lá = cây, xanh dương = nước, tối
  đều màu = mái nhà/công trình). Chạy ngay, không cần Internet, không cần
  huấn luyện model — nhưng là **heuristic**, có thể sai với ảnh đặc thù
  (sa mạc, tuyết...). Có 3 checkbox bật/tắt từng loại + ngưỡng diện tích.

- **OpenStreetMap (cần toạ độ GPS)** — `quadsim/osm_obstacles.py`:
  nếu bạn **biết toạ độ GPS (lat/lon) của 2 góc ảnh** (lấy từ Google Maps:
  chuột phải → xem toạ độ), ứng dụng sẽ gọi Overpass API để lấy **chính
  xác** ranh giới nhà/cây/mặt nước thật từ OpenStreetMap — không đoán màu,
  độ chính xác cao hơn hẳn. Cần máy có Internet và `pip install requests`.

- **✂️ SAM click-to-segment (mới)** — `quadsim/sam_obstacles.py`: dùng
  model **MobileSAM** (bản nhẹ của Segment Anything, Meta AI) để khoanh
  vùng vật cản bằng CHÍNH CÚ CLICK CHUỘT của bạn, không cần toạ độ GPS,
  không phụ thuộc màu sắc — dùng được cho **bất kỳ ảnh nào** (sơ đồ vẽ
  tay, ảnh cũ, ảnh không có vệ tinh). Cách dùng:
  1. Cài thư viện (chỉ cần làm 1 lần, xem mục "Cài đặt tính năng SAM" bên dưới).
  2. Chọn nguồn "✂️ SAM click-to-segment" ở sidebar, bấm "⬇️ Tải model
     MobileSAM (~40MB)" nếu chưa có (chỉ tải 1 lần, lưu vào `weights/`).
  3. Click lên **giữa vật thể** cần khoanh vùng trên ảnh (ví dụ giữa nóc
     nhà) — MobileSAM sẽ tự động vẽ vùng bao quanh vật thể đó, hiện màu
     cam mờ để bạn xem trước.
  4. Nếu vùng tô bị lan sang chỗ không mong muốn: chuyển nhãn sang
     "➖ Loại trừ" rồi click vào đúng chỗ bị lan sai để sửa (có thể click
     nhiều điểm +/- để tinh chỉnh).
  5. Bấm "✅ Xác nhận" để thêm vùng này vào danh sách vật cản, rồi
     "🆕 Vùng mới" để khoanh tiếp vật thể khác.

  Lưu ý: lần **đầu tiên click vào 1 ảnh mới** sẽ mất vài giây để mô hình
  "mã hoá" toàn bộ ảnh (hiện spinner chờ) — các lần click TIẾP THEO trên
  CÙNG ảnh đó chỉ mất chưa đến 0.2 giây vì đã tận dụng lại kết quả mã hoá.

Cả 4 nguồn đều xuất ra đúng định dạng `no_fly_zones` cũ nên cắm thẳng vào
A* (`pathfinding.py`) và toàn bộ GUI hiện có mà không cần sửa gì thêm.

### Cài đặt tính năng SAM (tuỳ chọn)

```bash
pip install torch torchvision opencv-python-headless timm
pip install "git+https://github.com/ChaoningZhang/MobileSAM.git"
```

Sau đó vào sidebar → chọn nguồn "✂️ SAM click-to-segment" → bấm nút tải
model (~40MB, chỉ 1 lần). Nếu không cài các gói trên, toàn bộ phần còn lại
của ứng dụng vẫn hoạt động bình thường — sidebar chỉ hiện hướng dẫn cài đặt
thay vì báo lỗi khó hiểu.

**Vì sao chọn MobileSAM thay vì SAM gốc?** SAM gốc (checkpoint nhỏ nhất
ViT-B cũng ~375MB) chạy khá nặng trên máy không có GPU rời. MobileSAM nhẹ
hơn khoảng 66 lần (~40MB), tốc độ nhanh hơn 5–38 lần, độ chính xác giảm
không đáng kể (~2–4 điểm mIoU) — phù hợp hơn nhiều cho một ứng dụng
Streamlit chạy local trên laptop thông thường.

### Muốn chính xác hơn nữa (nâng cấp bằng AI thật)
`color_obstacles.py` có sẵn phần ghi chú hướng nâng cấp: thay hàm
`classify_obstacle_mask()` bằng 1 model **semantic segmentation** thật.
Vài repo mã nguồn mở đáng tham khảo (mình đã kiểm tra còn hoạt động):

| Repo | Phù hợp khi nào |
|---|---|
| [`mapbox/robosat`](https://github.com/mapbox/robosat) | Bạn có ảnh vệ tinh/hàng không thật (không phải sơ đồ vẽ tay), cần tách building/road/water hàng loạt theo tile kiểu Slippy Map |
| [`ayushdabra/drone-images-semantic-segmentation`](https://github.com/ayushdabra/drone-images-semantic-segmentation) | Ảnh chụp **từ chính drone** (nadir, cao 5–30m), có sẵn UNet huấn luyện trên Semantic Drone Dataset (24 lớp: người, xe, cây, nước, mái nhà...) |
| [`santurini/aerial-view-segmentation`](https://github.com/santurini/aerial-view-segmentation) | Tương tự trên nhưng đã gộp sẵn thành lớp nhị phân "vật cản / bãi đáp an toàn" — gần đúng nhu cầu của bạn nhất |

Cách tích hợp: viết 1 hàm `segment_obstacles_ml(image_rgb) -> mask(H,W) bool`
dùng model đã train từ 1 trong các repo trên, rồi thay vào chỗ gọi
`classify_obstacle_mask()` trong `detect_no_fly_zones_from_color()` — phần
còn lại (gom vùng, tính convex hull, xuất JSON, nạp A*) **giữ nguyên**.

## 7. Về "dẫn hướng né vật cản real-time" trên drone thật

Đây là một dự án **khác hẳn** về quy mô so với `quadsim` (vốn là mô phỏng
Python offline). Nếu sau này bạn muốn làm drone thật tự né vật cản khi
đang bay (không phải lập kế hoạch trước như hiện tại), hướng đi thực tế là:

- [`PX4/PX4-Avoidance`](https://github.com/PX4/PX4-Avoidance) — package
  ROS chính thức của PX4, thuật toán VFH+*/3DVFH*, cắm cảm biến độ sâu
  (depth camera) vào, chạy trên companion computer thật hoặc SITL/Gazebo.
- [`HKUST-Aerial-Robotics/Fast-Planner`](https://github.com/HKUST-Aerial-Robotics/Fast-Planner)
  kết hợp Octomap — lập quỹ đạo né vật cản mượt (minimum-snap) từ point
  cloud cảm biến, có ví dụ tích hợp PX4 tại
  [`deepak-1530/FastPlannerOctomap`](https://github.com/deepak-1530/FastPlannerOctomap).

Cả 2 đều là **C++/ROS**, chạy trên PX4 SITL hoặc phần cứng thật — không
phải nâng cấp từ `quadsim.dynamics`/`controllers.py` hiện tại được, vì đó
là mô phỏng động lực học thuần Python không có flight stack/MAVLink. Nếu
bạn thực sự muốn đi hướng này, nên coi là **dự án 2, tách riêng**, dùng
PX4 SITL + Gazebo làm nền, chứ không cố nhét vào `quadsim`.


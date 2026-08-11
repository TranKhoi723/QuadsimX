# QUADSIM — Phần mềm mô phỏng Quadcopter (Terminal)

Phần mềm mô phỏng Digital Twin quadcopter + điều khiển PID, đóng gói thành
package Python có cấu trúc rõ ràng, giao diện hiện tại là **terminal**
(menu chọn số) — dễ nâng cấp lên GUI sau này vì logic tính toán và lớp
giao diện đã tách riêng hoàn toàn.

## Cài đặt

Chỉ cần Python 3 + `numpy` + `matplotlib` (không cần thư viện nào khác):

```bash
pip install numpy matplotlib
```

## Chạy

```bash
cd quadsim_project
python main.py
```

Sẽ hiện menu:

```
======================================================
   QUADSIM - Phan mem mo phong Quadcopter (Terminal)
======================================================
Drone hien tai: Crazyflie 2.0 (so lieu thuc nghiem cong khai)

  [1] Doi drone (preset)
  [2] Xem thong so drone (summary)
  [3] Chay kich ban VONG HO (leo cao -> roll -> pitch -> yaw)
  [4] Chay kich ban: 1 rotor lech khoi Trim
  [5] Chay dieu khien VONG KIN (Cascade PID)
  [0] Thoat
```

Gõ số rồi Enter. Ở mọi câu hỏi, **Enter (để trống) = dùng giá trị mặc định**
in trong `[...]` — chạy nhanh không cần nhớ số liệu.

Đồ thị luôn được **lưu file PNG** vào thư mục `outputs/` (in đường dẫn ra
màn hình sau khi vẽ xong) — vì terminal không hiển thị hình trực tiếp. Mỗi
lần chạy mục [3]/[4]/[5] sẽ tạo 5 file: `..._1_omega.png` (control input),
`..._2_positions.png`, `..._3_angles.png`, `..._4_full_state.png` (12 biến
trạng thái), `..._5_trajectory_3d.png` (quỹ đạo 3D).

## Cấu trúc code (package `quadsim/`)

```
quadsim/
    params.py        Nhận dạng hệ thống — DroneParams, get_preset()
    mixer.py          Control allocation — Mixer matrix, mixer_signs()
    dynamics.py       Newton-Euler 6-DOF + RK4
    scenarios.py      Kịch bản omega(t) vòng hở (đã SỬA LỖI, xem bên dưới)
    controllers.py    Cascade PID 4 tầng (vòng kín)
    simulate.py       2 vòng lặp: simulate() và simulate_closed_loop()
    plotting.py       Vẽ + lưu 5 loại đồ thị
    cli.py            Menu terminal (lớp giao diện — KHÔNG chứa logic tính toán)
main.py               Điểm vào — chỉ gọi quadsim.cli.run_app()
```

**Nguyên tắc:** mọi phép tính (Mixer, Dynamics, PID...) nằm trong các module
`.py` thuần túy, không phụ thuộc giao diện. `cli.py` chỉ hỏi input rồi gọi lại các hàm đó. Muốn nâng cấp lên giao diện khác (web, GUI desktop...) sau này, chỉ cần viết 1 lớp giao diện mới gọi lại **đúng các hàm này** — không phải viết lại phần tính toán.

## Lỗi đã sửa so với bản trước (`half_sine_pulse` → `smooth_pulse`)

Bản cũ dùng **nửa chu kỳ sin** cho mỗi giai đoạn (leo cao/roll/pitch/yaw) — mô-men chỉ đẩy theo **1 chiều duy nhất** suốt cả đoạn, không có pha hãm lại.
  Hậu quả: roll tích lũy tới **345°** (phi vật lý) trong 1 lần kiểm chứng thật.

`scenarios.smooth_pulse()` trong bản này dùng **1 chu kỳ sin đầy đủ** (nửa
đầu tăng tốc, nửa sau tự hãm lại) — kiểm chứng lại cho kết quả hợp lý
(roll đỉnh ~84°, không còn phát tán vô hạn). Chi tiết kỹ thuật xem docstring đầu file `quadsim/scenarios.py`.

## Dùng như thư viện (không qua terminal)

Mọi hàm đều import trực tiếp được, ví dụ trong Jupyter/script riêng:

```python
from quadsim.params import get_preset
from quadsim.scenarios import luukkonen_scenario
from quadsim.simulate import simulate
from quadsim.plotting import plot_all

params = get_preset("crazyflie")
omega_cmd, t_total = luukkonen_scenario(params)
t, state, omega = simulate(omega_cmd, params, t_final=t_total)
plot_all(t, state, omega, params, output_dir="my_outputs")
```

## Viết kịch bản omega(t) của riêng bạn

```python
from quadsim.mixer import mixer_signs
signs = mixer_signs(params)          # {'roll':.., 'pitch':.., 'yaw':..} dau +/-1 tung rotor

def omega_cmd(t):
    w_h = params.omega_hover
    return w_h + signs["roll"] * 0.02 * w_h * np.sin(2*np.pi*t)   # dao dong roll lien tuc
```

rồi `simulate(omega_cmd, params, t_final=...)` như bình thường.

## Việc còn để ngỏ

- LQR (bộ điều khiển tuyến tính đối chiếu PID) — đã có bản nháp ở phiên
  trước, chưa đưa vào package này.
- Waypoint Manager (quản lý chuỗi điểm chờ) — mới ở giai đoạn thiết kế.
- Amplitude mặc định trong `luukkonen_scenario()` (0.05/0.03/0.03/0.05) là
  ước lượng thủ công để ra góc nghiêng cùng bậc độ lớn với bài báo tham
  khảo — có thể cần tinh chỉnh thêm tùy mục đích trình bày.

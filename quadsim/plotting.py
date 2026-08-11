"""
quadsim.plotting
==================
VE VA LUU DO THI. Vi giao dien hien tai la TERMINAL (khong co man hinh do
thi truc tiep), mac dinh moi ham o day LUU FILE PNG ra thu muc chi dinh
(output_dir) va IN RA duong dan file - nguoi dung tu mo file de xem. Neu
chay trong moi truong co man hinh (vd Jupyter, IDE), truyen show=True de
hien thi truc tiep them.
"""

import os
import numpy as np
import matplotlib
matplotlib.use("Agg")   # an toan cho terminal khong co man hinh
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 - kich hoat projection='3d'


def _save(fig, output_dir, filename, show):
    os.makedirs(output_dir, exist_ok=True)
    path = os.path.join(output_dir, filename)
    fig.savefig(path, dpi=120, bbox_inches="tight")
    print(f"  -> Da luu: {path}")
    if show:
        plt.show()
    plt.close(fig)
    return path


def plot_control_inputs(t, omega, params, output_dir="outputs", show=False, filename="1_omega.png"):
    fig, ax = plt.subplots(figsize=(8, 4.5))
    for i in range(4):
        ax.plot(t, omega[:, i], label=f"$\\omega_{i+1}$")
    ax.set_xlabel("Time t (s)"); ax.set_ylabel("$\\omega_i$ (rad/s)")
    ax.set_title(f"Control inputs — {params.name}")
    ax.legend(); ax.grid(alpha=0.3)
    plt.tight_layout()
    return _save(fig, output_dir, filename, show)


def plot_positions(t, state, params, output_dir="outputs", show=False, filename="2_positions.png",
                    setpoint=None):
    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.plot(t, state[:, 0], label="x")
    ax.plot(t, state[:, 1], label="y")
    ax.plot(t, -state[:, 2], label="do cao")
    if setpoint is not None:
        ax.axhline(-setpoint[2], color="gray", linestyle=":", label="setpoint do cao")
    ax.set_xlabel("Time t (s)"); ax.set_ylabel("Position (m)")
    ax.set_title(f"Positions — {params.name}")
    ax.legend(); ax.grid(alpha=0.3)
    plt.tight_layout()
    return _save(fig, output_dir, filename, show)


def plot_angles(t, state, params, output_dir="outputs", show=False, filename="3_angles.png"):
    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.plot(t, np.rad2deg(state[:, 6]), label="roll")
    ax.plot(t, np.rad2deg(state[:, 7]), label="pitch")
    ax.plot(t, np.rad2deg(state[:, 8]), label="yaw")
    ax.set_xlabel("Time t (s)"); ax.set_ylabel("Angle (deg)")
    ax.set_title(f"Angles — {params.name}")
    ax.legend(); ax.grid(alpha=0.3)
    plt.tight_layout()
    return _save(fig, output_dir, filename, show)


def plot_full_state(t, state, omega, params, output_dir="outputs", show=False,
                     filename="4_full_state.png", title=None):
    fig, axes = plt.subplots(4, 4, figsize=(16, 11))

    axes[0, 0].plot(t, state[:, 0]); axes[0, 1].plot(t, state[:, 1])
    axes[0, 2].plot(t, -state[:, 2]); axes[0, 3].plot(t, np.hypot(state[:, 0], state[:, 1]))

    axes[1, 0].plot(t, state[:, 3]); axes[1, 1].plot(t, state[:, 4]); axes[1, 2].plot(t, state[:, 5])
    axes[1, 3].plot(t, np.sqrt(state[:, 3] ** 2 + state[:, 4] ** 2 + state[:, 5] ** 2))

    axes[2, 0].plot(t, np.rad2deg(state[:, 6])); axes[2, 1].plot(t, np.rad2deg(state[:, 7]))
    axes[2, 2].plot(t, np.rad2deg(state[:, 8])); axes[2, 3].axis("off")

    axes[3, 0].plot(t, state[:, 9]); axes[3, 1].plot(t, state[:, 10]); axes[3, 2].plot(t, state[:, 11])
    for i in range(4):
        axes[3, 3].plot(t, omega[:, i], label=f"$\\omega_{i+1}$", linewidth=1.2)
    axes[3, 3].legend(fontsize=7)

    labels = ["Vị trí x (m)", "Vị trí y (m)", "Độ cao (m)", "|Vị trí ngang| (m)",
              "Vận tốc u (m/s)", "Vận tốc v (m/s)", "Vận tốc w (m/s)", "|V| tổng (m/s)",
              "Roll φ (deg)", "Pitch θ (deg)", "Yaw ψ (deg)", "",
              "Rate p (rad/s)", "Rate q (rad/s)", "Rate r (rad/s)", "Omega 4 rotor (rad/s)"]
    for ax, lab in zip(axes.flat, labels):
        if lab:
            ax.set_title(lab, fontsize=9)
        ax.grid(alpha=0.3); ax.tick_params(labelsize=7)
    for ax in axes[3, :]:
        ax.set_xlabel("Thời gian (s)", fontsize=8)

    plt.suptitle(title or f"Toàn bộ trạng thái — {params.name}", fontsize=13, fontweight="bold", y=1.02)
    plt.tight_layout()
    return _save(fig, output_dir, filename, show)


def plot_3d_trajectory(state, params, output_dir="outputs", show=False,
                        filename="5_trajectory_3d.png", title=None, setpoint=None):
    fig = plt.figure(figsize=(8, 6.5))
    ax = fig.add_subplot(111, projection="3d")

    x, y, alt = state[:, 0], state[:, 1], -state[:, 2]
    ax.plot(x, y, alt, color="#1f77b4", linewidth=2, label=params.name)
    ax.scatter([x[0]], [y[0]], [alt[0]], color="#1f77b4", marker="o",
               s=70, facecolors="none", edgecolors="#1f77b4", linewidths=2, label="Bắt đầu")
    ax.scatter([x[-1]], [y[-1]], [alt[-1]], color="#d62728", marker="s", s=80, label="Kết thúc")
    if setpoint is not None:
        ax.scatter([setpoint[0]], [setpoint[1]], [-setpoint[2]], color="black",
                   marker="*", s=180, label="Setpoint")

    pad = 0.3
    xx, yy = np.meshgrid(np.linspace(x.min() - pad, x.max() + pad, 2),
                          np.linspace(y.min() - pad, y.max() + pad, 2))
    ax.plot_surface(xx, yy, np.zeros_like(xx), alpha=0.15, color="saddlebrown")

    ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)"); ax.set_zlabel("Độ cao (m)")
    ax.set_title(title or f"Quỹ đạo bay 3D — {params.name}", fontsize=12, fontweight="bold")
    ax.legend(fontsize=8, loc="upper left")
    plt.tight_layout()
    return _save(fig, output_dir, filename, show)


def plot_waypoint_radius_xy(state, world_wps, output_dir="outputs", show=False,
                             filename="6_waypoint_radius_xy.png", title=None, ax=None):
    """
    Do thi XY kieu PX4 flight-review "Local Position": duong bay thuc te,
    duong noi cac waypoint (net dut), va vong tron ban kinh chap nhan
    (epsilon/acceptance radius) quanh moi waypoint.

    world_wps: list cac dict co key "pos" ([x, y, z]) va "epsilon" (m).
    Neu ax duoc truyen vao thi ve len ax do (khong tu tao/luu figure) —
    dung khi nhung vao mot giao dien co san (vd Streamlit).
    """
    own_fig = ax is None
    if own_fig:
        fig, ax = plt.subplots(figsize=(7, 7))

    wp_x = [wp["pos"][0] for wp in world_wps]
    wp_y = [wp["pos"][1] for wp in world_wps]
    wp_eps = [wp.get("epsilon", 0.2) for wp in world_wps]

    # net dut noi cac waypoint theo thu tu (mission plan segments, giong PX4)
    ax.plot(wp_x, wp_y, linestyle="--", color="#444444", linewidth=1.1,
             zorder=2, label="Đường nối waypoint")

    # vong tron ban kinh chap nhan quanh moi waypoint
    for i, (x, y, eps) in enumerate(zip(wp_x, wp_y, wp_eps)):
        circ = plt.Circle((x, y), eps, fill=False, edgecolor="#ff9900",
                           linewidth=1.4, zorder=3,
                           label="Bán kính chấp nhận (epsilon)" if i == 0 else None)
        ax.add_patch(circ)

    # duong bay thuc te (setpoint da duoc lam muot boi controller)
    ax.plot(state[:, 0], state[:, 1], color="#2ca02c", linewidth=2.6,
             zorder=4, label="Quỹ đạo bay (Setpoint)")

    # marker tai vi tri cac waypoint
    ax.scatter(wp_x, wp_y, s=45, facecolors="#ff9900", edgecolors="black",
               linewidths=0.8, zorder=5, label="Waypoint")

    ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)")
    ax.set_title(title or "Quỹ đạo & bán kính waypoint (kiểu PX4)", fontsize=12, fontweight="bold")
    ax.grid(alpha=0.3)
    ax.set_aspect("equal", adjustable="datalim")
    ax.legend(fontsize=8, loc="best")

    if own_fig:
        plt.tight_layout()
        return _save(fig, output_dir, filename, show)
    return ax


def plot_all(t, state, omega, params, output_dir="outputs", show=False, prefix="",
             setpoint=None, title=None):
    """Goi ca 5 ham ve tren, luu vao 1 thu muc, tra ve danh sach duong dan file."""
    paths = []
    paths.append(plot_control_inputs(t, omega, params, output_dir, show, f"{prefix}1_omega.png"))
    paths.append(plot_positions(t, state, params, output_dir, show, f"{prefix}2_positions.png", setpoint))
    paths.append(plot_angles(t, state, params, output_dir, show, f"{prefix}3_angles.png"))
    paths.append(plot_full_state(t, state, omega, params, output_dir, show, f"{prefix}4_full_state.png", title))
    paths.append(plot_3d_trajectory(state, params, output_dir, show, f"{prefix}5_trajectory_3d.png", title, setpoint))
    return paths

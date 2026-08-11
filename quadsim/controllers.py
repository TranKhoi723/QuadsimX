"""
quadsim.controllers
======================
DIEU KHIEN VONG KIN (closed-loop) - Cascade PID 4 tang:

    Position -> Velocity -> Attitude -> Rate -> [tau_x, tau_y, tau_z]

Nguyen tac bandwidth separation: hang so thoi gian giam dan tu ngoai vao
trong (~3-5 lan moi buoc) de tang trong du nhanh de tang ngoai coi no la
"ly tuong". Gain tu tinh tu (params.mass, params.I) - KHONG dung so co dinh.

2 cong thuc quan trong suy TRUC TIEP tu dynamics.py (khong doan):
    F_cmd = mass*(g - az_cmd)                       (Newton truc Z World)
    theta_des = A*cos(psi) + B*sin(psi)              (cot 3 cua R = Rz*Ry*Rx)
    phi_des   = A*sin(psi) - B*cos(psi)              (A,B suy tu ax_cmd,ay_cmd)

Derivative-on-measurement: D-term lay dao ham theo GIA TRI DO, khong theo
sai so - tranh "derivative kick" khi setpoint nhay buoc.
"""

import numpy as np
from .dynamics import rotation_matrix_body_to_world


def wrap_to_pi(angle):
    """Dua goc ve khoang (-pi, pi] - can cho sai so yaw."""
    return (angle + np.pi) % (2 * np.pi) - np.pi


class PIDChannel:
    """1 kenh PID: anti-windup (integral_limit), derivative-on-measurement, output_limit."""

    def __init__(self, Kp, Ki=0.0, Kd=0.0, integral_limit=None, output_limit=None):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.integral_limit = integral_limit
        self.output_limit = output_limit
        self.integral = 0.0
        self.prev_measurement = None

    def reset(self):
        self.integral = 0.0
        self.prev_measurement = None

    def step(self, error, dt, measurement=None):
        self.integral += error * dt
        if self.integral_limit is not None:
            self.integral = np.clip(self.integral, -self.integral_limit, self.integral_limit)

        d_term = 0.0
        if self.Kd != 0.0 and measurement is not None and self.prev_measurement is not None:
            d_term = -self.Kd * (measurement - self.prev_measurement) / dt
        if measurement is not None:
            self.prev_measurement = measurement

        output = self.Kp * error + self.Ki * self.integral + d_term
        if self.output_limit is not None:
            output = np.clip(output, -self.output_limit, self.output_limit)
        return output


def default_gains(params, tau_pos=1.5, tau_vel=0.4, tau_att=0.12, tau_rate=0.03,
                   attitude_limit_deg=35.0):
    """Gain 4 tang TU (params.mass, params.I) + hang so thoi gian moi tang."""
    g = {}
    g["pos_x"] = PIDChannel(Kp=1 / tau_pos)
    g["pos_y"] = PIDChannel(Kp=1 / tau_pos)
    g["pos_z"] = PIDChannel(Kp=1 / tau_pos)

    g["vel_x"] = PIDChannel(Kp=1 / tau_vel, Ki=0.15 / tau_vel, Kd=0.05 / tau_vel, integral_limit=3.0)
    g["vel_y"] = PIDChannel(Kp=1 / tau_vel, Ki=0.15 / tau_vel, Kd=0.05 / tau_vel, integral_limit=3.0)
    g["vel_z"] = PIDChannel(Kp=1 / tau_vel, Ki=0.25 / tau_vel, Kd=0.05 / tau_vel, integral_limit=3.0)

    g["att_roll"] = PIDChannel(Kp=1 / tau_att)
    g["att_pitch"] = PIDChannel(Kp=1 / tau_att)
    g["att_yaw"] = PIDChannel(Kp=1 / tau_att)

    g["rate_roll"] = PIDChannel(Kp=params.Ixx / tau_rate, Ki=0.05 * params.Ixx / tau_rate,
                                 Kd=0.01 * params.Ixx / tau_rate, integral_limit=params.Ixx * 10)
    g["rate_pitch"] = PIDChannel(Kp=params.Iyy / tau_rate, Ki=0.05 * params.Iyy / tau_rate,
                                  Kd=0.01 * params.Iyy / tau_rate, integral_limit=params.Iyy * 10)
    g["rate_yaw"] = PIDChannel(Kp=params.Izz / tau_rate, Ki=0.05 * params.Izz / tau_rate,
                                Kd=0.01 * params.Izz / tau_rate, integral_limit=params.Izz * 10)

    g["attitude_limit_rad"] = np.deg2rad(attitude_limit_deg)
    return g


class CascadePIDController:
    """
    Dung:
        ctrl = CascadePIDController(params)
        wrench = ctrl.update(state, {'pos': [x,y,z], 'yaw': psi_sp}, dt)
    """

    def __init__(self, params, gains=None):
        self.p = params
        self.gains = gains if gains is not None else default_gains(params)
        self.att_limit = self.gains["attitude_limit_rad"]

    def reset(self):
        for ch in self.gains.values():
            if isinstance(ch, PIDChannel):
                ch.reset()

    def update(self, state, setpoint, dt):
        x, y, z = state[0], state[1], state[2]
        u, v, w = state[3], state[4], state[5]
        phi, theta, psi = state[6], state[7], state[8]
        p, q, r = state[9], state[10], state[11]

        R = rotation_matrix_body_to_world(phi, theta, psi)
        vx, vy, vz = R @ np.array([u, v, w])

        pos_sp = setpoint["pos"]
        psi_sp = setpoint.get("yaw", 0.0)

        vx_des = self.gains["pos_x"].step(pos_sp[0] - x, dt)
        vy_des = self.gains["pos_y"].step(pos_sp[1] - y, dt)
        vz_des = self.gains["pos_z"].step(pos_sp[2] - z, dt)

        ax_cmd = self.gains["vel_x"].step(vx_des - vx, dt, measurement=vx)
        ay_cmd = self.gains["vel_y"].step(vy_des - vy, dt, measurement=vy)
        az_cmd = self.gains["vel_z"].step(vz_des - vz, dt, measurement=vz)

        F_cmd = max(self.p.mass * (self.p.g - az_cmd), 1e-6)

        Acc = -self.p.mass * ax_cmd / F_cmd
        Bcc = -self.p.mass * ay_cmd / F_cmd
        theta_des = np.clip(Acc * np.cos(psi) + Bcc * np.sin(psi), -self.att_limit, self.att_limit)
        phi_des = np.clip(Acc * np.sin(psi) - Bcc * np.cos(psi), -self.att_limit, self.att_limit)

        p_des = self.gains["att_roll"].step(phi_des - phi, dt)
        q_des = self.gains["att_pitch"].step(theta_des - theta, dt)
        r_des = self.gains["att_yaw"].step(wrap_to_pi(psi_sp - psi), dt)

        tau_x = self.gains["rate_roll"].step(p_des - p, dt, measurement=p)
        tau_y = self.gains["rate_pitch"].step(q_des - q, dt, measurement=q)
        tau_z = self.gains["rate_yaw"].step(r_des - r, dt, measurement=r)

        return np.array([F_cmd, tau_x, tau_y, tau_z])


class WaypointManager:
    """
    Quan ly danh sach diem cho (Checkpoint-Switching, xem Hinh 14 - Luukkonen
    2011). Tu dong chuyen setpoint sang waypoint tiep theo khi drone bay vao
    ban kinh chap nhan epsilon quanh waypoint hien tai.

    Moi waypoint la 1 dict: {"pos": [x, y, z], "yaw": psi (rad, TUY CHON)}.
    Neu 1 waypoint khong co "yaw", dung yaw_default cua WaypointManager (mac
    dinh 0.0) - dam bao setpoint tra ra LUON co ca 'pos' va 'yaw' ro rang,
    khong phu thuoc vao gia tri mac dinh rieng cua CascadePIDController.

    KHONG con hard-code kich ban: waypoints truyen vao co the den tu bat ky
    dau - viet tay, doc file JSON (xem waypoint_io.py), hoac tu waypoint_editor.py
    (chon diem tren anh 2D).

    Dung trong vong lap thu cong:
        wm = WaypointManager([{"pos": [0, 0, -2]},
                               {"pos": [3, 0, -2], "yaw": 1.57}])
        setpoint, switched = wm.get_current_setpoint(state, t)
        wrench = ctrl.update(state, setpoint, dt)

    Hoac dung san simulate.simulate_waypoints() de chay ca vong lap.
    """

    def __init__(self, waypoints, epsilon=0.15, yaw_default=0.0):
        if not waypoints:
            raise ValueError("waypoints khong duoc rong - can it nhat 1 diem.")
        if epsilon <= 0:
            raise ValueError(f"epsilon ({epsilon}) phai > 0.")

        self.waypoints = []
        for i, wp in enumerate(waypoints):
            if "pos" not in wp:
                raise ValueError(f"waypoint[{i}] thieu key 'pos'.")
            pos = np.asarray(wp["pos"], dtype=float)
            if pos.shape != (3,):
                raise ValueError(
                    f"waypoint[{i}]['pos'] phai co dung 3 phan tu [x,y,z], "
                    f"nhan duoc shape {pos.shape}."
                )
            yaw = float(wp.get("yaw", yaw_default))
            # epsilon RIENG cho tung waypoint (tuy chon) - neu khong khai bao,
            # dung epsilon mac dinh cua ca WaypointManager (self.epsilon).
            wp_epsilon = float(wp["epsilon"]) if "epsilon" in wp and wp["epsilon"] is not None else None
            self.waypoints.append({"pos": pos, "yaw": yaw, "epsilon": wp_epsilon})

        self.epsilon = epsilon
        self.yaw_default = yaw_default
        self.current_idx = 0
        # Luu lai (idx_moi, t) moi lan chuyen waypoint - de log/ve moc tren do thi
        self.switch_log = []

    def reset(self):
        """Dua manager ve waypoint dau tien, xoa lich su chuyen doi."""
        self.current_idx = 0
        self.switch_log = []

    @property
    def n_waypoints(self):
        return len(self.waypoints)

    def get_current_setpoint(self, state, t=None):
        """
        Kiem tra vi tri hien tai so voi waypoint dang nham toi, tu dong
        chuyen sang waypoint ke tiep neu da vao ban kinh epsilon.

        Tra ve (setpoint, switched):
            setpoint : dict {'pos': ndarray(3,), 'yaw': float}, dua THANG vao
                       CascadePIDController.update(state, setpoint, dt).
            switched : True neu o LAN GOI NAY vua chuyen sang waypoint moi
                       (dung de danh dau moc / in log, khong bat buoc dung).
        """
        target = self.waypoints[self.current_idx]

        if self.is_completed():
            return target, False

        distance = np.linalg.norm(np.asarray(state[0:3], dtype=float) - target["pos"])
        eps_target = target["epsilon"] if target.get("epsilon") is not None else self.epsilon

        switched = False
        if distance < eps_target and self.current_idx < self.n_waypoints - 1:
            self.current_idx += 1
            self.switch_log.append((self.current_idx, t))
            target = self.waypoints[self.current_idx]
            switched = True

        return target, switched

    def is_completed(self):
        """True khi manager dang o (hoac da vuot qua) waypoint CUOI CUNG."""
        return self.current_idx >= self.n_waypoints - 1

    def distance_to_current(self, state):
        """Khoang cach Euclid tu vi tri hien tai den waypoint dang nham toi."""
        target = self.waypoints[self.current_idx]
        return float(np.linalg.norm(np.asarray(state[0:3], dtype=float) - target["pos"]))

    def epsilon_of_current(self):
        """Epsilon (m) ap dung cho waypoint dang nham toi (rieng neu co, khong thi mac dinh)."""
        target = self.waypoints[self.current_idx]
        return target["epsilon"] if target.get("epsilon") is not None else self.epsilon

    def progress(self):
        """Ty le da di qua trong chuoi waypoint (0..1), tinh theo CHI SO diem."""
        if self.n_waypoints <= 1:
            return 1.0
        return self.current_idx / (self.n_waypoints - 1)

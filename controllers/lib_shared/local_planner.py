from __future__ import annotations

import math
from typing import Dict, List, Tuple

DEFAULTS: Dict[str, float] = {
    "DT_CTRL": 0.08,
    "T_PRED": 2.0,
    "DT_SIM": 0.08,
    "NV": 7,
    "NW": 15,
    "V_MAX": 0.30,
    "W_MAX": 2.5,
    "A_V": 1.5,
    "A_W": 3.5,
    "RADIUS": 0.18,
    "SAFE": 0.06,
    "CLEAR_N": 0.5,
    "ALPHA": 0.18,
    "BETA": 0.34,
    "GAMMA": 0.18,
    "DELTA": 0.45,
    "EPS": 0.05,
    "D_NORM": 2.0,
}


def _clip(x: float, lo: float, hi: float) -> float:
    if x < lo:
        return lo
    if x > hi:
        return hi
    return x


def _wrap(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a <= -math.pi:
        a += 2.0 * math.pi
    return a


class DWA:
    def __init__(self, params: Dict[str, float] | None = None) -> None:
        cfg = dict(DEFAULTS)
        if params:
            cfg.update(params)
        self.p = cfg

    def rollout(self, v: float, w: float) -> List[Tuple[float, float, float]]:
        dt = self.p["DT_SIM"]
        t_max = self.p["T_PRED"]
        x = 0.0
        y = 0.0
        th = 0.0
        t = 0.0
        traj: List[Tuple[float, float, float]] = []
        while t < t_max:
            x += v * math.cos(th) * dt
            y += v * math.sin(th) * dt
            th = _wrap(th + w * dt)
            traj.append((x, y, th))
            t += dt
        return traj

    @staticmethod
    def _min_clear(
        traj_xy: List[Tuple[float, float]],
        obs_xy: List[Tuple[float, float]],
    ) -> float:
        if not obs_xy:
            return float("inf")
        best = float("inf")
        for tx, ty in traj_xy:
            for ox, oy in obs_xy:
                d = math.hypot(ox - tx, oy - ty)
                if d < best:
                    best = d
        return best

    def get_safe_velocities(
        self,
        lidar_points: List[Tuple[float, float]],
        goal_vec: Tuple[float, float],
        prev_cmd: Tuple[float, float] = (0.0, 0.0),
        cur: Tuple[float, float] = (0.0, 0.0),
    ) -> Tuple[float, float]:
        p = self.p
        v_now, w_now = cur

        v_lo = _clip(v_now - p["A_V"] * p["DT_CTRL"], -p["V_MAX"], p["V_MAX"])
        v_hi = _clip(v_now + p["A_V"] * p["DT_CTRL"], -p["V_MAX"], p["V_MAX"])
        w_lo = _clip(w_now - p["A_W"] * p["DT_CTRL"], -p["W_MAX"], p["W_MAX"])
        w_hi = _clip(w_now + p["A_W"] * p["DT_CTRL"], -p["W_MAX"], p["W_MAX"])

        samples: List[Tuple[float, float]] = [prev_cmd]
        if p["W_MAX"] > 0.0:
            samples.append((0.0, 0.6 * p["W_MAX"]))
            samples.append((0.0, -0.6 * p["W_MAX"]))

        nv = max(1, int(p["NV"]))
        nw = max(1, int(p["NW"]))
        for i in range(nv):
            if nv == 1:
                v = 0.5 * (v_lo + v_hi)
            else:
                v = v_lo + (v_hi - v_lo) * i / (nv - 1)
            for j in range(nw):
                if nw == 1:
                    w = 0.5 * (w_lo + w_hi)
                else:
                    w = w_lo + (w_hi - w_lo) * j / (nw - 1)
                samples.append((v, w))

        gx, gy = goal_vec
        dist_goal = math.hypot(gx, gy)
        if dist_goal > 1e-9:
            g_th = math.atan2(gy, gx)
        else:
            g_th = 0.0

        best_v = 0.0
        best_w = 0.0
        best_score = -1e9

        for v, w in samples:
            traj = self.rollout(v, w)
            pts = [(x, y) for x, y, _ in traj]
            d_raw = self._min_clear(pts, lidar_points)
            eff_clear = max(0.0, d_raw - (p["RADIUS"] + 0.02))
            if eff_clear < p["SAFE"]:
                continue

            x_e, y_e, th_e = traj[-1]
            heading = 1.0 - abs(_wrap(th_e - g_th)) / math.pi
            prog = 1.0 - min(math.hypot(gx - x_e, gy - y_e) / p["D_NORM"], 1.0)
            speed_term = max(0.0, v) / (p["V_MAX"] if p["V_MAX"] > 0 else 1.0)
            clear_term = min(eff_clear / p["CLEAR_N"], 1.0)
            smooth = math.exp(
                -math.hypot(v - prev_cmd[0], w - prev_cmd[1]) / 0.2
            )

            score = (
                p["ALPHA"] * heading
                + p["BETA"] * prog
                + p["GAMMA"] * clear_term
                + p["DELTA"] * speed_term
                + p["EPS"] * smooth
            )

            if score > best_score:
                best_score = score
                best_v = v
                best_w = w

        if best_score < -1e8:
            if lidar_points:
                bias = 0.0
                for ox, oy in lidar_points:
                    if ox > 0.05:
                        bias += oy
                if bias > 0.0:
                    return 0.0, -0.9
                if bias < 0.0:
                    return 0.0, 0.9
            if abs(g_th) > math.pi / 6.0:
                return 0.0, 0.8 * math.copysign(1.0, g_th)
            return 0.14, 0.0

        if best_v > 0.0 and dist_goal > 0.30:
            if best_v < 0.16:
                best_v = 0.16

        return best_v, best_w

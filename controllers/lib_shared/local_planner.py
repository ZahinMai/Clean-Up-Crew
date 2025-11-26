from __future__ import annotations

import math
from typing import List, Tuple, Dict
from collections import deque # New Import

DEFAULTS: Dict[str, float] = {
    "DT_CTRL": 0.08,
    "T_PRED": 2.0,
    "DT_SIM": 0.08,
    "NV": 8,
    "NW": 18,
    "V_MAX": 0.22,
    "W_MAX": 2.5,
    "A_V": 0.8,
    "A_W": 3.0,
    "RADIUS": 0.18,
    "SAFE": 0.08,
    "CLEAR_N": 0.7,
    "ALPHA": 0.35,
    "BETA": 0.40,
    "GAMMA": 0.45,
    "DELTA": 0.05,
    "EPS": 0.12,
    "D_NORM": 2.0,
}
# Defaults for Stuck Detection
STUCK_DEFAULTS: Dict[str, float] = {
    "WINDOW_LEN": 25,
    "AVG_THRESHOLD": 0.02,
    "COOLDOWN": 1.5,
    "RECOVERY_V": -0.10,
    "RECOVERY_W": 0.9,
}

def _clamp(x: float, a: float, b: float) -> float:
    if x < a:
        return a
    if x > b:
        return b
    return x


def _ang(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a <= -math.pi:
        a += 2.0 * math.pi
    return a


class DWA:
    def __init__(self, params: Dict | None = None) -> None:
        p = dict(DEFAULTS)
        if params:
            p.update(params)
        self.p = p
        
        # Stuck detection state (New)
        self.speed_window = deque(maxlen=int(STUCK_DEFAULTS["WINDOW_LEN"]))
        self.last_unstuck_time = -10.0
        self.unstuck_cooldown = STUCK_DEFAULTS["COOLDOWN"]
        self.avg_threshold = STUCK_DEFAULTS["AVG_THRESHOLD"]
        self.recovery_v = STUCK_DEFAULTS["RECOVERY_V"]
        self.recovery_w = STUCK_DEFAULTS["RECOVERY_W"]

    # ... (rollout and _min_clear methods remain the same) ...
    def rollout(self, v: float, w: float) -> List[Tuple[float, float, float]]:
        # ... (Implementation as before) ...
        x = 0.0
        y = 0.0
        th = 0.0
        t = 0.0
        traj: List[Tuple[float, float, float]] = []
        dt = self.p["DT_SIM"]
        horizon = self.p["T_PRED"]
        while t < horizon:
            x += v * math.cos(th) * dt
            y += v * math.sin(th) * dt
            th = _ang(th + w * dt)
            traj.append((x, y, th))
            t += dt
        return traj

    @staticmethod
    def _min_clear(
        traj_xy: List[Tuple[float, float]],
        obs_xy: List[Tuple[float, float]],
    ) -> float:
        # ... (Implementation as before) ...
        if not obs_xy:
            return float("inf")
        dmin = float("inf")
        for tx, ty in traj_xy:
            for ox, oy in obs_xy:
                d = math.hypot(ox - tx, oy - ty)
                if d < dmin:
                    dmin = d
        return dmin

    def get_safe_velocities(
        self,
        lidar_points: List[Tuple[float, float]],
        goal_vec: Tuple[float, float],
        prev_cmd: Tuple[float, float] = (0.0, 0.0),
        cur: Tuple[float, float] = (0.0, 0.0),
    ) -> Tuple[float, float]:
        # ... (Implementation as before - core DWA score calculation) ...
        p = self.p
        vcur, wcur = cur

        vmin = _clamp(vcur - p["A_V"] * p["DT_CTRL"], -p["V_MAX"], p["V_MAX"])
        vmax = _clamp(vcur + p["A_V"] * p["DT_CTRL"], -p["V_MAX"], p["V_MAX"])
        wmin = _clamp(wcur - p["A_W"] * p["DT_CTRL"], -p["W_MAX"], p["W_MAX"])
        wmax = _clamp(wcur + p["A_W"] * p["DT_CTRL"], -p["W_MAX"], p["W_MAX"])

        samples: List[Tuple[float, float]] = [prev_cmd, (0.0, 0.0)]
        if p["W_MAX"] > 0.0:
            samples.append((0.0, 0.6 * p["W_MAX"]))
            samples.append((0.0, -0.6 * p["W_MAX"]))

        nv = max(1, int(p["NV"]))
        nw = max(1, int(p["NW"]))

        for i in range(nv):
            if nv == 1:
                vv = 0.5 * (vmin + vmax)
            else:
                vv = vmin + (vmax - vmin) * (i / (nv - 1))
            for j in range(nw):
                if nw == 1:
                    ww = 0.5 * (wmin + wmax)
                else:
                    ww = wmin + (wmax - wmin) * (j / (nw - 1))
                samples.append((vv, ww))

        gx, gy = goal_vec
        if gx * gx + gy * gy > 1e-9:
            gtheta = math.atan2(gy, gx)
        else:
            gtheta = 0.0

        best = (0.0, 0.0)
        best_score = -1e9

        for v, w in samples:
            traj = self.rollout(v, w)
            pts = [(x, y) for (x, y, _) in traj]
            dmin_raw = self._min_clear(pts, lidar_points)
            eff_clear = max(0.0, dmin_raw - (p["RADIUS"] + 0.02))
            if eff_clear < p["SAFE"]:
                continue

            x_end, y_end, th_end = traj[-1]

            h = 1.0 - abs(_ang(th_end - gtheta)) / math.pi
            prog = 1.0 - min(math.hypot(gx - x_end, gy - y_end) / p["D_NORM"], 1.0)
            speed_score = max(0.0, v) / (p["V_MAX"] if p["V_MAX"] > 0 else 1.0)
            clear_score = min(eff_clear / p["CLEAR_N"], 1.0)
            smooth = math.exp(
                -math.hypot(v - prev_cmd[0], w - prev_cmd[1]) / 0.25
            )

            score = (
                p["ALPHA"] * h
                + p["BETA"] * prog
                + p["GAMMA"] * clear_score
                + p["DELTA"] * speed_score
                + p["EPS"] * smooth
            )

            if score > best_score:
                best_score = score
                best = (v, w)

        if best_score < -1e8:
            if lidar_points:
                side = 0.0
                for x_obs, y_obs in lidar_points:
                    if x_obs > 0.05:
                        side += y_obs
                if side > 0.0:
                    return 0.0, -0.5
                if side < 0.0:
                    return 0.0, 0.5
            if abs(gtheta) > math.pi / 6.0:
                return 0.0, 0.4 * math.copysign(1.0, gtheta)
            return 0.05, 0.0

        return best

    def get_stuck_safe_velocities(
        self,
        lidar_points: List[Tuple[float, float]],
        goal_vec: Tuple[float, float],
        cur_time: float,
        prev_cmd: Tuple[float, float] = (0.0, 0.0),
        cur: Tuple[float, float] = (0.0, 0.0),
    ) -> Tuple[float, float]:
        """
        Calculates safe velocities and applies stuck detection/recovery logic.
        """
        # 1. Get DWA-calculated command
        v_cmd, w_cmd = self.get_safe_velocities(lidar_points, goal_vec, prev_cmd, cur)

        # 2. Stuck Detection Check
        self.speed_window.append(abs(v_cmd))
        stuck = (
            len(self.speed_window) == self.speed_window.maxlen
            and sum(self.speed_window) / len(self.speed_window) < self.avg_threshold
        )

        v, w = v_cmd, w_cmd

        # 3. Stuck Recovery Logic
        if stuck and (cur_time - self.last_unstuck_time) > self.unstuck_cooldown:
            # Perform a hard turn to unstick
            print("[DWA: RECOVERY] STUCK! Initiating turn recovery.")
            # Reverse slightly and turn hard, magnitude of turn based on last angular velocity
            turn_sign = math.copysign(1.0, -w if w != 0 else 1.0)
            v, w = self.recovery_v, self.recovery_w * turn_sign
            
            self.last_unstuck_time = cur_time
            self.speed_window.clear() # Reset window after recovery

        # 4. Return potentially modified command
        return v, w
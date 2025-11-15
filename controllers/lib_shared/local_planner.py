# controllers/lib_shared/local_planner.py
# Dynamic Window Approach (DWA) local planner for Webots
# Units: meters, seconds, radians.

from __future__ import annotations
from typing import List, Tuple, Dict
import math

DEFAULTS: Dict[str, float] = dict(
    # Timing / sampling
    DT_CTRL = 0.08,      # controller period [s]
    T_PRED  = 2.0,       # rollout horizon [s]
    DT_SIM  = 0.08,      # rollout step [s]
    NV = 10,             # linear velocity samples
    NW = 16,             # angular velocity samples

    # Limits (tune from the controller if needed)
    V_MAX = 0.25,        # m/s  (kept conservative for tight aisles)
    W_MAX = 2.0,         # rad/s
    A_V   = 1.0,         # m/s^2
    A_W   = 2.5,         # rad/s^2

    # Geometry & safety
    RADIUS = 0.18,       # robot body radius [m] (TB3 footprint ~0.17–0.18)
    SAFE   = 0.10,       # REQUIRED effective clearance after inflation [m]
    CLEAR_N = 1.00,      # clearance normalization (score saturates at 1 m)

    # Weights (objective = α*heading + β*progress + γ*clearance + δ*speed + ε*smooth)
    ALPHA = 0.30,        # heading alignment
    BETA  = 0.35,        # progress to goal
    GAMMA = 0.50,        # clearance
    DELTA = 0.10,        # forward speed
    EPS   = 0.10,        # smoothness vs previous command

    # Normalization
    D_NORM = 2.0         # meters to normalize progress term
)

def _clamp(x, a, b): return a if x < a else b if x > b else x

def _ang(a: float) -> float:
    """Wrap angle to (-pi, pi]."""
    while a >  math.pi: a -= 2*math.pi
    while a <= -math.pi: a += 2*math.pi
    return a

class DWA:
    """
    Minimal, production-ready DWA.

    Public API:
        get_safe_velocities(lidar_points, goal_vec, prev_cmd=(0,0), cur=(0,0)) -> (v,w)

    Args:
      lidar_points: list of (x,y) obstacle points in ROBOT frame [m]
      goal_vec: (gx, gy) vector to goal in ROBOT frame [m]
      prev_cmd: previous commanded (v,w) used for smoothness
      cur: estimated current (v,w) used for dynamic window
    Returns:
      (v, w): linear and angular velocities [m/s, rad/s]
    """
    def __init__(self, params: Dict | None = None):
        p = dict(DEFAULTS)
        if params: p.update(params)
        self.p = p

    # ---------------- core utilities ----------------
    def rollout(self, v: float, w: float) -> List[Tuple[float, float, float]]:
        """Integrate unicycle model from (0,0,0) for T_PRED."""
        x = y = th = t = 0.0
        traj: List[Tuple[float, float, float]] = []
        while t < self.p['T_PRED']:
            x += v * math.cos(th) * self.p['DT_SIM']
            y += v * math.sin(th) * self.p['DT_SIM']
            th = _ang(th + w * self.p['DT_SIM'])
            traj.append((x, y, th))
            t += self.p['DT_SIM']
        return traj

    @staticmethod
    def _min_clear(traj_xy: List[Tuple[float, float]], obs_xy: List[Tuple[float, float]]) -> float:
        """Minimum point-to-point distance between trajectory and obstacles (raw, not inflated)."""
        if not obs_xy:
            return float('inf')
        dmin = float('inf')
        for tx, ty in traj_xy:
            for ox, oy in obs_xy:
                d = math.hypot(ox - tx, oy - ty)
                if d < dmin:
                    dmin = d
        return dmin

    # ---------------- main API ----------------
    def get_safe_velocities(
        self,
        lidar_points: List[Tuple[float, float]],
        goal_vec: Tuple[float, float],
        prev_cmd: Tuple[float, float] = (0.0, 0.0),
        cur: Tuple[float, float] = (0.0, 0.0)
    ) -> Tuple[float, float]:
        p = self.p
        vcur, wcur = cur

        # Dynamic window bounds from accel limits
        Vmin = _clamp(vcur - p['A_V'] * p['DT_CTRL'], -p['V_MAX'], p['V_MAX'])
        Vmax = _clamp(vcur + p['A_V'] * p['DT_CTRL'], -p['V_MAX'], p['V_MAX'])
        Wmin = _clamp(wcur - p['A_W'] * p['DT_CTRL'], -p['W_MAX'], p['W_MAX'])
        Wmax = _clamp(wcur + p['A_W'] * p['DT_CTRL'], -p['W_MAX'], p['W_MAX'])

        # Seed samples with a few bias points for stability
        samples: List[Tuple[float, float]] = [prev_cmd, (0.0, 0.0)]
        if p['W_MAX'] > 0.0:
            samples += [(0.0, 0.7 * p['W_MAX']), (0.0, -0.7 * p['W_MAX'])]

        # Grid sampling
        for i in range(max(1, p['NV'])):
            vv = Vmin + (Vmax - Vmin) * (i / (p['NV'] - 1 if p['NV'] > 1 else 1))
            for j in range(max(1, p['NW'])):
                ww = Wmin + (Wmax - Wmin) * (j / (p['NW'] - 1 if p['NW'] > 1 else 1))
                samples.append((vv, ww))

        gx, gy = goal_vec
        gtheta = math.atan2(gy, gx) if (gx * gx + gy * gy) > 1e-9 else 0.0

        best = (0.0, 0.0)
        bestJ = -1e9

        for v, w in samples:
            traj = self.rollout(v, w)
            pts = [(x, y) for (x, y, _) in traj]

            # --- clearance with robot inflation ---
            dmin_raw = self._min_clear(pts, lidar_points)
            # effective clearance AFTER accounting for robot radius + 2 cm buffer
            eff_clear = max(0.0, dmin_raw - (p['RADIUS'] + 0.02))

            # Hard safety gate
            if eff_clear < p['SAFE']:
                continue

            xE, yE, thE = traj[-1]

            # Scores
            H  = 1.0 - abs(_ang(thE - gtheta)) / math.pi                        # heading
            Pp = 1.0 - min(math.hypot(gx - xE, gy - yE) / p['D_NORM'], 1.0)     # progress
            Vt = max(0.0, v) / (p['V_MAX'] if p['V_MAX'] > 0 else 1.0)          # forward speed
            C  = min(eff_clear / p['CLEAR_N'], 1.0)                              # inflated clearance
            S  = math.exp(- math.hypot(v - prev_cmd[0], w - prev_cmd[1]) / 0.2)  # smoothness

            J = p['ALPHA'] * H + p['BETA'] * Pp + p['GAMMA'] * C + p['DELTA'] * Vt + p['EPS'] * S
            if J > bestJ:
                bestJ, best = J, (v, w)

        # Fallback if everything rejected or goal hard to face
        if bestJ < -1e8:
            if abs(gtheta) > math.pi / 6:
                return (0.0, 0.3 * math.copysign(1.0, gtheta))  # gentle in-place turn
            return (0.05, 0.0)  # creep forward

        return best

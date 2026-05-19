#!/usr/bin/env python3
"""
PHOENIX IMPPF — AP localisation from real FTM RTT + SLAM data
==============================================================

IMPPF implementation ported directly from Simulator3.py (the validated,
feature-complete version) and wired to the CSV files produced by
phoenix_logger (03_phoenix_logger.py).

CSV inputs (from phoenix_logger):
    rtt.csv        — t_sec, seq, n_frames, d_median_m, d_mad_m,
                     rssi_median_dbm, pose_x, pose_y, pose_yaw,
                     pose_ok, label
    trajectory.csv — t_sec, x, y, yaw
    map.npy        — int8 occupancy grid  (>50 occupied, 0-50 free, -1 unknown)
    map_meta.json  — resolution, origin_x, origin_y, width, height, …

Usage
-----
    python 04_imppf_prototype.py \\
        --map      run01/map.npy \\
        --map-meta run01/map_meta.json \\
        --traj     run01/trajectory.csv \\
        --rtt      run01/rtt.csv \\
        --sigma-los 0.5  --sigma-nlos 1.5  --bias 0.8 \\
        --offset-b 5.5 \\
        --save     run01/result

    # Outlier-robust Student-t likelihood (recommended for real data):
        --likelihood student_t --student-t-dof 4

Key parameters
--------------
    --offset-b    FTM systematic offset to subtract from every d_median_m.
                  ESP32-S3 pairs typically need 1.5–3 m, but our firmware at
                  32 frames/burst often shows ~5–6 m.  Calibrate per
                  01_firmware_and_calibration.md §2.1 for best results.
    --sigma-los   LOS noise σ after removing offset (m).  ~0.4–0.6 m typical.
    --sigma-nlos  NLOS noise σ (m).  ~1.0–2.0 m typical.
    --bias        Thin-wall additive bias (m).  ~0.5–1.5 m typical.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

try:
    from filterpy.monte_carlo import systematic_resample as _FILTERPY_RESAMPLE
except ImportError:
    _FILTERPY_RESAMPLE = None


# ============================================================================
# Occupancy grid  (adapted from Simulator3 to support SLAM-map origins)
# ============================================================================

@dataclass
class OccupancyGrid:
    """
    2-D binary occupancy grid.  cells[row, col] == 1 means occupied.

    For synthetic worlds  → use from_walls().
    For SLAM maps          → use from_slam_map().
    """
    width:      float      = 30.0
    height:     float      = 30.0
    resolution: float      = 0.1
    origin_x:   float      = 0.0
    origin_y:   float      = 0.0
    cells: np.ndarray = field(
        default_factory=lambda: np.zeros((1, 1), dtype=np.uint8))

    # ── constructors ──────────────────────────────────────────────────────────

    @classmethod
    def from_slam_map(cls, map_array: np.ndarray,
                      meta: dict) -> "OccupancyGrid":
        """
        Load from a SLAM OccupancyGrid saved by phoenix_logger.

        map_array  int8 array  (>50 = occupied, 0-50 = free, -1 = unknown)
        meta       dict with keys: resolution, origin_x, origin_y
        """
        res      = float(meta["resolution"])
        origin_x = float(meta.get("origin_x", 0.0))
        origin_y = float(meta.get("origin_y", 0.0))
        H, W     = map_array.shape
        # Binary: occupied walls only (unknown cells treated as free)
        cells = (map_array > 50).astype(np.uint8)
        return cls(
            width      = W * res,
            height     = H * res,
            resolution = res,
            origin_x   = origin_x,
            origin_y   = origin_y,
            cells      = cells,
        )

    @classmethod
    def from_walls(cls, walls,
                   width=30.0, height=30.0, resolution=0.1,
                   wall_thickness: int = 1) -> "OccupancyGrid":
        cols  = int(round(width  / resolution))
        rows  = int(round(height / resolution))
        cells = np.zeros((rows, cols), dtype=np.uint8)
        for (x1, y1, x2, y2) in walls:
            _rasterize_segment(cells, x1, y1, x2, y2,
                               resolution, wall_thickness)
        return cls(width=width, height=height, resolution=resolution,
                   cells=cells)

    # ── coordinate helpers ────────────────────────────────────────────────────

    def world_to_grid(self, xy: np.ndarray) -> np.ndarray:
        """
        Convert (..., 2) world coords [x, y] to fractional grid coords
        [col, row].  Accounts for map origin.
        """
        xy = np.asarray(xy, dtype=np.float64)
        orig = np.array([self.origin_x, self.origin_y], dtype=np.float64)
        return (xy - orig) / self.resolution


def _rasterize_segment(cells, x1, y1, x2, y2, res, thickness):
    rows, cols = cells.shape
    c1 = int(round(x1 / res)); r1 = int(round(y1 / res))
    c2 = int(round(x2 / res)); r2 = int(round(y2 / res))
    dc = abs(c2 - c1); dr = abs(r2 - r1)
    sc = 1 if c1 < c2 else -1
    sr = 1 if r1 < r2 else -1
    err = dc - dr
    c, r = c1, r1
    while True:
        for dr_off in range(-thickness + 1, thickness):
            for dc_off in range(-thickness + 1, thickness):
                rr, cc = r + dr_off, c + dc_off
                if 0 <= rr < rows and 0 <= cc < cols:
                    cells[rr, cc] = 1
        if c == c2 and r == r2:
            break
        e2 = 2 * err
        if e2 > -dr:
            err -= dr; c += sc
        if e2 < dc:
            err += dc; r += sr


# ============================================================================
# Vectorized DDA raycaster  (from Simulator3, unchanged)
# ============================================================================

def vectorized_los_check(robot_grid_xy: np.ndarray,
                         particles_grid_xy: np.ndarray,
                         cells: np.ndarray) -> np.ndarray:
    """
    Batch DDA raycast. Returns bool array length N, True == LOS.
    All inputs in grid coordinates [col, row], fractional allowed.
    """
    rows, cols = cells.shape
    rx, ry = float(robot_grid_xy[0]), float(robot_grid_xy[1])
    px = particles_grid_xy[:, 0].astype(np.float64)
    py = particles_grid_xy[:, 1].astype(np.float64)
    dx = px - rx
    dy = py - ry

    n_steps_per = np.maximum(np.abs(dx), np.abs(dy)).astype(np.int32) + 1
    max_steps   = int(n_steps_per.max()) if n_steps_per.size else 1
    if max_steps < 2:
        return np.ones(particles_grid_xy.shape[0], dtype=bool)

    t  = np.linspace(0.0, 1.0, max_steps)
    xs = rx + dx[:, None] * t[None, :]
    ys = ry + dy[:, None] * t[None, :]

    ci = np.clip(np.round(xs).astype(np.int32), 0, cols - 1)
    ri = np.clip(np.round(ys).astype(np.int32), 0, rows - 1)

    step_idx = np.arange(max_steps)[None, :]
    nsm1     = (n_steps_per - 1)[:, None]
    interior = (step_idx > 0) & (step_idx < nsm1)

    blocked = (cells[ri, ci] >= 1) & interior
    return ~blocked.any(axis=1)


# ============================================================================
# IMPPF  (from Simulator3 — includes Student-t likelihood + collapse recovery)
# ============================================================================

@dataclass
class IMPPFConfig:
    N:               int   = 1000
    sigma_los:       float = 0.5
    sigma_nlos:      float = 1.5
    bias_thin_wall:  float = 0.8
    sigma_init:      float = 0.5
    rough_jitter_m:  float = 0.10
    seed:   Optional[int]  = 42
    # "gaussian" or "student_t".  Student-t is more robust against RTT outliers.
    likelihood:      str   = "gaussian"
    student_t_dof:   float = 4.0


class IMPPF:
    """Interacting Multiple Particle Filter for a stationary 2-D AP."""

    def __init__(self, occupancy: OccupancyGrid, cfg: IMPPFConfig):
        self.occ  = occupancy
        self.cfg  = cfg
        self.rng  = np.random.default_rng(cfg.seed)

        self.particles:  Optional[np.ndarray] = None   # (N, 2) world coords
        self.weights:    Optional[np.ndarray] = None   # (N,)
        self.initialized = False

        self.last_estimate:  Optional[Tuple[float, float]] = None
        self.last_los_frac:  float = 1.0
        self.last_neff:      float = float(cfg.N)
        self.last_resampled: bool  = False

    def initialize(self, robot_xy: np.ndarray, d_first: float) -> None:
        N     = self.cfg.N
        theta = self.rng.uniform(0.0, 2.0 * math.pi, size=N)
        r     = d_first + self.rng.normal(0.0, self.cfg.sigma_init, size=N)
        r     = np.maximum(r, 0.05)
        x     = robot_xy[0] + r * np.cos(theta)
        y     = robot_xy[1] + r * np.sin(theta)
        self.particles   = np.stack([x, y], axis=1)
        self.weights     = np.full(N, 1.0 / N)
        self.initialized = True

    def step(self, robot_xy, d_rtt: float) -> None:
        robot_xy = np.asarray(robot_xy, dtype=np.float64)

        if not self.initialized:
            self.initialize(robot_xy, float(d_rtt))
            self.last_estimate = self.estimate()
            return

        robot_g = self.occ.world_to_grid(robot_xy)
        part_g  = self.occ.world_to_grid(self.particles)
        is_los  = vectorized_los_check(robot_g, part_g, self.occ.cells)
        self.last_los_frac = float(is_los.mean())

        d_hyp  = np.linalg.norm(self.particles - robot_xy, axis=1)
        d_pred = np.where(is_los, d_hyp, d_hyp + self.cfg.bias_thin_wall)
        sigma  = np.where(is_los, self.cfg.sigma_los, self.cfg.sigma_nlos)

        residual = (d_rtt - d_pred) / sigma
        if self.cfg.likelihood == "student_t":
            dof     = max(self.cfg.student_t_dof, 1.0)
            log_lik = (-0.5 * (dof + 1.0) * np.log1p(residual ** 2 / dof)
                       - np.log(sigma))
        else:
            log_lik = -0.5 * residual ** 2 - np.log(sigma)

        log_lik -= log_lik.max()
        lik = np.exp(log_lik)

        self.weights = self.weights * lik
        s = self.weights.sum()
        if not np.isfinite(s) or s <= 0.0:
            self.weights = np.full(self.cfg.N, 1.0 / self.cfg.N)
        else:
            self.weights /= s

        neff = 1.0 / np.sum(self.weights ** 2)
        self.last_neff      = float(neff)
        self.last_resampled = False
        if neff < self.cfg.N / 2.0:
            self._systematic_resample()
            if self.cfg.rough_jitter_m > 0.0:
                self.particles += self.rng.normal(
                    0.0, self.cfg.rough_jitter_m, self.particles.shape)
            self.last_resampled = True

        self.last_estimate = self.estimate()

    def _systematic_resample(self) -> None:
        N = self.cfg.N
        if _FILTERPY_RESAMPLE is not None:
            idx = _FILTERPY_RESAMPLE(self.weights)
        else:
            positions = (np.arange(N) + self.rng.uniform()) / N
            cumsum    = np.cumsum(self.weights)
            cumsum[-1] = 1.0
            idx = np.searchsorted(cumsum, positions)
            idx = np.clip(idx, 0, N - 1)
        self.particles = self.particles[idx]
        self.weights   = np.full(N, 1.0 / N)

    def estimate(self) -> Tuple[float, float]:
        if self.particles is None:
            return (float("nan"), float("nan"))
        x = float(np.average(self.particles[:, 0], weights=self.weights))
        y = float(np.average(self.particles[:, 1], weights=self.weights))
        return (x, y)


# ============================================================================
# Data loading  (real CSV format from phoenix_logger)
# ============================================================================

def load_rtt(path: str, offset_b: float, skip_bad_pose: bool = True):
    """
    Load rtt.csv produced by phoenix_logger.

    Columns: t_sec, seq, n_frames, d_median_m, d_mad_m, rssi_median_dbm,
             pose_x, pose_y, pose_yaw, pose_ok, label

    Returns list of dicts with keys: t, x, y, d_meas, label
    """
    rows = []
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            if skip_bad_pose and r.get("pose_ok", "1") == "0":
                continue
            try:
                d_raw = float(r["d_median_m"])
                d     = d_raw - offset_b
                if d <= 0.0:
                    continue        # negative corrected distance → skip
                rows.append({
                    "t":     float(r["t_sec"]),
                    "x":     float(r["pose_x"]),
                    "y":     float(r["pose_y"]),
                    "d_meas": d,
                    "d_raw":  d_raw,
                    "label":  r.get("label", "UNKNOWN"),
                })
            except (ValueError, KeyError):
                continue
    return rows


def load_trajectory(path: str):
    """
    Load trajectory.csv (t_sec, x, y, yaw).
    Returns Nx3 array [t, x, y].
    """
    rows = []
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            try:
                rows.append([float(r["t_sec"]),
                             float(r["x"]),
                             float(r["y"])])
            except (ValueError, KeyError):
                continue
    return np.array(rows) if rows else np.empty((0, 3))


# ============================================================================
# Plotting
# ============================================================================

def _map_extent(meta: dict):
    ox = meta.get("origin_x", 0.0)
    oy = meta.get("origin_y", 0.0)
    W  = int(meta.get("width",  1))
    H  = int(meta.get("height", 1))
    res = meta.get("resolution", 0.05)
    return [ox, ox + W * res, oy, oy + H * res]   # [xmin, xmax, ymin, ymax]


def plot_results(occ: OccupancyGrid, meta: dict,
                 traj: np.ndarray,
                 measurements: List[dict],
                 estimates: List[Tuple[float, float]],
                 particles_final: Optional[np.ndarray],
                 cfg: IMPPFConfig,
                 save_prefix: Optional[str]):

    ext = _map_extent(meta)

    # ── Figure 1: main result ─────────────────────────────────────────────────
    fig1, ax = plt.subplots(figsize=(9, 7))
    ax.set_facecolor("#1a1a2e")
    fig1.patch.set_facecolor("#0f0f23")

    # Map
    map_disp = occ.cells.astype(float)
    ax.imshow(map_disp, origin="lower", extent=ext,
              cmap="gray", vmin=0, vmax=1, alpha=0.6, zorder=1)

    # Trajectory
    if traj.shape[0] > 1:
        ax.plot(traj[:, 1], traj[:, 2], color="#4a9eff", linewidth=1.0,
                alpha=0.6, label="Trajectory", zorder=2)

    # Measurement poses coloured by label
    los_x  = [m["x"] for m in measurements if "LOS"  in m["label"]]
    los_y  = [m["y"] for m in measurements if "LOS"  in m["label"]]
    nlos_x = [m["x"] for m in measurements if "NLOS" in m["label"]]
    nlos_y = [m["y"] for m in measurements if "NLOS" in m["label"]]
    if los_x:
        ax.scatter(los_x, los_y, s=20, c="#2ecc71", alpha=0.8,
                   edgecolors="none", label=f"LOS bursts ({len(los_x)})", zorder=3)
    if nlos_x:
        ax.scatter(nlos_x, nlos_y, s=20, c="#e74c3c", alpha=0.8,
                   edgecolors="none", label=f"NLOS bursts ({len(nlos_x)})", zorder=3)

    # Final particle cloud
    if particles_final is not None:
        ax.scatter(particles_final[:, 0], particles_final[:, 1],
                   s=3, c="#f39c12", alpha=0.25, zorder=4, label="Particles (final)")

    # Estimate trace
    if estimates:
        ex = [p[0] for p in estimates]
        ey = [p[1] for p in estimates]
        ax.plot(ex, ey, color="#f39c12", linewidth=1.5, alpha=0.9,
                label="Estimate trace", zorder=5)
        ax.plot(ex[-1], ey[-1], "X", color="#f39c12",
                markersize=16, markeredgecolor="white",
                markeredgewidth=1.2, zorder=6,
                label=f"Final estimate ({ex[-1]:.2f}, {ey[-1]:.2f})")

    # Start marker
    if traj.shape[0] > 0:
        ax.plot(traj[0, 1], traj[0, 2], "o", color="#2ecc71",
                markersize=10, markeredgecolor="white", markeredgewidth=1,
                label="Start", zorder=6)

    ax.set_xlabel("x (m)", color="white")
    ax.set_ylabel("y (m)", color="white")
    ax.set_title("PHOENIX IMPPF — AP Localisation", color="white", fontsize=13)
    ax.tick_params(colors="white")
    for spine in ax.spines.values():
        spine.set_edgecolor("#444")
    leg = ax.legend(loc="upper right", fontsize=8,
                    facecolor="#1a1a2e", edgecolor="#444",
                    labelcolor="white")
    plt.tight_layout()

    if save_prefix:
        p = save_prefix + ".main.png"
        fig1.savefig(p, dpi=150, bbox_inches="tight",
                     facecolor=fig1.get_facecolor())
        print(f"Saved figure: {p}")
    else:
        plt.show()

    # ── Figure 2: evolution snapshots ─────────────────────────────────────────
    n_snap = min(9, len(estimates))
    if n_snap < 2:
        return

    step = max(1, len(estimates) // n_snap)
    snap_idxs = list(range(0, len(estimates), step))[:n_snap]

    cols_g = 3
    rows_g = math.ceil(n_snap / cols_g)
    fig2, axes = plt.subplots(rows_g, cols_g,
                               figsize=(4 * cols_g, 3.5 * rows_g))
    fig2.patch.set_facecolor("#0f0f23")
    axes = np.array(axes).flatten()

    for k, idx in enumerate(snap_idxs):
        ax2 = axes[k]
        ax2.set_facecolor("#1a1a2e")
        ax2.imshow(map_disp, origin="lower", extent=ext,
                   cmap="gray", vmin=0, vmax=1, alpha=0.5)
        # trajectory up to this point
        if traj.shape[0] > 1:
            t_cut = measurements[min(idx, len(measurements)-1)]["t"]
            mask = traj[:, 0] <= t_cut
            if mask.any():
                ax2.plot(traj[mask, 1], traj[mask, 2],
                         color="#4a9eff", linewidth=0.8, alpha=0.5)
        # estimate up to this point
        ex = [p[0] for p in estimates[:idx+1]]
        ey = [p[1] for p in estimates[:idx+1]]
        if ex:
            ax2.plot(ex, ey, color="#f39c12", linewidth=1.0, alpha=0.8)
            ax2.plot(ex[-1], ey[-1], "X", color="#f39c12",
                     markersize=8, markeredgecolor="white",
                     markeredgewidth=0.8)
        ax2.set_title(f"Step {idx+1}/{len(estimates)}\n"
                      f"est=({estimates[idx][0]:.2f}, {estimates[idx][1]:.2f})",
                      color="white", fontsize=7)
        ax2.tick_params(colors="white", labelsize=6)
        for sp in ax2.spines.values():
            sp.set_edgecolor("#444")

    for k in range(n_snap, len(axes)):
        axes[k].set_visible(False)

    fig2.suptitle("IMPPF Estimate Evolution", color="white", fontsize=11)
    plt.tight_layout()

    if save_prefix:
        p = save_prefix + ".evolution.png"
        fig2.savefig(p, dpi=150, bbox_inches="tight",
                     facecolor=fig2.get_facecolor())
        print(f"Saved figure: {p}")
    else:
        plt.show()

    plt.close("all")


# ============================================================================
# Entry point
# ============================================================================

def main():
    ap = argparse.ArgumentParser(
        description="PHOENIX IMPPF — real FTM RTT + SLAM AP localisation")
    ap.add_argument("--map",         required=True,  help="map.npy from logger")
    ap.add_argument("--map-meta",    required=True,  help="map_meta.json from logger")
    ap.add_argument("--traj",        required=True,  help="trajectory.csv from logger")
    ap.add_argument("--rtt",         required=True,  help="rtt.csv from logger")
    ap.add_argument("--particles",   type=int,   default=1000)
    ap.add_argument("--sigma-los",   type=float, default=0.5)
    ap.add_argument("--sigma-nlos",  type=float, default=1.5)
    ap.add_argument("--bias",        type=float, default=0.8,
                    help="Thin-wall NLOS additive bias (m)")
    ap.add_argument("--sigma-init",  type=float, default=0.5)
    ap.add_argument("--rough",       type=float, default=0.10)
    ap.add_argument("--offset-b",    type=float, default=0.0,
                    help="FTM systematic offset to subtract from d_median_m (m)")
    ap.add_argument("--likelihood",  default="gaussian",
                    choices=["gaussian", "student_t"],
                    help="student_t is more robust against RTT outliers")
    ap.add_argument("--student-t-dof", type=float, default=4.0)
    ap.add_argument("--seed",        type=int,   default=42)
    ap.add_argument("--save",        default=None,
                    help="Path prefix for output figures (omit to show interactively)")
    args = ap.parse_args()

    # ── Load data ──────────────────────────────────────────────────────────────
    print("Loading map …")
    map_array = np.load(args.map)
    with open(args.map_meta) as f:
        meta = json.load(f)

    print("Loading trajectory …")
    traj = load_trajectory(args.traj)

    print("Loading RTT bursts …")
    measurements = load_rtt(args.rtt, offset_b=args.offset_b)

    if not measurements:
        print("ERROR: no valid bursts after applying offset-b filter. "
              "Try a smaller --offset-b value.")
        return

    print(f"  Map:        {map_array.shape[1]}x{map_array.shape[0]} cells "
          f"@ {meta['resolution']:.3f} m/cell  "
          f"origin=({meta.get('origin_x',0):.2f}, {meta.get('origin_y',0):.2f})")
    print(f"  Trajectory: {traj.shape[0]} poses")
    print(f"  Bursts:     {len(measurements)} valid "
          f"(offset-b={args.offset_b:.2f} m applied)")
    d_vals = [m["d_meas"] for m in measurements]
    print(f"  Corrected d: min={min(d_vals):.2f}  max={max(d_vals):.2f}  "
          f"median={np.median(d_vals):.2f} m")

    # ── Build occupancy grid ───────────────────────────────────────────────────
    occ = OccupancyGrid.from_slam_map(map_array, meta)

    # ── Configure and run filter ───────────────────────────────────────────────
    cfg = IMPPFConfig(
        N              = args.particles,
        sigma_los      = args.sigma_los,
        sigma_nlos     = args.sigma_nlos,
        bias_thin_wall = args.bias,
        sigma_init     = args.sigma_init,
        rough_jitter_m = args.rough,
        seed           = args.seed,
        likelihood     = args.likelihood,
        student_t_dof  = args.student_t_dof,
    )

    filt      = IMPPF(occ, cfg)
    estimates: List[Tuple[float, float]] = []
    n         = len(measurements)

    print(f"\nRunning IMPPF ({args.likelihood} likelihood, "
          f"N={args.particles} particles) …")

    for i, m in enumerate(measurements):
        filt.step(np.array([m["x"], m["y"]]), m["d_meas"])
        estimates.append(filt.last_estimate)

        if (i + 1) % max(1, n // 10) == 0 or i == n - 1:
            print(f"  [{i+1:4d}/{n}]  est=({filt.last_estimate[0]:.2f}, "
                  f"{filt.last_estimate[1]:.2f})  "
                  f"LOS={filt.last_los_frac*100:.0f}%  "
                  f"Neff={filt.last_neff:.0f}"
                  f"{'  [resampled]' if filt.last_resampled else ''}")

    final = estimates[-1]
    print(f"\nFinal AP estimate: ({final[0]:.3f}, {final[1]:.3f})")
    print("(Compare with the surveyed AP position to compute error.)")

    # ── Plot ───────────────────────────────────────────────────────────────────
    plot_results(
        occ          = occ,
        meta         = meta,
        traj         = traj,
        measurements = measurements,
        estimates    = estimates,
        particles_final = filt.particles,
        cfg          = cfg,
        save_prefix  = args.save,
    )


if __name__ == "__main__":
    main()

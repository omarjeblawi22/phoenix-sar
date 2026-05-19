#!/usr/bin/env python3
"""
PHOENIX Creator
===============

Interactive Tk GUI for building synthetic FTM-RTT datasets and validating the
IMPPF (Interacting Multiple Particle Filter) before recording real data.

Workflow
--------
1.  Choose "Add wall" / "Set path" / "Place AP" and click on the canvas.
        - Walls    : click two points per segment. Repeat for more walls.
        - Path     : click waypoints. Between runs the path is interpolated
                     at a constant vehicle speed (default 0.4 m/s) and
                     sampled every `burst_interval` seconds (default 0.6 s)
                     to produce a list of (x, y, t) measurement poses.
        - AP       : single click sets the true AP location.

2.  Tune parameters (particles, sigmas, bias, speed, burst interval) in the
    parameter panel.

3.  Click "Generate dataset" to simulate FTM RTT measurements. Each pose
    along the interpolated path yields a noisy distance whose noise model
    is chosen by raycasting against the wall map (LOS or NLOS).

4.  Click "Run IMPPF" to run the filter on the dataset in a background
    thread. The canvas updates live as the filter converges.

5.  "Save run..." writes the world (walls + AP + path) to JSON and the
    measurements to CSV in the format expected by the offline IMPPF
    prototype.

The filter implementation matches the PHOENIX spec exactly:
    - Ring initialization on first measurement
    - Vectorized DDA raycast for per-particle LOS/NLOS classification
    - Gaussian likelihood with NLOS bias (eqs. 3, 4 in the spec)
    - Systematic resampling when N_eff < N/2, with post-resample roughening

Run:        python3 Creator.py
"""

from __future__ import annotations

import csv
import json
import math
import queue
import threading
import time
import tkinter as tk
from dataclasses import dataclass, field
from pathlib import Path
from tkinter import filedialog, messagebox, ttk
from typing import List, Optional, Tuple

import numpy as np
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

# Optional: use filterpy's reference systematic_resample when installed.
# https://github.com/rlabbe/filterpy (MIT licence, Roger R. Labbe Jr.)
try:
    from filterpy.monte_carlo import systematic_resample as _FILTERPY_RESAMPLE
except ImportError:                                    # pragma: no cover
    _FILTERPY_RESAMPLE = None


# ──────────────────────────────────────────────────────────────────────────────
# Occupancy grid + vectorized raycaster
# ──────────────────────────────────────────────────────────────────────────────
@dataclass
class OccupancyGrid:
    """
    Simple axis-aligned occupancy grid built from line segments (walls).

    cells[row, col] == 1 means occupied. Origin is at (0, 0) in world frame,
    cells[0, 0] covers x in [0, res), y in [0, res).
    """
    width: float = 30.0        # metres
    height: float = 30.0       # metres
    resolution: float = 0.1    # metres per cell
    cells: np.ndarray = field(default_factory=lambda: np.zeros((1, 1), dtype=np.uint8))

    @classmethod
    def from_walls(cls, walls, width=30.0, height=30.0, resolution=0.1,
                   wall_thickness: int = 1) -> "OccupancyGrid":
        cols = int(round(width / resolution))
        rows = int(round(height / resolution))
        cells = np.zeros((rows, cols), dtype=np.uint8)

        for (x1, y1, x2, y2) in walls:
            _rasterize_segment(cells, x1, y1, x2, y2, resolution, wall_thickness)

        return cls(width=width, height=height, resolution=resolution, cells=cells)

    def world_to_grid(self, xy: np.ndarray) -> np.ndarray:
        """Convert (..., 2) world coords to fractional grid coords (col, row)."""
        return np.asarray(xy, dtype=np.float64) / self.resolution


def _rasterize_segment(cells: np.ndarray, x1, y1, x2, y2,
                       res: float, thickness: int) -> None:
    """Bresenham rasterisation of a wall segment into the cell grid."""
    rows, cols = cells.shape
    c1 = int(round(x1 / res));  r1 = int(round(y1 / res))
    c2 = int(round(x2 / res));  r2 = int(round(y2 / res))

    dc = abs(c2 - c1)
    dr = abs(r2 - r1)
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
            err -= dr
            c += sc
        if e2 < dc:
            err += dc
            r += sr


def vectorized_los_check(robot_grid_xy: np.ndarray,
                         particles_grid_xy: np.ndarray,
                         cells: np.ndarray) -> np.ndarray:
    """
    Batch DDA raycast: for each particle, decide whether the straight line
    from the robot to that particle clears every occupied cell along its
    interior (endpoints excluded). Returns a bool array of length N where
    True == LOS.

    All inputs are in *grid* coordinates (col, row), fractional allowed.
    """
    rows, cols = cells.shape
    rx, ry = float(robot_grid_xy[0]), float(robot_grid_xy[1])
    px = particles_grid_xy[:, 0].astype(np.float64)
    py = particles_grid_xy[:, 1].astype(np.float64)

    dx = px - rx
    dy = py - ry

    # number of cells touched by the longest ray
    n_steps_per = np.maximum(np.abs(dx), np.abs(dy)).astype(np.int32) + 1
    max_steps = int(n_steps_per.max()) if n_steps_per.size else 1
    if max_steps < 2:
        return np.ones(particles_grid_xy.shape[0], dtype=bool)

    # parametric samples in [0,1] along every ray, broadcast to (N, S)
    t = np.linspace(0.0, 1.0, max_steps)
    xs = rx + dx[:, None] * t[None, :]
    ys = ry + dy[:, None] * t[None, :]

    ci = np.clip(np.round(xs).astype(np.int32), 0, cols - 1)
    ri = np.clip(np.round(ys).astype(np.int32), 0, rows - 1)

    # exclude endpoints so particles drawn inside walls don't self-block
    step_idx = np.arange(max_steps)[None, :]
    nsm1 = (n_steps_per - 1)[:, None]
    interior = (step_idx > 0) & (step_idx < nsm1)

    blocked = (cells[ri, ci] >= 1) & interior
    return ~blocked.any(axis=1)


# ──────────────────────────────────────────────────────────────────────────────
# IMPPF
# ──────────────────────────────────────────────────────────────────────────────
@dataclass
class IMPPFConfig:
    N: int = 1000
    sigma_los: float = 0.5
    sigma_nlos: float = 1.5
    bias_thin_wall: float = 0.8
    sigma_init: float = 0.5
    rough_jitter_m: float = 0.10
    seed: Optional[int] = 42
    # Robust likelihood: "gaussian" or "student_t".
    # Student-t with low dof (e.g. 3-5) has heavy tails — it is the standard
    # textbook fix for outlier-robust particle filtering. See e.g.
    # Thrun, Burgard & Fox "Probabilistic Robotics" §4.3, and
    # Roger Labbe's "Kalman and Bayesian Filters in Python" (filterpy),
    # chapter 12 on particle filters with non-Gaussian measurement noise.
    likelihood: str = "gaussian"
    student_t_dof: float = 4.0


class IMPPF:
    """Interacting Multiple Particle Filter for a stationary 2D target."""

    def __init__(self, occupancy: OccupancyGrid, cfg: IMPPFConfig):
        self.occ = occupancy
        self.cfg = cfg
        self.rng = np.random.default_rng(cfg.seed)

        self.particles: Optional[np.ndarray] = None   # (N, 2) world coords
        self.weights:   Optional[np.ndarray] = None   # (N,)
        self.initialized = False

        # diagnostics from last step
        self.last_estimate: Optional[Tuple[float, float]] = None
        self.last_los_frac: float = 1.0
        self.last_neff: float = float(cfg.N)
        self.last_resampled = False

    def initialize(self, robot_xy: np.ndarray, d_first: float) -> None:
        N = self.cfg.N
        theta = self.rng.uniform(0.0, 2.0 * math.pi, size=N)
        r = d_first + self.rng.normal(0.0, self.cfg.sigma_init, size=N)
        r = np.maximum(r, 0.05)
        x = robot_xy[0] + r * np.cos(theta)
        y = robot_xy[1] + r * np.sin(theta)
        self.particles = np.stack([x, y], axis=1)
        self.weights = np.full(N, 1.0 / N)
        self.initialized = True

    def step(self, robot_xy, d_rtt: float) -> None:
        robot_xy = np.asarray(robot_xy, dtype=np.float64)

        if not self.initialized:
            self.initialize(robot_xy, float(d_rtt))
            self.last_estimate = self.estimate()
            return

        # interaction: LOS / NLOS per particle
        robot_g = self.occ.world_to_grid(robot_xy)
        part_g = self.occ.world_to_grid(self.particles)
        is_los = vectorized_los_check(robot_g, part_g, self.occ.cells)
        self.last_los_frac = float(is_los.mean())

        # measurement update (eqs. 3, 4 in PHOENIX spec)
        d_hyp = np.linalg.norm(self.particles - robot_xy, axis=1)
        d_pred = np.where(is_los, d_hyp, d_hyp + self.cfg.bias_thin_wall)
        sigma = np.where(is_los, self.cfg.sigma_los, self.cfg.sigma_nlos)

        # log-likelihood for numerical stability
        residual = (d_rtt - d_pred) / sigma
        if self.cfg.likelihood == "student_t":
            # log Student-t pdf:  -((dof+1)/2) * log(1 + r²/dof) - log(sigma)
            #                     (omitting normaliser, since we renormalise)
            dof = max(self.cfg.student_t_dof, 1.0)
            log_lik = -0.5 * (dof + 1.0) * np.log1p(residual ** 2 / dof) \
                      - np.log(sigma)
        else:
            log_lik = -0.5 * residual ** 2 - np.log(sigma)
        log_lik -= log_lik.max()
        lik = np.exp(log_lik)

        self.weights = self.weights * lik
        s = self.weights.sum()
        if not np.isfinite(s) or s <= 0.0:
            # likelihood collapse: reset to uniform on existing particles
            self.weights = np.full(self.cfg.N, 1.0 / self.cfg.N)
        else:
            self.weights /= s

        # resample if degenerate
        neff = 1.0 / np.sum(self.weights ** 2)
        self.last_neff = float(neff)
        self.last_resampled = False
        if neff < self.cfg.N / 2.0:
            self._systematic_resample()
            # roughening: small Gaussian jitter to fight depletion
            if self.cfg.rough_jitter_m > 0.0:
                self.particles += self.rng.normal(
                    0.0, self.cfg.rough_jitter_m, self.particles.shape
                )
            self.last_resampled = True

        self.last_estimate = self.estimate()

    def _systematic_resample(self) -> None:
        N = self.cfg.N
        # Prefer the reference implementation from filterpy (MIT licensed,
        # by Roger Labbe) when installed; falls back to a local copy that
        # is line-for-line equivalent. The local fallback uses the same
        # algorithm as filterpy.monte_carlo.systematic_resample.
        if _FILTERPY_RESAMPLE is not None:
            idx = _FILTERPY_RESAMPLE(self.weights)
        else:
            positions = (np.arange(N) + self.rng.uniform()) / N
            cumsum = np.cumsum(self.weights)
            cumsum[-1] = 1.0
            idx = np.searchsorted(cumsum, positions)
            idx = np.clip(idx, 0, N - 1)
        self.particles = self.particles[idx]
        self.weights = np.full(N, 1.0 / N)

    def estimate(self) -> Tuple[float, float]:
        if self.particles is None:
            return (float("nan"), float("nan"))
        x = float(np.average(self.particles[:, 0], weights=self.weights))
        y = float(np.average(self.particles[:, 1], weights=self.weights))
        return (x, y)


# ──────────────────────────────────────────────────────────────────────────────
# Measurement simulation
# ──────────────────────────────────────────────────────────────────────────────
def interpolate_path(waypoints: List[Tuple[float, float]],
                     speed: float, dt: float) -> List[Tuple[float, float, float]]:
    """
    Given a list of (x, y) waypoints, return a list of (x, y, t) samples
    spaced at intervals of `dt` seconds along the polyline at constant speed.
    """
    if len(waypoints) < 2:
        return [(waypoints[0][0], waypoints[0][1], 0.0)] if waypoints else []

    out: List[Tuple[float, float, float]] = []
    t = 0.0
    step_len = speed * dt   # metres between samples

    # always include the very first point
    out.append((waypoints[0][0], waypoints[0][1], 0.0))
    leftover = 0.0  # leftover distance from previous segment

    for (x1, y1), (x2, y2) in zip(waypoints[:-1], waypoints[1:]):
        seg_len = math.hypot(x2 - x1, y2 - y1)
        if seg_len < 1e-9:
            continue
        ux = (x2 - x1) / seg_len
        uy = (y2 - y1) / seg_len

        d = step_len - leftover     # distance from segment start to next sample
        while d <= seg_len + 1e-9:
            x = x1 + ux * d
            y = y1 + uy * d
            t += dt
            out.append((x, y, t))
            d += step_len
        leftover = seg_len - (d - step_len)

    return out


# ──────────────────────────────────────────────────────────────────────────────
# Materials model — analytic ray vs. wall-with-thickness
# ──────────────────────────────────────────────────────────────────────────────
@dataclass
class MaterialWall:
    """
    A wall segment (x1,y1)→(x2,y2) with a finite thickness and material delay.

    bias_per_m  : extra distance bias *per metre of material traversed*.
                  Typical empirical values:
                    drywall   ~ 0.6  m/m   (very fast, thin walls)
                    wood door ~ 1.0  m/m
                    brick     ~ 2.0  m/m
                    concrete  ~ 3.0+ m/m
    sigma_extra : extra noise sigma added (in quadrature) per metre of material.
    """
    x1: float
    y1: float
    x2: float
    y2: float
    thickness: float = 0.10          # metres
    bias_per_m: float = 8.0          # bias per metre of *path through material*
                                     # (path includes thickness expansion, so
                                     #  this number is calibrated against
                                     #  perpendicular crossing of a wall)
    sigma_extra: float = 0.8         # extra σ per metre of material path


def _ray_segment_intersect_thick(rx, ry, ax, ay, w: MaterialWall):
    """
    Analytic intersection of ray (rx,ry)→(ax,ay) with a thickened line segment.

    Models the wall as the Minkowski sum of the centreline segment with a disk
    of radius `thickness/2`. This is the rectangle along the segment plus two
    semicircular caps — a good first-order approximation that captures both
    perpendicular and grazing incidence smoothly.

    Returns the length (in metres) of the portion of the ray that is inside
    the wall material, or 0.0 if no intersection. Capped at ray length.
    """
    px = ax - rx
    py = ay - ry
    ray_len = math.hypot(px, py)
    if ray_len < 1e-9:
        return 0.0
    dx = px / ray_len
    dy = py / ray_len

    # Build wall-local frame: u along centreline, v perpendicular
    wx = w.x2 - w.x1
    wy = w.y2 - w.y1
    wlen = math.hypot(wx, wy)
    if wlen < 1e-9:
        return 0.0
    ux = wx / wlen
    uy = wy / wlen
    vx = -uy
    vy = ux
    half_t = max(w.thickness, 1e-6) * 0.5

    # Robot in wall-local frame, with wall centreline as u-axis [0, wlen]
    ox = rx - w.x1
    oy = ry - w.y1
    r_u = ox * ux + oy * uy
    r_v = ox * vx + oy * vy
    d_u = dx * ux + dy * uy
    d_v = dx * vx + dy * vy

    # Ray param t in [0, ray_len]. Find t-interval where |v| <= half_t and u in [0, wlen].
    # Slab 1: |r_v + t*d_v| <= half_t
    t_in_v, t_out_v = -math.inf, math.inf
    if abs(d_v) < 1e-12:
        if abs(r_v) > half_t:
            return 0.0
    else:
        a = (-half_t - r_v) / d_v
        b = ( half_t - r_v) / d_v
        t_in_v, t_out_v = (a, b) if a < b else (b, a)

    # Slab 2: 0 <= r_u + t*d_u <= wlen
    t_in_u, t_out_u = -math.inf, math.inf
    if abs(d_u) < 1e-12:
        if r_u < 0.0 or r_u > wlen:
            # outside ends — endcap test below handles this
            t_in_u, t_out_u = math.inf, -math.inf
    else:
        a = (0.0 - r_u) / d_u
        b = (wlen - r_u) / d_u
        t_in_u, t_out_u = (a, b) if a < b else (b, a)

    # Rectangle intersection
    t_in = max(t_in_v, t_in_u, 0.0)
    t_out = min(t_out_v, t_out_u, ray_len)
    rect_len = max(0.0, t_out - t_in)

    # Endcap discs at (0,0) and (wlen,0) in local frame, radius half_t
    def disc_len(cu, cv):
        # ray origin offset from disc centre
        ex = r_u - cu
        ey = r_v - cv
        bcoef = ex * d_u + ey * d_v
        ccoef = ex * ex + ey * ey - half_t * half_t
        disc = bcoef * bcoef - ccoef
        if disc <= 0.0:
            return 0.0
        s = math.sqrt(disc)
        t0 = -bcoef - s
        t1 = -bcoef + s
        t0 = max(t0, 0.0)
        t1 = min(t1, ray_len)
        if t1 <= t0:
            return 0.0
        # Only count portion *outside* the rectangle u∈[0,wlen]; we'll Min/Max
        # to avoid double counting after.
        return t1 - t0

    cap_a = disc_len(0.0, 0.0)
    cap_b = disc_len(wlen, 0.0)

    # The rectangle already covers the part of the discs that overlaps the
    # rectangle slab in u. To avoid double-count we take a conservative
    # upper bound: max(rect, cap) per side. In practice rays cross at most
    # one endcap, so this is fine.
    total = rect_len + max(0.0, cap_a + cap_b - rect_len * 0.0)
    # Cap by ray_len just in case
    return min(total, ray_len)


def compute_ray_material_path(rx, ry, ax, ay, walls):
    """
    Returns (total_path_through_material_metres, list_of_walls_hit).
    walls is a list of MaterialWall objects.
    """
    total = 0.0
    hit = []
    for w in walls:
        L = _ray_segment_intersect_thick(rx, ry, ax, ay, w)
        if L > 1e-6:
            total += L
            hit.append((w, L))
    return total, hit


# ──────────────────────────────────────────────────────────────────────────────
# Measurement simulation
# ──────────────────────────────────────────────────────────────────────────────
def simulate_measurements(path_samples, ap_xy, material_walls,
                          *,
                          sigma_los, sigma_nlos_floor,
                          bias_nlos_floor, multipath_sigma_los,
                          outlier_rate, outlier_scale,
                          dropout_rate, quantize_ns,
                          clock_bias_m, pose_noise_xy,
                          rng: np.random.Generator):
    """
    Generate realistic ESP32-FTM-like RTT measurements along the path.

    Noise model per sample (all distances in metres):
        d_meas = d_true
               + Σ_wall (bias_per_m_w · L_w)           ← per-material delay
               + (LOS_extra | NLOS_extra)              ← Gaussian core
               + clock_bias_m                          ← constant per run
               + multipath_offset (LOS only, prob.)    ← positive Gaussian
               + outlier (rare exponential tail)       ← long-tail OR fault
        then quantized to `quantize_ns` ns equivalent distance.

    `dropout_rate` ∈ [0,1] randomly omits whole samples (filter sees nothing).

    Returns: list of dicts. Dropped samples are *not* present in the list
    (mimicking how the real logger only writes successful FTM bursts).
    """
    out = []
    ap = np.asarray(ap_xy, dtype=np.float64)
    quant_m = (quantize_ns * 1e-9) * 299_792_458.0 / 2.0  # one-way distance per ns of RTT

    for (x, y, t) in path_samples:
        # Random burst dropout (filter never sees this sample)
        if dropout_rate > 0.0 and rng.random() < dropout_rate:
            continue

        robot_true = np.array([x, y])
        d_true = float(np.linalg.norm(ap - robot_true))

        # Material path (analytic, thickness-aware)
        mat_len, hits = compute_ray_material_path(x, y, ap[0], ap[1], material_walls)
        is_los_truth = (mat_len < 1e-6)

        # Per-material bias and extra noise (compounds linearly)
        bias_material = 0.0
        sigma_material_sq = 0.0
        for w, L in hits:
            bias_material += w.bias_per_m * L
            sigma_material_sq += (w.sigma_extra * L) ** 2

        if is_los_truth:
            sigma = sigma_los
            bias = 0.0
            # LOS multipath: small positive offset with some prob.
            mp = 0.0
            if multipath_sigma_los > 0.0:
                # half-normal: |N(0, σ)| -> always non-negative offset
                mp = abs(rng.normal(0.0, multipath_sigma_los))
            noise = rng.normal(0.0, sigma) + mp
            label = "LOS"
        else:
            sigma = math.sqrt(sigma_nlos_floor ** 2 + sigma_material_sq)
            bias = bias_nlos_floor + bias_material
            noise = rng.normal(0.0, sigma)
            label = "NLOS"

        # Heavy-tailed outliers (positive only — FTM outliers are reflections,
        # which add path, not subtract): rare exponential burst on top.
        outlier = 0.0
        if outlier_rate > 0.0 and rng.random() < outlier_rate:
            outlier = rng.exponential(outlier_scale)

        # Pose noise — what the SLAM pose looks like vs. true pose.
        # The filter receives the *noisy* pose; the simulator records both.
        if pose_noise_xy > 0.0:
            noisy_pose = (
                x + rng.normal(0.0, pose_noise_xy),
                y + rng.normal(0.0, pose_noise_xy),
            )
        else:
            noisy_pose = (x, y)

        d_meas = d_true + bias + noise + outlier + clock_bias_m

        # Quantization (round to nearest distance corresponding to quantize_ns
        # of RTT). Skip if quant_m == 0.
        if quant_m > 0.0:
            d_meas = round(d_meas / quant_m) * quant_m

        d_meas = max(d_meas, 0.0)

        out.append({
            "t": t,
            "x": noisy_pose[0], "y": noisy_pose[1],   # what the filter sees
            "x_true": x, "y_true": y,                  # ground truth pose
            "d_true": d_true, "d_meas": d_meas,
            "label": label, "mat_len": mat_len,
            "outlier": bool(outlier > 0.0),
        })
    return out


# ──────────────────────────────────────────────────────────────────────────────
# GUI
# ──────────────────────────────────────────────────────────────────────────────
MAP_SIZE = 30.0
RES = 0.1
WALL_THICKNESS = 2

MODE_WALL = "wall"
MODE_PATH = "path"
MODE_AP   = "ap"


class CreatorApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        root.title("PHOENIX Creator — synthetic FTM-RTT dataset")
        root.protocol("WM_DELETE_WINDOW", self.on_closing)

        # world state
        self.walls: List[Tuple[float, float, float, float]] = []  # (x1,y1,x2,y2)
        self.waypoints: List[Tuple[float, float]] = []
        self.ap: Optional[Tuple[float, float]] = None

        # interaction state
        self.mode = MODE_WALL
        self.wall_first_click: Optional[Tuple[float, float]] = None

        # results
        self.measurements: Optional[list] = None
        self.estimates: List[Tuple[float, float]] = []
        self.particles: Optional[np.ndarray] = None
        self.rover_pose: Optional[Tuple[float, float, str]] = None  # (x, y, label) during run
        self.filter_thread: Optional[threading.Thread] = None
        self.filter_stop = threading.Event()
        self.update_queue: "queue.Queue" = queue.Queue()

        self._build_ui()
        self._refresh_plot()
        self._poll_updates()

    # ────────────────────────────────── UI scaffolding ────────────────────────
    def _build_ui(self) -> None:
        top = ttk.Frame(self.root, padding=4)
        top.pack(side=tk.TOP, fill=tk.X)

        # mode buttons
        self.mode_var = tk.StringVar(value=MODE_WALL)
        for label, mode in (("Add wall",  MODE_WALL),
                            ("Set path", MODE_PATH),
                            ("Place AP", MODE_AP)):
            ttk.Radiobutton(top, text=label, value=mode,
                            variable=self.mode_var,
                            command=self._on_mode_change).pack(side=tk.LEFT, padx=2)

        ttk.Separator(top, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=6)
        ttk.Button(top, text="Undo last", command=self._undo_last).pack(side=tk.LEFT, padx=2)
        ttk.Button(top, text="Clear all", command=self._clear_all).pack(side=tk.LEFT, padx=2)
        ttk.Separator(top, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=6)
        ttk.Button(top, text="Generate dataset", command=self._generate_dataset).pack(side=tk.LEFT, padx=2)
        ttk.Button(top, text="Run IMPPF",        command=self._run_imppf).pack(side=tk.LEFT, padx=2)
        ttk.Button(top, text="Stop",             command=self._stop_filter).pack(side=tk.LEFT, padx=2)
        ttk.Separator(top, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=6)
        ttk.Button(top, text="Save run…",        command=self._save_run).pack(side=tk.LEFT, padx=2)
        ttk.Button(top, text="Load run…",        command=self._load_run).pack(side=tk.LEFT, padx=2)

        # parameter rows
        params_frame = ttk.Frame(self.root)
        params_frame.pack(side=tk.TOP, fill=tk.X, padx=4, pady=2)

        filter_params = ttk.LabelFrame(params_frame, text="Filter (IMPPF)", padding=4)
        filter_params.pack(side=tk.TOP, fill=tk.X, pady=1)

        sim_params = ttk.LabelFrame(params_frame, text="Simulator (RTT physics)", padding=4)
        sim_params.pack(side=tk.TOP, fill=tk.X, pady=1)

        self.params = {
            # filter
            "particles":      tk.StringVar(value="1000"),
            "sigma_los":      tk.StringVar(value="0.5"),
            "sigma_nlos":     tk.StringVar(value="1.5"),
            "bias_nlos":      tk.StringVar(value="0.8"),
            "sigma_init":     tk.StringVar(value="0.5"),
            "rough_jitter":   tk.StringVar(value="0.10"),
            "speed":          tk.StringVar(value="0.4"),
            "burst_interval": tk.StringVar(value="0.6"),
            "seed":           tk.StringVar(value="42"),
            # simulator — realism knobs
            "wall_thickness":   tk.StringVar(value="0.10"),
            "sim_sigma_los":    tk.StringVar(value="0.30"),
            "sim_sigma_nlos":   tk.StringVar(value="0.40"),  # per-material floor
            "sim_bias_nlos":    tk.StringVar(value="0.20"),  # per-material floor
            "bias_per_m":       tk.StringVar(value="8.0"),
            "sigma_per_m":      tk.StringVar(value="6.0"),
            "multipath_los":    tk.StringVar(value="0.15"),
            "outlier_rate":     tk.StringVar(value="0.03"),
            "outlier_scale":    tk.StringVar(value="3.0"),
            "dropout_rate":     tk.StringVar(value="0.05"),
            "quantize_ns":      tk.StringVar(value="1.0"),
            "clock_bias_m":     tk.StringVar(value="0.0"),
            "pose_noise_xy":    tk.StringVar(value="0.05"),
        }

        filter_fields = [
            ("Particles", "particles"), ("σ LOS (m)", "sigma_los"),
            ("σ NLOS (m)", "sigma_nlos"), ("NLOS bias (m)", "bias_nlos"),
            ("σ init (m)", "sigma_init"), ("roughen (m)", "rough_jitter"),
            ("speed (m/s)", "speed"), ("burst dt (s)", "burst_interval"),
            ("seed", "seed"),
        ]
        for i, (lbl, key) in enumerate(filter_fields):
            ttk.Label(filter_params, text=lbl).grid(row=0, column=2 * i, padx=(6, 2), pady=2, sticky="e")
            ttk.Entry(filter_params, textvariable=self.params[key], width=6) \
                .grid(row=0, column=2 * i + 1, padx=(0, 4), pady=2)

        # Likelihood selector + dof for Student-t
        col = 2 * len(filter_fields)
        ttk.Label(filter_params, text="Likelihood:").grid(row=0, column=col,
                                                          padx=(12, 2), pady=2, sticky="e")
        self.likelihood_var = tk.StringVar(value="gaussian")
        like_box = ttk.Combobox(filter_params, textvariable=self.likelihood_var,
                                values=("gaussian", "student_t"), state="readonly",
                                width=10)
        like_box.grid(row=0, column=col + 1, padx=(0, 4), pady=2)

        ttk.Label(filter_params, text="t dof:").grid(row=0, column=col + 2,
                                                     padx=(8, 2), pady=2, sticky="e")
        self.params["student_t_dof"] = tk.StringVar(value="4.0")
        ttk.Entry(filter_params, textvariable=self.params["student_t_dof"],
                  width=4).grid(row=0, column=col + 3, padx=(0, 4), pady=2)

        # Preset selector lives at the start of the simulator row
        ttk.Label(sim_params, text="Preset:").grid(row=0, column=0, padx=(6, 2), pady=2, sticky="e")
        self.preset_var = tk.StringVar(value="Realistic")
        preset_box = ttk.Combobox(sim_params, textvariable=self.preset_var, width=10,
                                   values=("Ideal", "Realistic", "Harsh", "Custom"),
                                   state="readonly")
        preset_box.grid(row=0, column=1, padx=(0, 8), pady=2)
        preset_box.bind("<<ComboboxSelected>>", lambda e: self._apply_preset())

        sim_fields = [
            ("wall t (m)",  "wall_thickness"),
            ("σ LOS (m)",   "sim_sigma_los"),
            ("σ NLOS₀ (m)", "sim_sigma_nlos"),
            ("bias₀ (m)",   "sim_bias_nlos"),
            ("bias/m",      "bias_per_m"),
            ("σ/m",         "sigma_per_m"),
            ("σ mp LOS",    "multipath_los"),
            ("outl rate",   "outlier_rate"),
            ("outl scale",  "outlier_scale"),
            ("dropout",     "dropout_rate"),
            ("quant (ns)",  "quantize_ns"),
            ("clk bias",    "clock_bias_m"),
            ("pose σ",      "pose_noise_xy"),
        ]
        for i, (lbl, key) in enumerate(sim_fields):
            ttk.Label(sim_params, text=lbl).grid(row=0, column=2 + 2 * i, padx=(6, 2), pady=2, sticky="e")
            ttk.Entry(sim_params, textvariable=self.params[key], width=5) \
                .grid(row=0, column=2 + 2 * i + 1, padx=(0, 4), pady=2)

        # status bar
        self.status_var = tk.StringVar(value="Mode: Add wall — click two points per wall.")
        ttk.Label(self.root, textvariable=self.status_var, anchor="w", relief="sunken") \
            .pack(side=tk.BOTTOM, fill=tk.X)

        # matplotlib canvas
        self.fig = Figure(figsize=(8, 8), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self._reset_axes()

        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        self.canvas.mpl_connect("button_press_event", self._on_click)

    def _reset_axes(self) -> None:
        self.ax.clear()
        self.ax.set_xlim(0.0, MAP_SIZE)
        self.ax.set_ylim(0.0, MAP_SIZE)
        self.ax.set_aspect("equal")
        self.ax.set_xlabel("x (m)")
        self.ax.set_ylabel("y (m)")
        self.ax.grid(True, alpha=0.3)

    # ────────────────────────────────── interaction ───────────────────────────
    def _on_mode_change(self) -> None:
        self.mode = self.mode_var.get()
        self.wall_first_click = None
        hints = {
            MODE_WALL: "click two points per wall",
            MODE_PATH: "click waypoints in order",
            MODE_AP:   "click once to place the AP",
        }
        self.status_var.set(f"Mode: {self.mode} — {hints[self.mode]}.")

    def _on_click(self, event) -> None:
        if event.xdata is None or event.ydata is None:
            return
        x, y = float(event.xdata), float(event.ydata)
        if not (0.0 <= x <= MAP_SIZE and 0.0 <= y <= MAP_SIZE):
            return

        if self.mode == MODE_WALL:
            if self.wall_first_click is None:
                self.wall_first_click = (x, y)
                self.status_var.set(f"Wall start ({x:.2f}, {y:.2f}). Click endpoint.")
            else:
                x1, y1 = self.wall_first_click
                self.walls.append((x1, y1, x, y))
                self.wall_first_click = None
                self.status_var.set(
                    f"Wall added — {len(self.walls)} total. Click two more points for another."
                )
                self._refresh_plot()

        elif self.mode == MODE_PATH:
            self.waypoints.append((x, y))
            self.status_var.set(
                f"Waypoint {len(self.waypoints)} at ({x:.2f}, {y:.2f})."
            )
            self._refresh_plot()

        elif self.mode == MODE_AP:
            self.ap = (x, y)
            self.status_var.set(f"AP placed at ({x:.2f}, {y:.2f}).")
            self._refresh_plot()

    def _undo_last(self) -> None:
        if self.mode == MODE_WALL and self.walls:
            self.walls.pop()
        elif self.mode == MODE_PATH and self.waypoints:
            self.waypoints.pop()
        elif self.mode == MODE_AP:
            self.ap = None
        self._refresh_plot()

    def _clear_all(self) -> None:
        self._stop_filter()
        self.walls.clear()
        self.waypoints.clear()
        self.ap = None
        self.wall_first_click = None
        self.measurements = None
        self.estimates = []
        self.particles = None
        self.rover_pose = None
        self._refresh_plot()
        self.status_var.set("Cleared.")

    # ────────────────────────────────── plotting ──────────────────────────────
    def _refresh_plot(self) -> None:
        self._reset_axes()

        # particles (back layer)
        if self.particles is not None and len(self.particles):
            self.ax.scatter(self.particles[:, 0], self.particles[:, 1],
                            s=2, c="gray", alpha=0.35, zorder=1)

        # walls — drawn with actual thickness as a filled band
        wt = self._parse_float("wall_thickness", 0.10)
        # matplotlib linewidth is in points; rough conversion to data units
        # is unreliable across DPIs, so draw thickness as a Rectangle-equivalent
        # using a fat line with capstyle='round' and a thin centreline overlay.
        # Width in pts: data-units * (axes height in pts / data range)
        for (x1, y1, x2, y2) in self.walls:
            # Compute perpendicular offset for filled polygon
            dx, dy = x2 - x1, y2 - y1
            L = math.hypot(dx, dy)
            if L < 1e-9:
                continue
            nx, ny = -dy / L * wt / 2, dx / L * wt / 2
            poly_x = [x1 + nx, x2 + nx, x2 - nx, x1 - nx]
            poly_y = [y1 + ny, y2 + ny, y2 - ny, y1 - ny]
            self.ax.fill(poly_x, poly_y, color="black", alpha=0.55, zorder=2)
            self.ax.plot([x1, x2], [y1, y2], "k-", linewidth=1.0, zorder=3)

        # waypoints + path
        if self.waypoints:
            xs, ys = zip(*self.waypoints)
            self.ax.plot(xs, ys, "b-o", linewidth=2, markersize=5,
                         label="path waypoints", zorder=4)

        # measurement poses
        if self.measurements:
            mx = [m["x"] for m in self.measurements]
            my = [m["y"] for m in self.measurements]
            colors = ["#2ca02c" if m["label"] == "LOS" else "#d62728"
                      for m in self.measurements]
            self.ax.scatter(mx, my, s=18, c=colors, alpha=0.7,
                            edgecolors="black", linewidths=0.3, zorder=5,
                            label="measurements (green=LOS, red=NLOS)")
            # mark outliers with a distinct cross
            ox = [m["x"] for m in self.measurements if m.get("outlier")]
            oy = [m["y"] for m in self.measurements if m.get("outlier")]
            if ox:
                self.ax.scatter(ox, oy, s=80, c="none", edgecolors="#9467bd",
                                linewidths=1.6, marker="o", zorder=6,
                                label="outliers")

        # filter trace
        if self.estimates:
            ex = [p[0] for p in self.estimates]
            ey = [p[1] for p in self.estimates]
            self.ax.plot(ex, ey, color="orange", linewidth=1.2, alpha=0.8,
                         label="estimate trace", zorder=6)
            self.ax.plot(ex[-1], ey[-1], "X", color="orange",
                         markersize=14, markeredgecolor="black",
                         markeredgewidth=1.0, zorder=7,
                         label=f"estimate ({ex[-1]:.2f}, {ey[-1]:.2f})")

        # moving rover marker (only during a running filter)
        if self.rover_pose is not None:
            rx, ry, rlabel = self.rover_pose
            face = "#2ca02c" if rlabel == "LOS" else "#d62728"
            self.ax.plot(rx, ry, "o", color=face, markersize=14,
                         markeredgecolor="black", markeredgewidth=1.4,
                         zorder=9, label=f"rover ({rlabel})")
            # line from rover to current estimate, hinting at the residual
            if self.estimates:
                ex, ey = self.estimates[-1]
                self.ax.plot([rx, ex], [ry, ey], "--",
                             color="orange", linewidth=0.8, alpha=0.5,
                             zorder=8)

        # true AP
        if self.ap is not None:
            self.ax.plot(self.ap[0], self.ap[1], "*", color="red",
                         markersize=18, markeredgecolor="black",
                         markeredgewidth=0.8, label="true AP", zorder=8)

        if any([self.waypoints, self.measurements, self.estimates, self.ap]):
            self.ax.legend(loc="upper right", fontsize=8)

        self.canvas.draw_idle()

    # ────────────────────────────────── dataset gen ───────────────────────────
    def _parse_float(self, key: str, default: float) -> float:
        try:
            return float(self.params[key].get())
        except (ValueError, TypeError):
            return default

    def _parse_int(self, key: str, default: int) -> int:
        try:
            return int(float(self.params[key].get()))
        except (ValueError, TypeError):
            return default

    def _validate_world(self) -> bool:
        if not self.waypoints or len(self.waypoints) < 2:
            messagebox.showwarning("Missing data",
                                   "Need at least two path waypoints.")
            return False
        if self.ap is None:
            messagebox.showwarning("Missing data", "Place the true AP first.")
            return False
        return True

    def _build_occupancy(self) -> OccupancyGrid:
        # Convert wall thickness in metres to a half-width in cells (≥1)
        wt_m = self._parse_float("wall_thickness", 0.10)
        wt_cells = max(1, int(round(wt_m / RES)))
        return OccupancyGrid.from_walls(
            self.walls,
            width=MAP_SIZE, height=MAP_SIZE,
            resolution=RES, wall_thickness=wt_cells,
        )

    def _build_material_walls(self) -> List[MaterialWall]:
        wt = self._parse_float("wall_thickness", 0.10)
        bpm = self._parse_float("bias_per_m", 8.0)
        spm = self._parse_float("sigma_per_m", 6.0)
        return [
            MaterialWall(x1=x1, y1=y1, x2=x2, y2=y2,
                         thickness=wt, bias_per_m=bpm, sigma_extra=spm)
            for (x1, y1, x2, y2) in self.walls
        ]

    PRESETS = {
        "Ideal": {
            # Simulator
            "wall_thickness": "0.10",
            "sim_sigma_los": "0.30", "sim_sigma_nlos": "0.20",
            "sim_bias_nlos": "0.0",
            "bias_per_m": "0.0", "sigma_per_m": "0.0",
            "multipath_los": "0.0", "outlier_rate": "0.0",
            "outlier_scale": "0.0", "dropout_rate": "0.0",
            "quantize_ns": "0.0", "clock_bias_m": "0.0",
            "pose_noise_xy": "0.0",
            # Filter — matched to a noise-free simulator
            "sigma_los": "0.30", "sigma_nlos": "0.30",
            "bias_nlos": "0.0",
        },
        "Realistic": {
            "wall_thickness": "0.10",
            "sim_sigma_los": "0.30", "sim_sigma_nlos": "0.40",
            "sim_bias_nlos": "0.20",
            "bias_per_m": "8.0", "sigma_per_m": "6.0",
            "multipath_los": "0.15", "outlier_rate": "0.03",
            "outlier_scale": "3.0", "dropout_rate": "0.05",
            "quantize_ns": "1.0", "clock_bias_m": "0.0",
            "pose_noise_xy": "0.05",
            # Filter — covers material bias on average for a 10 cm wall
            "sigma_los": "0.5", "sigma_nlos": "1.5",
            "bias_nlos": "0.8",
        },
        "Harsh": {
            "wall_thickness": "0.20",
            "sim_sigma_los": "0.50", "sim_sigma_nlos": "0.80",
            "sim_bias_nlos": "0.40",
            "bias_per_m": "15.0", "sigma_per_m": "12.0",
            "multipath_los": "0.40", "outlier_rate": "0.10",
            "outlier_scale": "6.0", "dropout_rate": "0.15",
            "quantize_ns": "1.0", "clock_bias_m": "0.30",
            "pose_noise_xy": "0.15",
            # Filter — loosened for heavy noise but cannot fully model the worst tails
            "sigma_los": "0.7", "sigma_nlos": "2.5",
            "bias_nlos": "1.5",
        },
    }

    def _apply_preset(self) -> None:
        name = self.preset_var.get()
        if name not in self.PRESETS:
            return
        for k, v in self.PRESETS[name].items():
            if k in self.params:
                self.params[k].set(v)
        self.status_var.set(
            f"Applied '{name}' preset — both simulator and filter parameters set."
        )

    def _generate_dataset(self) -> None:
        if not self._validate_world():
            return

        speed = max(self._parse_float("speed", 0.4), 0.01)
        dt = max(self._parse_float("burst_interval", 0.6), 0.01)
        seed = self._parse_int("seed", 42)

        path_samples = interpolate_path(self.waypoints, speed, dt)
        if len(path_samples) < 2:
            messagebox.showwarning(
                "Path too short",
                "Interpolated path has fewer than 2 samples. "
                "Add more waypoints or reduce burst interval."
            )
            return

        material_walls = self._build_material_walls()
        rng = np.random.default_rng(seed)
        self.measurements = simulate_measurements(
            path_samples, self.ap, material_walls,
            sigma_los          = self._parse_float("sim_sigma_los",   0.30),
            sigma_nlos_floor   = self._parse_float("sim_sigma_nlos",  0.40),
            bias_nlos_floor    = self._parse_float("sim_bias_nlos",   0.20),
            multipath_sigma_los= self._parse_float("multipath_los",   0.15),
            outlier_rate       = self._parse_float("outlier_rate",    0.03),
            outlier_scale      = self._parse_float("outlier_scale",   3.0),
            dropout_rate       = self._parse_float("dropout_rate",    0.05),
            quantize_ns        = self._parse_float("quantize_ns",     1.0),
            clock_bias_m       = self._parse_float("clock_bias_m",    0.0),
            pose_noise_xy      = self._parse_float("pose_noise_xy",   0.05),
            rng=rng,
        )

        n_los = sum(1 for m in self.measurements if m["label"] == "LOS")
        n_nlos = len(self.measurements) - n_los
        n_out = sum(1 for m in self.measurements if m.get("outlier"))
        n_drop = len(path_samples) - len(self.measurements)
        self.status_var.set(
            f"Dataset: {len(self.measurements)}/{len(path_samples)} samples "
            f"— {n_los} LOS, {n_nlos} NLOS, {n_out} outliers, {n_drop} dropped. "
            f"Click Run IMPPF."
        )
        self.estimates = []
        self.particles = None
        self._refresh_plot()

    # ────────────────────────────────── filter run ────────────────────────────
    def _run_imppf(self) -> None:
        if not self.measurements:
            messagebox.showwarning("No data",
                                   "Generate a dataset first.")
            return
        if self.filter_thread is not None and self.filter_thread.is_alive():
            messagebox.showinfo("Busy", "Filter already running.")
            return

        cfg = IMPPFConfig(
            N=self._parse_int("particles", 1000),
            sigma_los=self._parse_float("sigma_los", 0.5),
            sigma_nlos=self._parse_float("sigma_nlos", 1.5),
            bias_thin_wall=self._parse_float("bias_nlos", 0.8),
            sigma_init=self._parse_float("sigma_init", 0.5),
            rough_jitter_m=self._parse_float("rough_jitter", 0.10),
            seed=self._parse_int("seed", 42),
            likelihood=self.likelihood_var.get(),
            student_t_dof=self._parse_float("student_t_dof", 4.0),
        )
        occ = self._build_occupancy()

        self.estimates = []
        self.particles = None
        self.rover_pose = None
        self.filter_stop.clear()
        self.filter_thread = threading.Thread(
            target=self._filter_worker,
            args=(occ, cfg, list(self.measurements)),
            daemon=True,
        )
        self.filter_thread.start()
        self.status_var.set("Running IMPPF…")

    def _stop_filter(self) -> None:
        self.filter_stop.set()

    def _filter_worker(self, occ, cfg, measurements):
        filt = IMPPF(occ, cfg)
        t_start = time.perf_counter()

        # Update GUI every K steps to avoid overwhelming the event loop.
        K = max(1, len(measurements) // 60)

        for i, m in enumerate(measurements):
            if self.filter_stop.is_set():
                break
            filt.step(np.array([m["x"], m["y"]]), m["d_meas"])

            if i % K == 0 or i == len(measurements) - 1:
                self.update_queue.put({
                    "type": "step",
                    "i": i,
                    "n": len(measurements),
                    "estimate": filt.last_estimate,
                    "particles": filt.particles.copy(),
                    "los_frac": filt.last_los_frac,
                    "neff": filt.last_neff,
                    "rover": (m["x"], m["y"]),
                    "label": m["label"],
                })

        elapsed = time.perf_counter() - t_start
        self.update_queue.put({
            "type": "done",
            "elapsed": elapsed,
            "n_steps": len(measurements),
            "estimate": filt.last_estimate,
            "particles": filt.particles.copy() if filt.particles is not None else None,
        })

    def _poll_updates(self) -> None:
        """Pump worker updates onto the Tk main thread."""
        try:
            redraw = False
            while True:
                msg = self.update_queue.get_nowait()
                if msg["type"] == "step":
                    self.estimates.append(msg["estimate"])
                    self.particles = msg["particles"]
                    self.rover_pose = (msg["rover"][0], msg["rover"][1], msg["label"])
                    self.status_var.set(
                        f"IMPPF step {msg['i']+1}/{msg['n']}  "
                        f"est=({msg['estimate'][0]:.2f}, {msg['estimate'][1]:.2f})  "
                        f"LOS={msg['los_frac']*100:.0f}%  "
                        f"Neff={msg['neff']:.0f}"
                    )
                    redraw = True
                elif msg["type"] == "done":
                    self.particles = msg["particles"]
                    self.rover_pose = None    # done — stop showing the moving dot
                    est = msg["estimate"]
                    if est and self.ap:
                        err = math.hypot(est[0] - self.ap[0], est[1] - self.ap[1])
                        self.status_var.set(
                            f"Done in {msg['elapsed']*1000:.0f} ms over {msg['n_steps']} steps. "
                            f"Final est=({est[0]:.2f}, {est[1]:.2f})  "
                            f"error={err:.2f} m"
                        )
                    else:
                        self.status_var.set(
                            f"Done in {msg['elapsed']*1000:.0f} ms."
                        )
                    redraw = True
            # unreachable
        except queue.Empty:
            pass
        if redraw:
            self._refresh_plot()
        self.root.after(50, self._poll_updates)

    # ────────────────────────────────── save / load ───────────────────────────
    def _save_run(self) -> None:
        if not self._validate_world():
            return

        out_dir = filedialog.askdirectory(title="Choose output folder")
        if not out_dir:
            return
        out = Path(out_dir)
        out.mkdir(parents=True, exist_ok=True)

        world = {
            "map_size": MAP_SIZE,
            "resolution": RES,
            "walls": list(self.walls),
            "waypoints": list(self.waypoints),
            "ap": list(self.ap),
            "params": {k: v.get() for k, v in self.params.items()},
        }
        (out / "world.json").write_text(json.dumps(world, indent=2))

        # CSV of measurements, in the format the offline IMPPF reads
        if self.measurements:
            with (out / "rtt.csv").open("w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["t", "x", "y", "x_true", "y_true",
                            "d_true", "d_meas", "label", "mat_len", "outlier"])
                for m in self.measurements:
                    w.writerow([
                        f"{m['t']:.3f}",
                        f"{m['x']:.4f}", f"{m['y']:.4f}",
                        f"{m.get('x_true', m['x']):.4f}",
                        f"{m.get('y_true', m['y']):.4f}",
                        f"{m['d_true']:.4f}", f"{m['d_meas']:.4f}",
                        m["label"],
                        f"{m.get('mat_len', 0.0):.4f}",
                        int(bool(m.get("outlier", False))),
                    ])

            # trajectory CSV in the format expected by the IMPPF prototype
            with (out / "trajectory.csv").open("w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["t", "x", "y", "theta"])
                prev = None
                for m in self.measurements:
                    if prev is None:
                        theta = 0.0
                    else:
                        theta = math.atan2(m["y"] - prev["y"], m["x"] - prev["x"])
                    w.writerow([f"{m['t']:.3f}", f"{m['x']:.4f}", f"{m['y']:.4f}",
                                f"{theta:.4f}"])
                    prev = m

            # also save the rasterized map for the offline IMPPF
            occ = self._build_occupancy()
            np.save(out / "map.npy", occ.cells)
            (out / "map_meta.json").write_text(json.dumps({
                "resolution": RES,
                "origin_x": 0.0,
                "origin_y": 0.0,
                "width": MAP_SIZE,
                "height": MAP_SIZE,
            }, indent=2))

        self.status_var.set(f"Saved run to {out}")

    def _load_run(self) -> None:
        path = filedialog.askopenfilename(
            title="Open world.json",
            filetypes=[("JSON", "*.json"), ("All", "*.*")],
        )
        if not path:
            return
        try:
            world = json.loads(Path(path).read_text())
            self.walls = [tuple(w) for w in world.get("walls", [])]
            self.waypoints = [tuple(p) for p in world.get("waypoints", [])]
            ap = world.get("ap")
            self.ap = tuple(ap) if ap else None
            params = world.get("params", {})
            for k, v in params.items():
                if k in self.params:
                    self.params[k].set(str(v))
        except (json.JSONDecodeError, OSError, ValueError) as e:
            messagebox.showerror("Load failed", str(e))
            return
        self.measurements = None
        self.estimates = []
        self.particles = None
        self._refresh_plot()
        self.status_var.set(f"Loaded {path}")

    # ────────────────────────────────── lifecycle ─────────────────────────────
    def on_closing(self) -> None:
        self.filter_stop.set()
        self.root.after(50, self.root.destroy)


def main() -> None:
    root = tk.Tk()
    CreatorApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
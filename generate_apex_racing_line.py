#!/usr/bin/env python3
"""Reconstructed offline apex-biased racing-line generator for the ICRA26 map.

This is a RECONSTRUCTION of the method used for the earlier apex CSV, not the
exact original experiment script. It independently optimizes waypoints from the
centerline and map, and will not necessarily reproduce identical coordinates.

Run from repository root:
    python3 generate_apex_racing_line.py

Inputs:
    maps/icra26_compete_centerline.csv
    maps/icra26_compete_handdrawn_safe.csv (only for the comparison plot)
    maps/icra26_compete.yaml and icra26_compete.pgm

Outputs (never replaces your stable apex CSV):
    maps/icra26_compete_racingline_apex_reconstructed.csv
    apex_reconstructed_comparison.png

The optimizer trades off (1) route length, so it cuts *toward* inside apexes,
(2) excessive curvature, so it does not make sharp kinks, and (3) a map
clearance penalty, keeping the car away from walls. It shifts centerline
waypoints sideways along their local normals and uses L-BFGS-B optimization.
This is not an optimal-lap-time or vehicle-dynamics optimizer. Test in simulator.
"""

import argparse
from pathlib import Path
import csv
import math

import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt
from scipy.optimize import minimize
import yaml


W_CURVE = 0.040      # integrated squared geometric curvature
W_HIGH_CURVE = 2.0  # penalize sharp peaks above SOFT_CURVE_LIMIT
SOFT_CURVE_LIMIT = 1.25  # [1/m] soft target, not a vehicle guarantee
W_SMOOTH = 0.12     # removes abrupt point-to-point heading changes
W_LENGTH = 0.285     # avoid overly long detours around bends
W_CENTER = 0.005    # prefer small shifts from the known centerline
W_START = 0.0       # preserve first/last few points near starting line
W_CLEARANCE = 6500.0  # soft barrier; ALSO perform strict final validation


def read_path(path):
    points = []
    with open(path, newline="", encoding="utf-8") as fp:
        for line in csv.reader(fp):
            if not line or not line[0].strip() or line[0].lstrip().startswith("#"):
                continue
            points.append((float(line[0]), float(line[1])))
    a = np.asarray(points, dtype=float)
    if a.ndim != 2 or a.shape[1] != 2 or len(a) < 4 or not np.isfinite(a).all():
        raise ValueError(f"Invalid x,y path: {path}")
    return a


def normals_on_loop(points):
    tan = np.roll(points, -1, axis=0) - np.roll(points, 1, axis=0)
    norms = np.linalg.norm(tan, axis=1)
    if np.any(norms < 1e-8):
        raise ValueError("Centerline has duplicate or degenerate neighboring points")
    tan /= norms[:, None]
    return np.stack((-tan[:, 1], tan[:, 0]), axis=1)


def curvature(points):
    """Signed 3-point curvature, suitable for comparing CSV geometry."""
    u = points - np.roll(points, 1, axis=0)
    v = np.roll(points, -1, axis=0) - points
    w = u + v
    denom = (np.linalg.norm(u, axis=1)
             * np.linalg.norm(v, axis=1)
             * np.linalg.norm(w, axis=1))
    if np.any(denom < 1e-10):
        raise ValueError("Path contains near-zero-length segments")
    return 2.0 * (u[:, 0] * v[:, 1] - u[:, 1] * v[:, 0]) / denom


def curvature_energy_and_gradient(points):
    """Smooth curvature energy + exact gradient w.r.t. x,y of every point."""
    u = points - np.roll(points, 1, axis=0)
    v = np.roll(points, -1, axis=0) - points
    w = u + v
    nu2 = np.sum(u * u, axis=1)
    nv2 = np.sum(v * v, axis=1)
    nw2 = np.sum(w * w, axis=1)
    denom = np.sqrt(nu2 * nv2 * nw2).clip(1e-10)
    cross = u[:, 0] * v[:, 1] - u[:, 1] * v[:, 0]
    k = 2.0 * cross / denom
    high = np.maximum(np.abs(k) - SOFT_CURVE_LIMIT, 0.0)
    cost = W_CURVE * np.sum(k * k) + W_HIGH_CURVE * np.sum(high * high)
    dk = 2 * W_CURVE * k + 2 * W_HIGH_CURVE * high * np.sign(k)
    dku = (2.0 / denom)[:, None] * (
        np.stack((v[:, 1], -v[:, 0]), axis=1)
        - cross[:, None] * (u / nu2[:, None] + w / nw2[:, None]))
    dkv = (2.0 / denom)[:, None] * (
        np.stack((-u[:, 1], u[:, 0]), axis=1)
        - cross[:, None] * (v / nv2[:, None] + w / nw2[:, None]))
    gu = dk[:, None] * dku
    gv = dk[:, None] * dkv
    # Each point participates in the preceding, current, and following turns.
    grad = gu - gv - np.roll(gu, -1, axis=0) + np.roll(gv, 1, axis=0)
    return float(cost), grad


class TrackMap:
    def __init__(self, yaml_file):
        with open(yaml_file, encoding="utf-8") as fp:
            meta = yaml.safe_load(fp)
        self.resolution = float(meta["resolution"])
        self.origin = np.asarray(meta["origin"][:2], dtype=float)
        self.image_path = yaml_file.parent / meta["image"]
        self.image = np.asarray(Image.open(self.image_path).convert("L"))
        # This project uses pixels: 254=driveable, 0=wall, 205=UNKNOWN.
        # Treat UNKNOWN as NOT driveable, never as free.
        self.free = self.image >= 250
        self.height, self.width = self.image.shape
        if not np.any(self.free):
            raise ValueError("Map has no driveable pixels")
        self.distance = distance_transform_edt(self.free) * self.resolution

    def sample_clearance(self, points, gradients=False):
        """Bilinear grid distance, with exact bilinear derivatives in world axes."""
        p = np.asarray(points, dtype=float)
        rf = self.height - 1 - (p[:, 1] - self.origin[1]) / self.resolution
        cf = (p[:, 0] - self.origin[0]) / self.resolution
        inside = (rf >= 0) & (cf >= 0) & (rf < self.height - 1) & (cf < self.width - 1)
        r0 = np.clip(np.floor(rf).astype(int), 0, self.height - 2)
        c0 = np.clip(np.floor(cf).astype(int), 0, self.width - 2)
        ar = np.clip(rf - r0, 0, 1)
        ac = np.clip(cf - c0, 0, 1)
        d00 = self.distance[r0, c0]
        d01 = self.distance[r0, c0 + 1]
        d10 = self.distance[r0 + 1, c0]
        d11 = self.distance[r0 + 1, c0 + 1]
        upper = (1 - ac) * d00 + ac * d01
        lower = (1 - ac) * d10 + ac * d11
        d = np.where(inside, (1 - ar) * upper + ar * lower, 0.0)
        if not gradients:
            return d
        # grid row increases downward; world y increases upward
        gx = ((1 - ar) * (d01 - d00) + ar * (d11 - d10)) / self.resolution
        gy = -(lower - upper) / self.resolution
        g = np.stack((np.where(inside, gx, 0), np.where(inside, gy, 0)), axis=1)
        return d, g

    def check_dense_path(self, points, spacing=0.025):
        """Sample every segment densely, including interpolation between CSV rows."""
        nxt = np.roll(points, -1, axis=0)
        distances = np.linalg.norm(nxt - points, axis=1)
        all_samples = []
        for i, seg in enumerate(distances):
            fractions = np.arange(max(2, int(np.ceil(seg / spacing)) + 1))
            fractions = (fractions / (len(fractions) - 1))[:, None]
            all_samples.append(points[i] * (1 - fractions) + nxt[i] * fractions)
        dense = np.concatenate(all_samples, axis=0)
        return float(np.min(self.sample_clearance(dense))), dense

    @property
    def extent(self):
        x, y = self.origin
        return (x, x + self.width * self.resolution,
                y, y + self.height * self.resolution)


def optimize(center, track, min_clearance, max_offset, maxiter):
    n = len(center)
    normal = normals_on_loop(center)
    indices = np.unique(np.array([0, 1, 2, 3, 4, n - 1, n - 2, n - 3]))

    def loss_and_gradient(offset):
        pts = center + offset[:, None] * normal
        cost, grad_xy = curvature_energy_and_gradient(pts)
        d2 = np.roll(pts, 1, axis=0) - 2 * pts + np.roll(pts, -1, axis=0)
        cost += W_SMOOTH * np.sum(d2 * d2)
        grad_xy += 2 * W_SMOOTH * (
            np.roll(d2, 1, axis=0) - 2 * d2 + np.roll(d2, -1, axis=0))
        d1 = pts - np.roll(pts, -1, axis=0)
        cost += W_LENGTH * np.sum(d1 * d1)
        grad_xy += 2 * W_LENGTH * (
            2 * pts - np.roll(pts, -1, axis=0) - np.roll(pts, 1, axis=0))
        cost += W_CENTER * np.dot(offset, offset)
        grad_offset = 2 * W_CENTER * offset
        cost += W_START * np.dot(offset[indices], offset[indices])
        grad_offset[indices] += 2 * W_START * offset[indices]
        for t in (0.0, 0.5):
            sample = (1 - t) * pts + t * np.roll(pts, -1, axis=0)
            clearance, grad_d = track.sample_clearance(sample, gradients=True)
            under = np.minimum(clearance - min_clearance, 0.0)
            cost += W_CLEARANCE * np.dot(under, under)
            grad_samples = (2 * W_CLEARANCE * under)[:, None] * grad_d
            grad_xy += (1 - t) * grad_samples + t * np.roll(grad_samples, 1, axis=0)
        grad_offset += np.sum(grad_xy * normal, axis=1)
        return float(cost), grad_offset

    result = minimize(
        loss_and_gradient, np.zeros(n), jac=True, method="L-BFGS-B",
        bounds=[(-max_offset, max_offset)] * n,
        options={"maxiter": maxiter, "ftol": 1e-10, "gtol": 1e-6},
    )
    return center + result.x[:, None] * normal, result


def route_stats(points, track):
    curvature_abs = np.abs(curvature(points))
    minimum, _ = track.check_dense_path(points)
    length = np.linalg.norm(points - np.roll(points, -1, axis=0), axis=1).sum()
    return dict(length=float(length), max_curvature=float(np.max(curvature_abs)),
                p95_curvature=float(np.quantile(curvature_abs, .95)),
                minimum_clearance=minimum)


def make_plot(center, hand, candidate, track, save_path, prior=None):
    fig = plt.figure(figsize=(14, 10.5))
    gs = fig.add_gridspec(2, 2, width_ratios=[0.87, 1.13], hspace=0.21, wspace=0.25)
    axes = [fig.add_subplot(gs[:, 0]), fig.add_subplot(gs[0, 1]), fig.add_subplot(gs[1, 1])]
    overlays = [
        (center, "#2563eb", "Centerline", "-", 1.5),
        (hand, "#f59e0b", "Hand-drawn", "--", 1.4),
    ]
    if prior is not None:
        overlays.append((prior, "#16a34a", "Conservative optimized", "-", 1.6))
    overlays.append((candidate, "#dc2626", "Apex-focused reconstructed", "-", 2.1))
    for ax in axes:
        ax.imshow(track.image, cmap="gray", origin="upper", extent=track.extent,
                  interpolation="nearest", alpha=0.82)
        for route, color, label, style, width in overlays:
            closed = np.vstack((route, route[0]))
            ax.plot(closed[:, 0], closed[:, 1], color=color,
                    linestyle=style, lw=width, label=label)
        ax.set_aspect("equal")
        ax.grid(alpha=.18)
        ax.set_xlabel("World X (m)")
        ax.set_ylabel("World Y (m)")
    axes[0].set_title("Full ICRA 2026 circuit")
    axes[0].legend(loc="lower right", fontsize=8)
    axes[1].set_title("Tight inner U-turn")
    axes[1].set_xlim(-6.1, -1.4)
    axes[1].set_ylim(-1.8, 4.9)
    axes[2].set_title("Bottom hairpin")
    axes[2].set_xlim(-4.5, 2.0)
    axes[2].set_ylim(-12.8, -5.4)
    labels = [(name, route_stats(p, track)) for p, _, name, _, _ in overlays]
    line = lambda name, st: (f"{name}: {st['length']:.2f} m, "
                             f"max curvature {st['max_curvature']:.2f} 1/m, "
                             f"mapped center clearance {st['minimum_clearance']:.2f} m")
    fig.suptitle("Apex-biased racing-line reconstruction — geometry only", fontsize=13)
    fig.text(.5, .014, "\n".join(line(name, st) for name, st in labels),
             ha="center", va="bottom", fontsize=8.5,
             bbox={"boxstyle": "round,pad=.4", "facecolor": "#f7f7f7", "edgecolor": "#dddddd"})
    fig.subplots_adjust(top=.94, bottom=.15)
    fig.savefig(save_path, dpi=160, bbox_inches="tight")
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--maps-dir", type=Path, default=Path("maps"))
    parser.add_argument("--clearance", type=float, default=.44,
                        help="requested map clearance of path center in metres (default 0.44)")
    parser.add_argument("--max-offset", type=float, default=.55,
                        help="maximum lateral displacement from centerline (m)")
    parser.add_argument("--length-weight", type=float, default=0.285,
                        help="higher favors shorter routes, potentially tighter apexes")
    parser.add_argument("--maxiter", type=int, default=4500)
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--plot", type=Path, default=Path("apex_reconstructed_comparison.png"))
    args = parser.parse_args()
    global W_LENGTH
    if not (0.02 <= args.length_weight <= 1.0):
        parser.error("--length-weight must be in [0.02, 1.0]")
    W_LENGTH = args.length_weight
    maps_dir = args.maps_dir.expanduser().resolve()
    track = TrackMap(maps_dir / "icra26_compete.yaml")
    center = read_path(maps_dir / "icra26_compete_centerline.csv")
    hand = read_path(maps_dir / "icra26_compete_handdrawn_safe.csv")
    if not (0.3 <= args.clearance <= 0.9):
        parser.error("--clearance must be within [0.3,0.9] m")
    if not (0.05 <= args.max_offset <= 0.75):
        parser.error("--max-offset must be within [0.05,0.75] m")
    output = args.output or maps_dir / "icra26_compete_racingline_apex_reconstructed.csv"
    print(f"Optimizing {len(center)} waypoints; clearance={args.clearance:.2f} m; "
          f"max offset={args.max_offset:.2f} m; length weight={W_LENGTH:.3f}")
    candidate, result = optimize(center, track, args.clearance,
                                 args.max_offset, args.maxiter)
    stats = route_stats(candidate, track)
    if stats["minimum_clearance"] < args.clearance - 0.008:
        raise RuntimeError("Candidate failed DENSE map clearance check "
                           f"({stats['minimum_clearance']:.3f}m < "
                           f"{args.clearance:.3f}m). No path exported.")
    if stats["minimum_clearance"] < 0.40:
        raise RuntimeError("Path gets too close to mapped boundary (<0.40 m)")
    if not np.all(np.isfinite(candidate)):
        raise RuntimeError("Non-finite candidate, refusing to export")
    output.parent.mkdir(parents=True, exist_ok=True)
    with open(output, "w", newline="", encoding="utf-8") as fp:
        fp.write("# Reconstructed apex-focused optimization — not simulator validated\n")
        fp.write(f"# Requested path-center clearance: {args.clearance:.3f} m\n")
        fp.write("# x_m,y_m\n")
        writer = csv.writer(fp)
        writer.writerows((f"{x:.6f}", f"{y:.6f}") for x, y in candidate)
    prior_path = maps_dir / "icra26_compete_racingline_candidate.csv"
    prior = read_path(prior_path) if prior_path.exists() else None
    make_plot(center, hand, candidate, track, args.plot, prior=prior)
    print("\nRoute                 Length(m)   Max |k|(1/m)   Min clearance(m)")
    for name, points in [("Centerline", center), ("Hand-drawn", hand), ("Candidate", candidate)]:
        s = route_stats(points, track)
        print(f"{name:<20} {s['length']:8.2f} {s['max_curvature']:13.3f} "
              f"{s['minimum_clearance']:18.3f}")
    print(f"\nOptimizer: {result.message} (iterations={result.nit})")
    if not result.success:
        print("Warning: optimizer stopped without converging. Treat as experimental.")
    reference_csv = maps_dir / "icra26_compete_racingline_apex.csv"
    if reference_csv.exists():
        reference = read_path(reference_csv)
        if reference.shape == candidate.shape:
            rms = float(np.sqrt(np.mean(np.sum((candidate - reference)**2, axis=1))))
            print(f"RMS waypoint difference vs earlier apex CSV: {rms:.3f} m")
            print("The earlier apex CSV is intentionally NOT overwritten.")
    print(f"New CSV: {output.resolve()}")
    print(f"Plot: {args.plot.resolve()}")
    print("WARNING: Geometry is NOT proof of drivable, fast, or collision-free behavior.")
    print("Test at 0.5 m/s first, then increase gradually; keep originals unchanged.")


if __name__ == "__main__":
    main()

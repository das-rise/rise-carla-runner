"""
renderer.py — Draws parsed OpenDRIVE data with a dark map aesthetic.

Lane types are color-coded; junction roads are tinted brighter with
dashed orange reference lines.
"""

import matplotlib.lines as mlines
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import LineCollection, PolyCollection

# ── Color palette ─────────────────────────────────────────────────────────────

# Base (dark) colors for normal roads
_LANE_COLORS = {
    "driving": "#3a3a3a",
    "shoulder": "#4a3a22",
    "sidewalk": "#1e3a5a",
    "border": "#383838",
    "restricted": "#5a1a1a",
    "parking": "#2e1e50",
    "median": "#1a3a1a",
    "biking": "#0e3838",
    "none": "#1a1a1a",
    "curb": "#3c3010",
    "connectingRamp": "#3a2a0a",
    "mwyEntry": "#1e3010",
    "mwyExit": "#3a1a0a",
    "entry": "#1a3a1a",
    "exit": "#3a1a1a",
    "rail": "#282e38",
    "tram": "#282e38",
    "roadWorks": "#3a1e00",
    "bidirectional": "#162040",
    "special1": "#2e0e28",
    "special2": "#220820",
    "special3": "#180218",
}
_DEFAULT_LANE_COLOR = "#2a2a2a"

# How much brighter junction roads appear (0.0 = no change, 1.0 = white)
_JUNCTION_TINT = 0.35

# Reference line colors
_REF_COLOR = "#FFD700"  # gold  — normal roads
_REF_JNC_COLOR = "#FF8C00"  # orange — junction roads

# Boundary line color
_BOUNDARY_COLOR = "#606060"

# Background
_BG_COLOR = "#0d0d0d"


def _tint(hex_color: str, amount: float) -> str:
    """Blend hex_color towards white by `amount` (0–1)."""
    r = int(hex_color[1:3], 16)
    g = int(hex_color[3:5], 16)
    b = int(hex_color[5:7], 16)
    r = min(255, int(r + (255 - r) * amount))
    g = min(255, int(g + (255 - g) * amount))
    b = min(255, int(b + (255 - b) * amount))
    return f"#{r:02x}{g:02x}{b:02x}"


def _lane_color(lane_type: str, is_junction: bool) -> str:
    base = _LANE_COLORS.get(lane_type, _DEFAULT_LANE_COLOR)
    return _tint(base, _JUNCTION_TINT) if is_junction else base


# ── Main render function ──────────────────────────────────────────────────────


def render(
    all_polys,
    all_boundaries,
    all_refs,
    title: str = "OpenDRIVE Map",
    ep_issues=None,
    lane_issues=None,
    route=None,
    routes=None,
):
    """
    Parameters
    ----------
    all_polys      : list of {'verts': Nx2, 'type': str, 'is_junction': bool}
    all_boundaries : list of {'xs': array, 'ys': array, 'is_junction': bool}
    all_refs       : list of {'xs': array, 'ys': array, 'is_junction': bool}
    title          : window / plot title
    """
    # ── Map OpenDRIVE to CARLA Visual Orientation ─────────────────────────────
    # OpenDRIVE: X East, Y North
    # CARLA Top-Down View: X Forward/Up, Y Right
    # Transformation: Plot X = -Y_odr, Plot Y = X_odr

    transformed_polys = []
    for poly in all_polys:
        v = poly["verts"]
        transformed_polys.append(
            {
                "verts": np.column_stack([-v[:, 1], v[:, 0]]),
                "type": poly["type"],
                "is_junction": poly["is_junction"],
            }
        )
    all_polys = transformed_polys

    transformed_boundaries = []
    for b in all_boundaries:
        transformed_boundaries.append(
            {"xs": -b["ys"], "ys": b["xs"], "is_junction": b["is_junction"]}
        )
    all_boundaries = transformed_boundaries

    transformed_refs = []
    for ref in all_refs:
        transformed_refs.append(
            {"xs": -ref["ys"], "ys": ref["xs"], "is_junction": ref["is_junction"]}
        )
    all_refs = transformed_refs

    if ep_issues:
        for i in ep_issues:
            orig_x, orig_y = i.x, i.y
            i.x, i.y = -orig_y, orig_x

    if lane_issues:
        for i in lane_issues:
            orig_x, orig_y = i.x, i.y
            i.x, i.y = -orig_y, orig_x

    fig, ax = plt.subplots(figsize=(14, 10))
    fig.patch.set_facecolor(_BG_COLOR)
    ax.set_facecolor(_BG_COLOR)
    ax.set_aspect("equal", adjustable="datalim")
    ax.axis("off")
    ax.set_title(title, color="#c8c8c8", fontsize=11, pad=8, fontfamily="monospace")

    # ── 1. Filled lane polygons (grouped by color for performance) ────────────
    from collections import defaultdict

    poly_groups = defaultdict(list)
    seen_types = {}  # lane_type → color (for legend)

    for poly in all_polys:
        color = _lane_color(poly["type"], poly["is_junction"])
        poly_groups[color].append(poly["verts"])
        if poly["type"] not in seen_types:
            seen_types[poly["type"]] = _lane_color(poly["type"], False)

    for color, verts_list in poly_groups.items():
        col = PolyCollection(
            verts_list, facecolor=color, edgecolor="none", zorder=1, rasterized=True
        )
        ax.add_collection(col)

    # ── 2. Lane boundary lines ────────────────────────────────────────────────
    if all_boundaries:
        segs = [np.column_stack([b["xs"], b["ys"]]) for b in all_boundaries]
        lc = LineCollection(
            segs, colors=_BOUNDARY_COLOR, linewidths=0.3, zorder=2, rasterized=True
        )
        ax.add_collection(lc)

    # ── 3. Reference lines (normal vs junction) ───────────────────────────────
    normal_segs = []
    junc_segs = []
    for ref in all_refs:
        seg = np.column_stack([ref["xs"], ref["ys"]])
        if ref["is_junction"]:
            junc_segs.append(seg)
        else:
            normal_segs.append(seg)

    if normal_segs:
        lc = LineCollection(
            normal_segs, colors=_REF_COLOR, linewidths=0.8, zorder=3, rasterized=True
        )
        ax.add_collection(lc)
    if junc_segs:
        lc = LineCollection(
            junc_segs,
            colors=_REF_JNC_COLOR,
            linewidths=0.7,
            linestyles="dashed",
            zorder=3,
            rasterized=True,
        )
        ax.add_collection(lc)

    # ── 4. Legend ─────────────────────────────────────────────────────────────
    handles = []

    # Lane type patches (only types that actually appear)
    for ltype, color in sorted(seen_types.items()):
        handles.append(
            mpatches.Patch(
                facecolor=color, edgecolor="#888", linewidth=0.3, label=ltype
            )
        )

    # Reference line entries
    handles.append(
        mlines.Line2D([], [], color=_REF_COLOR, lw=1.2, label="reference line")
    )
    handles.append(
        mlines.Line2D(
            [],
            [],
            color=_REF_JNC_COLOR,
            lw=1.2,
            linestyle="dashed",
            label="junction ref.",
        )
    )

    legend = ax.legend(
        handles=handles,
        loc="upper right",
        framealpha=0.35,
        facecolor="#1a1a1a",
        edgecolor="#444",
        labelcolor="#cccccc",
        fontsize=7.5,
        title="Lane types",
        title_fontsize=8,
    )
    legend.get_title().set_color("#aaaaaa")

    ax.autoscale_view()

    # ── Optional connectivity overlay ─────────────────────────────────────────
    if ep_issues is not None or lane_issues is not None:
        from . import connectivity

        connectivity.render_overlay(ax, ep_issues or [], lane_issues or [])

    # ── Optional route overlay ───────────────────────────────────────────────
    if route is not None:
        xs = [pt[1] for pt in route]
        ys = [pt[0] for pt in route]
        ax.plot(xs, ys, color="cyan", linewidth=2.0, zorder=4, label="Route")
        ax.scatter(
            xs[-1],
            ys[-1],
            color="lime",
            edgecolor="white",
            s=80,
            marker="x",
            zorder=5,
            label="Route Start",
        )

    if routes is not None:
        for r in routes:
            if not r:
                continue
            xs = [-pt[1] for pt in r]
            ys = [pt[0] for pt in r]
            ax.plot(xs, ys, color="cyan", linewidth=1.5, zorder=4, alpha=0.7)
            ax.scatter(
                xs[-1],
                ys[-1],
                color="lime",
                edgecolor="white",
                s=40,
                marker="x",
                zorder=5,
            )

    plt.tight_layout(pad=0.5)

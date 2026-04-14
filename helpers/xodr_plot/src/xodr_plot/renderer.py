"""
renderer.py — Draws parsed OpenDRIVE data with a dark map aesthetic.

Lane types are color-coded; junction roads are tinted brighter with
dashed orange reference lines.
"""

import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.widgets as widgets
import matplotlib.patches as mpatches
import matplotlib.lines as mlines
from matplotlib.collections import PolyCollection, LineCollection


# ── Color palette ─────────────────────────────────────────────────────────────

# Base (dark) colors for normal roads
_LANE_COLORS = {
    'driving':        '#3a3a3a',
    'shoulder':       '#4a3a22',
    'sidewalk':       '#1e3a5a',
    'border':         '#383838',
    'restricted':     '#5a1a1a',
    'parking':        '#2e1e50',
    'median':         '#1a3a1a',
    'biking':         '#0e3838',
    'none':           '#1a1a1a',
    'curb':           '#3c3010',
    'connectingRamp': '#3a2a0a',
    'mwyEntry':       '#1e3010',
    'mwyExit':        '#3a1a0a',
    'entry':          '#1a3a1a',
    'exit':           '#3a1a1a',
    'rail':           '#282e38',
    'tram':           '#282e38',
    'roadWorks':      '#3a1e00',
    'bidirectional':  '#162040',
    'special1':       '#2e0e28',
    'special2':       '#220820',
    'special3':       '#180218',
}
_DEFAULT_LANE_COLOR = '#2a2a2a'

# How much brighter junction roads appear (0.0 = no change, 1.0 = white)
_JUNCTION_TINT = 0.35

# Reference line colors
_REF_COLOR      = '#FFD700'   # gold  — normal roads
_REF_JNC_COLOR  = '#FF8C00'   # orange — junction roads

# Boundary line color
_BOUNDARY_COLOR = '#606060'

# Background
_BG_COLOR = '#0d0d0d'


def _tint(hex_color: str, amount: float) -> str:
    """Blend hex_color towards white by `amount` (0–1)."""
    r = int(hex_color[1:3], 16)
    g = int(hex_color[3:5], 16)
    b = int(hex_color[5:7], 16)
    r = min(255, int(r + (255 - r) * amount))
    g = min(255, int(g + (255 - g) * amount))
    b = min(255, int(b + (255 - b) * amount))
    return f'#{r:02x}{g:02x}{b:02x}'


def _lane_color(lane_type: str, is_junction: bool) -> str:
    base = _LANE_COLORS.get(lane_type, _DEFAULT_LANE_COLOR)
    return _tint(base, _JUNCTION_TINT) if is_junction else base


# ── Interactive Components ────────────────────────────────────────────────────

class InteractiveSelector:
    def __init__(self, fig, ax):
        self.fig = fig
        self.ax = ax
        self.cid = self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        self.cid_close = self.fig.canvas.mpl_connect('close_event', self.on_main_closed)
        self.current_point = None
        self.arrow = None
        self.popup_fig = None
        self.slider = None
        self.button = None
        self.x_carla = 0.0
        self.y_carla = 0.0
        self.selections = []

    def on_click(self, event):
        if event.inaxes != self.ax or self.popup_fig is not None:
            return
        if event.button != 1:
            return
            
        # Plot X is Y_carla, Plot Y is X_carla to match CARLA spectator visually
        self.y_carla = event.xdata
        self.x_carla = event.ydata
        print(f"Location selected: x={self.x_carla:.3f}, y={self.y_carla:.3f}")
        
        self.current_point = (event.xdata, event.ydata)
        self.spawn_popup()
        
    def draw_arrow(self, angle_deg):
        if self.arrow is not None:
            self.arrow.remove()
            self.arrow = None
            
        # CARLA Yaw: 0 is +X (Up/Plot Y), 90 is +Y (Right/Plot X)
        plot_angle_deg = 90 - angle_deg
        rad = np.deg2rad(plot_angle_deg)
        dx_plot = np.cos(rad)
        dy_plot = np.sin(rad)
        
        x_lim = self.ax.get_xlim()
        length = abs(x_lim[1] - x_lim[0]) * 0.05
        if length <= 0:
            length = 5.0
            
        self.arrow = self.ax.annotate(
            '',
            xy=(self.current_point[0] + dx_plot * length, self.current_point[1] + dy_plot * length),
            xytext=self.current_point,
            arrowprops=dict(arrowstyle="->", color="red", lw=2)
        )
        self.fig.canvas.draw_idle()

    def spawn_popup(self):
        self.popup_fig = plt.figure(figsize=(4, 2))
        self.popup_fig.canvas.manager.set_window_title('Set Angle')
        
        ax_slider = self.popup_fig.add_axes([0.15, 0.5, 0.7, 0.15])
        self.slider = widgets.Slider(ax_slider, 'Angle', -180, 180, valinit=0, valstep=1)
        
        ax_button = self.popup_fig.add_axes([0.4, 0.1, 0.2, 0.25])
        self.button = widgets.Button(ax_button, 'OK')
        
        self.slider.on_changed(self.on_slider_changed)
        self.button.on_clicked(self.on_ok_clicked)
        self.popup_fig.canvas.mpl_connect('close_event', self.on_popup_closed)
        
        self.draw_arrow(0)
        self.popup_fig.show()

    def on_slider_changed(self, val):
        self.draw_arrow(val)
        
    def on_ok_clicked(self, event):
        angle = self.slider.val
        print(f"Angle selected: {angle:.3f}")
        self.selections.append((self.x_carla, self.y_carla, angle))
        self.arrow = None # leave arrow on plot
        plt.close(self.popup_fig)
        
    def on_popup_closed(self, event):
        self.popup_fig = None
        self.slider = None
        self.button = None
        if self.arrow is not None:
            self.arrow.remove()
            self.arrow = None
            self.fig.canvas.draw_idle()

    def on_main_closed(self, event):
        if len(self.selections) > 0:
            print('<?xml version="1.0" ?>')
            print('<route id="_" town="_">')
            for sel in self.selections:
                print(f'\t<waypoint pitch="0.0" roll="0.0" x="{sel[0]:.3f}" y="{sel[1]:.3f}" yaw="{sel[2]:.3f}" z="0.0"/>')
            print('</route>')


# ── Main render function ──────────────────────────────────────────────────────

def render(all_polys, all_boundaries, all_refs, title: str = 'OpenDRIVE Map',
           ep_issues=None, lane_issues=None, route=None, routes=None):
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
        v = poly['verts']
        transformed_polys.append({
            'verts': np.column_stack([-v[:, 1], v[:, 0]]),
            'type': poly['type'],
            'is_junction': poly['is_junction']
        })
    all_polys = transformed_polys

    transformed_boundaries = []
    for b in all_boundaries:
        transformed_boundaries.append({
            'xs': -b['ys'],
            'ys': b['xs'],
            'is_junction': b['is_junction']
        })
    all_boundaries = transformed_boundaries

    transformed_refs = []
    for ref in all_refs:
        transformed_refs.append({
            'xs': -ref['ys'],
            'ys': ref['xs'],
            'is_junction': ref['is_junction']
        })
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
    ax.set_aspect('equal', adjustable='datalim')
    ax.axis('off')
    ax.set_title(title, color='#c8c8c8', fontsize=11, pad=8,
                 fontfamily='monospace')

    # ── 1. Filled lane polygons (grouped by color for performance) ────────────
    from collections import defaultdict
    poly_groups = defaultdict(list)
    seen_types  = {}  # lane_type → color (for legend)

    for poly in all_polys:
        color = _lane_color(poly['type'], poly['is_junction'])
        poly_groups[color].append(poly['verts'])
        if poly['type'] not in seen_types:
            seen_types[poly['type']] = _lane_color(poly['type'], False)

    for color, verts_list in poly_groups.items():
        col = PolyCollection(verts_list, facecolor=color, edgecolor='none',
                             zorder=1, rasterized=True)
        ax.add_collection(col)

    # ── 2. Lane boundary lines ────────────────────────────────────────────────
    if all_boundaries:
        segs = [np.column_stack([b['xs'], b['ys']]) for b in all_boundaries]
        lc = LineCollection(segs, colors=_BOUNDARY_COLOR, linewidths=0.3,
                            zorder=2, rasterized=True)
        ax.add_collection(lc)

    # ── 3. Reference lines (normal vs junction) ───────────────────────────────
    normal_segs  = []
    junc_segs    = []
    for ref in all_refs:
        seg = np.column_stack([ref['xs'], ref['ys']])
        if ref['is_junction']:
            junc_segs.append(seg)
        else:
            normal_segs.append(seg)

    if normal_segs:
        lc = LineCollection(normal_segs, colors=_REF_COLOR, linewidths=0.8,
                            zorder=3, rasterized=True)
        ax.add_collection(lc)
    if junc_segs:
        lc = LineCollection(junc_segs, colors=_REF_JNC_COLOR, linewidths=0.7,
                            linestyles='dashed', zorder=3, rasterized=True)
        ax.add_collection(lc)

    # ── 4. Legend ─────────────────────────────────────────────────────────────
    handles = []

    # Lane type patches (only types that actually appear)
    for ltype, color in sorted(seen_types.items()):
        handles.append(
            mpatches.Patch(facecolor=color, edgecolor='#888', linewidth=0.3,
                           label=ltype)
        )

    # Reference line entries
    handles.append(mlines.Line2D([], [], color=_REF_COLOR,    lw=1.2,
                                 label='reference line'))
    handles.append(mlines.Line2D([], [], color=_REF_JNC_COLOR, lw=1.2,
                                 linestyle='dashed', label='junction ref.'))

    legend = ax.legend(
        handles=handles,
        loc='upper right',
        framealpha=0.35,
        facecolor='#1a1a1a',
        edgecolor='#444',
        labelcolor='#cccccc',
        fontsize=7.5,
        title='Lane types',
        title_fontsize=8,
    )
    legend.get_title().set_color('#aaaaaa')

    ax.autoscale_view()
    
    fig.selector = InteractiveSelector(fig, ax)

    # ── Optional connectivity overlay ─────────────────────────────────────────
    if ep_issues is not None or lane_issues is not None:
        from . import connectivity
        connectivity.render_overlay(ax, ep_issues or [], lane_issues or [])

    # ── Optional route overlay ───────────────────────────────────────────────
    if route is not None:
        xs = [pt[1] for pt in route]
        ys = [pt[0] for pt in route]
        ax.plot(xs, ys, color='cyan', linewidth=2.0, zorder=4, label='Route')
        ax.scatter(xs[-1], ys[-1], color='lime', edgecolor='white', s=80, marker="x", zorder=5, label='Route Start')

    if routes is not None:
        for r in routes:
            if not r: continue
            xs = [-pt[1] for pt in r]
            ys = [pt[0] for pt in r]
            ax.plot(xs, ys, color='cyan', linewidth=1.5, zorder=4, alpha=0.7)
            ax.scatter(xs[-1], ys[-1], color='lime', edgecolor='white', s=40, marker="x", zorder=5)

    plt.tight_layout(pad=0.5)
    plt.show()

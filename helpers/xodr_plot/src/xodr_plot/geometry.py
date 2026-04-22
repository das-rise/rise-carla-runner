"""
geometry.py — Samples OpenDRIVE geometry primitives into world-space coordinates
and computes filled lane polygons + boundary lines.
"""

import numpy as np
import math


# ── Geometry primitive samplers ───────────────────────────────────────────────

def _sample_line(x0, y0, hdg, length, n):
    t = np.linspace(0, length, n)
    return x0 + t * np.cos(hdg), y0 + t * np.sin(hdg), np.full(n, hdg)


def _sample_arc(x0, y0, hdg, length, k, n):
    if abs(k) < 1e-12:
        return _sample_line(x0, y0, hdg, length, n)
    r  = 1.0 / k
    t  = np.linspace(0, length, n)
    dh = t * k
    return (x0 + r * (np.sin(hdg + dh) - np.sin(hdg)),
            y0 - r * (np.cos(hdg + dh) - np.cos(hdg)),
            hdg + dh)


def _sample_spiral(x0, y0, hdg, length, ks, ke, n):
    """Clothoid / Euler spiral with exact heading and midpoint position integration."""
    L  = max(length, 1e-12)
    s  = np.linspace(0, length, n)
    # Exact heading: θ(s) = θ₀ + ks·s + (ke−ks)·s²/(2L)
    hs = hdg + ks * s + (ke - ks) * s ** 2 / (2.0 * L)
    # Position: numerically integrate using midpoint headings
    xs = np.empty(n)
    ys = np.empty(n)
    xs[0], ys[0] = x0, y0
    if n > 1:
        s_mid  = 0.5 * (s[:-1] + s[1:])
        h_mid  = hdg + ks * s_mid + (ke - ks) * s_mid ** 2 / (2.0 * L)
        ds     = np.diff(s)
        xs[1:] = x0 + np.cumsum(ds * np.cos(h_mid))
        ys[1:] = y0 + np.cumsum(ds * np.sin(h_mid))
    return xs, ys, hs


def _sample_poly3(x0, y0, hdg, length, a, b, c, d, n):
    u  = np.linspace(0, length, n)
    v  = a + b * u + c * u ** 2 + d * u ** 3
    dv = b + 2 * c * u + 3 * d * u ** 2
    ch, sh = np.cos(hdg), np.sin(hdg)
    return (x0 + u * ch - v * sh,
            y0 + u * sh + v * ch,
            hdg + np.arctan(dv))


def _sample_ppoly3(x0, y0, hdg, length,
                   aU, bU, cU, dU, aV, bV, cV, dV, p_range, n):
    pmax = 1.0 if p_range == 'normalized' else length
    p  = np.linspace(0, pmax, n)
    u  = aU + bU * p + cU * p ** 2 + dU * p ** 3
    v  = aV + bV * p + cV * p ** 2 + dV * p ** 3
    du = bU + 2 * cU * p + 3 * dU * p ** 2
    dv = bV + 2 * cV * p + 3 * dV * p ** 2
    ch, sh = np.cos(hdg), np.sin(hdg)
    return (x0 + u * ch - v * sh,
            y0 + u * sh + v * ch,
            hdg + np.arctan2(dv, du))


def _sample_geom(g, density):
    n = max(20, int(g.length * density) + 1)
    x0, y0, h, l, p = g.x, g.y, g.hdg, g.length, g.params
    if   g.type == 'line':       return _sample_line(x0, y0, h, l, n)
    elif g.type == 'arc':        return _sample_arc(x0, y0, h, l, p['curvature'], n)
    elif g.type == 'spiral':     return _sample_spiral(x0, y0, h, l, p['curvStart'], p['curvEnd'], n)
    elif g.type == 'poly3':      return _sample_poly3(x0, y0, h, l, p['a'], p['b'], p['c'], p['d'], n)
    elif g.type == 'paramPoly3':
        return _sample_ppoly3(x0, y0, h, l,
                              p['aU'], p['bU'], p['cU'], p['dU'],
                              p['aV'], p['bV'], p['cV'], p['dV'],
                              p.get('pRange', 'normalized'), n)
    else:
        return _sample_line(x0, y0, h, l, n)


# ── Reference line ────────────────────────────────────────────────────────────

def build_reference_line(road, density=2):
    """Return (xs, ys, headings, s_values) for the full road reference line.

    The exact endpoint of every geometry primitive is always included as the
    final sample so that lane polygons extend flush to the road boundary with
    no floating-point gap.
    """
    parts = []
    s_offset = 0.0
    for g in road.geometries:
        xs, ys, hs = _sample_geom(g, density)
        s_local = np.linspace(0, g.length, len(xs))

        # Always include the exact declared endpoint of this primitive.
        # Re-sample just the last point at t=g.length to avoid linspace drift.
        ex, ey, eh = [arr[-1:] for arr in _sample_geom_at(g, g.length)]
        if abs(s_local[-1] - g.length) > 1e-9:
            xs = np.append(xs, ex)
            ys = np.append(ys, ey)
            hs = np.append(hs, eh)
            s_local = np.append(s_local, g.length)

        parts.append((xs, ys, hs, s_offset + s_local))
        s_offset += g.length

    if not parts:
        empty = np.array([])
        return empty, empty, empty, empty
    return tuple(np.concatenate([p[i] for p in parts]) for i in range(4))


def _sample_geom_at(g, t):
    """Sample a single point on geometry primitive g at arc-length t."""
    x0, y0, h, p = g.x, g.y, g.hdg, g.params
    if g.type == 'line':
        return (np.array([x0 + t * np.cos(h)]),
                np.array([y0 + t * np.sin(h)]),
                np.array([h]))
    elif g.type == 'arc':
        xs, ys, hs = _sample_arc(x0, y0, h, g.length, p['curvature'], 2)
        return xs[-1:], ys[-1:], hs[-1:]
    elif g.type == 'spiral':
        xs, ys, hs = _sample_spiral(x0, y0, h, g.length, p['curvStart'], p['curvEnd'], 2)
        return xs[-1:], ys[-1:], hs[-1:]
    elif g.type == 'poly3':
        xs, ys, hs = _sample_poly3(x0, y0, h, g.length, p['a'], p['b'], p['c'], p['d'], 2)
        return xs[-1:], ys[-1:], hs[-1:]
    elif g.type == 'paramPoly3':
        xs, ys, hs = _sample_ppoly3(x0, y0, h, g.length,
                                    p['aU'], p['bU'], p['cU'], p['dU'],
                                    p['aV'], p['bV'], p['cV'], p['dV'],
                                    p.get('pRange', 'normalized'), 2)
        return xs[-1:], ys[-1:], hs[-1:]
    return (np.array([x0]), np.array([y0]), np.array([h]))


# ── Lane width helper ─────────────────────────────────────────────────────────

def _offset_at(road, s_in_road: float) -> float:
    """Evaluate lane offset polynomial at s_in_road."""
    if not hasattr(road, 'lane_offsets') or not road.lane_offsets:
        return 0.0
    active = road.lane_offsets[0]
    for lo in road.lane_offsets:
        if lo.s <= s_in_road:
            active = lo
        else:
            break
    ds = s_in_road - active.s
    return active.a + active.b*ds + active.c*(ds**2) + active.d*(ds**3)

def _width_at(lane, s_in_section):
    """Evaluate lane width polynomial at s_in_section (distance from section start)."""
    best = None
    for w in lane.widths:
        if w.s_offset <= s_in_section + 1e-9:
            best = w
    if best is None:
        return 0.0
    ds = s_in_section - best.s_offset
    return max(0.0, best.a + best.b * ds + best.c * ds ** 2 + best.d * ds ** 3)


def road_width_at_endpoint(road, endpoint: str) -> float:
    """Return the total road width (sum of all lane widths) at a road endpoint.

    endpoint: 'start' (s=0, first laneSection) or 'end' (s=road.length, last laneSection).

    This is used to compute Carla's geometric tolerance (lane_width * 0.7) at
    each connection point.
    """
    if not road.lane_sections:
        return 0.0

    sections = sorted(road.lane_sections, key=lambda ls: ls.s)

    if endpoint == 'start':
        ls = sections[0]
        s_in_section = 0.0
    else:
        ls = sections[-1]
        # s relative to the start of the last section, evaluated at road end
        s_in_section = road.length - ls.s

    total = 0.0
    for lane in ls.left + ls.right:
        if lane.id == 0:
            continue
        total += _width_at(lane, max(0.0, s_in_section))
    return total


# ── Main build function ───────────────────────────────────────────────────────

def build_road_data(road, density=2):
    """
    Returns:
        polys      — list of {'verts': Nx2 array, 'type': str, 'is_junction': bool}
        boundaries — list of {'xs': array, 'ys': array, 'is_junction': bool}
        ref_xs, ref_ys — reference line world coordinates
    """
    ref_xs, ref_ys, ref_hs, ref_s = build_reference_line(road, density)
    if len(ref_xs) == 0:
        return [], [], ref_xs, ref_ys

    is_junc  = road.is_junction_road
    sections = sorted(road.lane_sections, key=lambda ls: ls.s)
    polys      = []
    boundaries = []

    for i, ls in enumerate(sections):
        s0 = ls.s
        s1 = sections[i + 1].s if i + 1 < len(sections) else road.length

        # Slice reference line points that fall in this section's s range
        mask = (ref_s >= s0 - 1e-6) & (ref_s <= s1 + 1e-6)
        if not mask.any():
            continue

        xs  = ref_xs[mask]
        ys  = ref_ys[mask]
        hs  = ref_hs[mask]
        ss  = ref_s[mask] - s0          # s relative to section start

        # Left-pointing normal vector
        nx = -np.sin(hs)
        ny =  np.cos(hs)

        # side_sign=+1 → left lanes (offset in normal direction)
        # side_sign=−1 → right lanes (offset opposite to normal)
        for side_sign, lanes in (
            (+1, sorted(ls.left,  key=lambda l: l.id)),
            (-1, sorted(ls.right, key=lambda l: abs(l.id))),
        ):
            if not lanes:
                continue

            prev_bx = xs.copy()
            prev_by = ys.copy()
            cum = np.zeros(len(xs))

            for lane in lanes:
                w = np.array([_width_at(lane, s) for s in ss])
                cum += w

                cur_bx = xs + side_sign * cum * nx
                cur_by = ys + side_sign * cum * ny

                # Filled polygon between previous and current boundary
                verts = np.column_stack([
                    np.concatenate([prev_bx, cur_bx[::-1]]),
                    np.concatenate([prev_by, cur_by[::-1]]),
                ])
                polys.append({'verts': verts, 'type': lane.type, 'is_junction': is_junc})

                # Outer boundary line for this lane
                boundaries.append({'xs': cur_bx, 'ys': cur_by, 'is_junction': is_junc})

                prev_bx = cur_bx.copy()
                prev_by = cur_by.copy()

    return polys, boundaries, ref_xs, ref_ys

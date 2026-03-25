"""
fixer.py -- Fixes geometry gaps in an OpenDRIVE file.

For every road-to-road link with any geometric gap, the road whose contact
point is 'start' has its first <geometry> element's x, y, and hdg snapped to
the computed endpoint of the linked road.  This directly closes the gap
without touching topology, lane widths, or any other attributes.

End-to-end connections (both contact points are 'end') cannot be fixed this
way and are reported as skipped.
"""

import math
import os
import xml.etree.ElementTree as ET
from typing import Dict, List, Set, Tuple, Optional

import numpy as np

from . import geometry as geom_mod
from .geometry import road_width_at_endpoint


# -- Helpers ------------------------------------------------------------------

def _road_endpoints(road) -> Tuple[Tuple[float, float, float],
                                   Tuple[float, float, float]]:
    """Return (start, end) as (x, y, hdg) triples."""
    rx, ry, rh, _ = geom_mod.build_reference_line(road)
    if len(rx) == 0:
        g = road.geometries[0]
        p = (g.x, g.y, g.hdg)
        return p, p
    return (float(rx[0]),  float(ry[0]),  float(rh[0])),  \
           (float(rx[-1]), float(ry[-1]), float(rh[-1]))


def _gap(a_xyz, b_xyz) -> float:
    return math.hypot(a_xyz[0] - b_xyz[0], a_xyz[1] - b_xyz[1])


# -- Core fix computation -----------------------------------------------------

def compute_fixes(roads) -> Tuple[Dict[str, Tuple[float, float, float]],
                                  List[str]]:
    """
    Walk every road-to-road link and decide what to snap.

    Crucially: we do NOT trust the declared contactPoint — instead we snap to
    whichever endpoint of the target road is geometrically nearest. Wrong
    contactPoint values in the xodr are a common exporter bug and would
    otherwise cause the fixer to move roads to completely wrong positions.

    Returns
    -------
    fixes   : dict  road_id -> (new_x, new_y, new_hdg)
    skipped : list of human-readable strings for pairs we cannot fix
    wrong_contact : list of warnings where declared contactPoint disagreed
                    with the geometrically nearest end
    """
    road_map = {r.id: r for r in roads}

    ep_cache: Dict[str, Tuple] = {}
    for r in roads:
        ep_cache[r.id] = _road_endpoints(r)

    fixes:         Dict[str, Tuple[float, float, float]] = {}
    skipped:       List[str] = []
    wrong_contact: List[str] = []
    seen:          Set[frozenset] = set()

    for road in roads:
        if not road.link:
            continue

        start_xyz, end_xyz = ep_cache[road.id]

        for my_end, my_xyz, link_ep in (
            ('start', start_xyz, road.link.predecessor),
            ('end',   end_xyz,   road.link.successor),
        ):
            if link_ep is None:
                continue
            if link_ep.element_type != 'road':
                continue

            tid = link_ep.element_id
            if tid not in road_map:
                continue

            pair_key = frozenset([(road.id, my_end), (tid,)])
            if pair_key in seen:
                continue
            seen.add(pair_key)

            t_start, t_end = ep_cache[tid]

            # Ignore declared contactPoint — find the geometrically nearest end
            dist_to_start = _gap(my_xyz, t_start)
            dist_to_end   = _gap(my_xyz, t_end)

            if dist_to_start <= dist_to_end:
                nearest_contact = 'start'
                target_xyz      = t_start
                gap             = dist_to_start
            else:
                nearest_contact = 'end'
                target_xyz      = t_end
                gap             = dist_to_end

            declared_contact = link_ep.contact_point or 'start'
            if declared_contact != nearest_contact:
                wrong_contact.append(
                    f'Road {road.id} {my_end} -> road {tid}: '
                    f'declared contactPoint="{declared_contact}" but nearest '
                    f'end is "{nearest_contact}" '
                    f'(dist to declared={dist_to_start if declared_contact=="start" else dist_to_end:.3f} m, '
                    f'dist to nearest={gap:.3f} m)'
                )

            if gap == 0.0:
                continue

            # Snap whichever side has a 'start' (x,y,hdg are directly editable)
            if my_end == 'start':
                fixes[road.id] = target_xyz
            elif nearest_contact == 'start':
                fixes[tid] = my_xyz
            else:
                skipped.append(
                    f'Road {road.id} end <-> road {tid} end: '
                    f'gap {gap:.4f} m — both ends, skipped'
                )

    return fixes, skipped, wrong_contact


# -- XML patch ----------------------------------------------------------------

def _strip_ns(tag: str) -> str:
    return tag.split('}', 1)[1] if '}' in tag else tag


def apply_fixes_and_write(filepath: str,
                          fixes: Dict[str, Tuple[float, float, float]],
                          skipped: List[str]) -> str:
    """
    Apply geometry fixes to the XML and write a new file.

    Parameters
    ----------
    filepath : path to the original .xodr file
    fixes    : road_id -> (new_x, new_y, new_hdg)  from compute_fixes()
    skipped  : list of skip messages (just printed)

    Returns
    -------
    out_path : path of the written file
    """
    if skipped:
        print(f"  Note: {len(skipped)} end-to-end gap(s) could not be fixed automatically:")
        for s in skipped:
            print(f"    {s}")

    # Build output path
    base, ext = os.path.splitext(filepath)
    out_path  = base + '_fixed_geometry' + ext

    # Parse preserving original structure (namespaces included)
    ET.register_namespace('', 'http://www.opendrive.org')
    tree = ET.parse(filepath)
    root = tree.getroot()

    fixed_count = 0

    for road_elem in root.iter():
        if _strip_ns(road_elem.tag) != 'road':
            continue
        rid = road_elem.get('id')
        if rid not in fixes:
            continue

        new_x, new_y, new_hdg = fixes[rid]

        # Find the first <geometry> inside <planView>
        plan_view = None
        for child in road_elem:
            if _strip_ns(child.tag) == 'planView':
                plan_view = child
                break
        if plan_view is None:
            continue

        first_geom = None
        for child in plan_view:
            if _strip_ns(child.tag) == 'geometry':
                first_geom = child
                break
        if first_geom is None:
            continue

        old_x   = float(first_geom.get('x',   0))
        old_y   = float(first_geom.get('y',   0))
        old_hdg = float(first_geom.get('hdg', 0))
        gap     = math.hypot(new_x - old_x, new_y - old_y)

        first_geom.set('x',   f'{new_x:.10f}')
        first_geom.set('y',   f'{new_y:.10f}')
        # Preserve original heading — offset is tiny so direction is unchanged
        first_geom.set('hdg', f'{old_hdg:.10f}')

        print(f"  Fixed road {rid}: start moved {gap:.4f} m  "
              f"({old_x:.4f},{old_y:.4f}) -> ({new_x:.4f},{new_y:.4f})")
        fixed_count += 1

    tree.write(out_path, encoding='unicode', xml_declaration=True)
    print(f"\n  {fixed_count} road(s) fixed.")
    print(f"  Written to: {out_path}")
    return out_path


# -- Public entry point -------------------------------------------------------

def fix_geometry(filepath: str, roads) -> str:
    """Compute and apply all geometry fixes. Returns the output file path."""
    print("Computing geometry fixes …")
    fixes, skipped, wrong_contact = compute_fixes(roads)

    if wrong_contact:
        print(f"\n  WARNING: {len(wrong_contact)} link(s) had incorrect contactPoint declarations "
              f"(snapped to geometrically nearest end instead):")
        for w in wrong_contact:
            print(f"    {w}")
        print()

    if not fixes and not skipped:
        print("  No geometry gaps found — nothing to fix.")
        base, ext = os.path.splitext(filepath)
        out_path  = base + '_fixed_geometry' + ext
        import shutil
        shutil.copy2(filepath, out_path)
        print(f"  Copied unchanged file to: {out_path}")
        return out_path

    print(f"  {len(fixes)} road start(s) to snap …")
    return apply_fixes_and_write(filepath, fixes, skipped)

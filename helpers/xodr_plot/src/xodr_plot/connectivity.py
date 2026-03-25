"""
connectivity.py -- Diagnoses topological connectivity issues in an OpenDRIVE map.

Road-level checks (circles on map):
  green    -- endpoint linked and geometrically consistent
  red      -- endpoint has no link (orphaned / dead-end)
  yellow   -- endpoint links to an element ID that doesn't exist
  orange   -- endpoint is linked but geometry gap exceeds threshold

Lane-level checks (triangles on map):
  red      -- driveable lane at road boundary has no lane link at all
  yellow   -- lane link references a lane ID that doesn't exist on the target road
"""

import numpy as np
import matplotlib.patches as mpatches
import matplotlib.lines as mlines
from dataclasses import dataclass
from typing import List, Dict, Optional, Tuple, Set

from . import geometry as geom_mod


# -- Config -------------------------------------------------------------------

GAP_WARN_THRESHOLD = 0.5   # metres

ROUTABLE_LANE_TYPES = {
    'driving', 'entry', 'exit', 'offRamp', 'onRamp',
    'connectingRamp', 'mwyEntry', 'mwyExit', 'bidirectional',
}


# -- Issue dataclasses --------------------------------------------------------

@dataclass
class EndpointIssue:
    road_id: str
    end:     str            # 'start' | 'end'
    x:       float
    y:       float
    status:  str            # 'ok' | 'orphan' | 'missing_ref' | 'gap'
    detail:  str
    gap_m:   Optional[float] = None


@dataclass
class LaneIssue:
    road_id:  str
    end:      str           # 'start' | 'end'
    lane_id:  int
    x:        float
    y:        float
    status:   str           # 'no_link' | 'bad_ref'
    detail:   str


# -- Colors / labels ----------------------------------------------------------

_EP_COLOR = {
    'ok':          '#44dd44',
    'orphan':      '#ee3333',
    'missing_ref': '#dddd22',
    'gap':         '#ff8800',
}
_EP_LABEL = {
    'ok':          'road endpoint: ok',
    'orphan':      'road endpoint: no link',
    'missing_ref': 'road endpoint: target missing',
    'gap':         f'road endpoint: gap > {GAP_WARN_THRESHOLD} m',
}

_LANE_COLOR = {
    'no_link': '#ee3333',
    'bad_ref': '#dddd22',
}
_LANE_LABEL = {
    'no_link': 'lane: no link declared',
    'bad_ref': 'lane: link target missing',
}


# -- Geometry helpers ---------------------------------------------------------

def _road_endpoints(road) -> Tuple[Tuple[float, float], Tuple[float, float]]:
    rx, ry, _, _ = geom_mod.build_reference_line(road)
    if len(rx) == 0:
        p = (road.geometries[0].x, road.geometries[0].y)
        return p, p
    return (rx[0], ry[0]), (rx[-1], ry[-1])


def _contact_xy(road, contact_point: str) -> Tuple[float, float]:
    start, end = _road_endpoints(road)
    return start if contact_point == 'start' else end


# -- Road-level diagnostics ---------------------------------------------------

def _check_road_endpoints(roads, road_map, junction_ids, ep_cache) -> List[EndpointIssue]:
    issues = []
    for road in roads:
        start_xy, end_xy = ep_cache[road.id]
        for end_name, xy, link_ep in (
            ('start', start_xy, road.link.predecessor if road.link else None),
            ('end',   end_xy,   road.link.successor   if road.link else None),
        ):
            x, y = xy
            if link_ep is None:
                issues.append(EndpointIssue(road.id, end_name, x, y, 'orphan',
                    f'Road {road.id} {end_name}: no link declared'))
                continue

            tid = link_ep.element_id

            if link_ep.element_type == 'junction':
                status = 'ok' if tid in junction_ids else 'missing_ref'
                detail = (f'Road {road.id} {end_name} -> junction {tid}'
                          + ('' if status == 'ok' else ' (NOT FOUND)'))
                issues.append(EndpointIssue(road.id, end_name, x, y, status, detail))
                continue

            if tid not in road_map:
                issues.append(EndpointIssue(road.id, end_name, x, y, 'missing_ref',
                    f'Road {road.id} {end_name} -> road {tid} (NOT FOUND)'))
                continue

            contact = link_ep.contact_point or 'start'
            target_xy = _contact_xy(road_map[tid], contact)
            gap = float(np.hypot(x - target_xy[0], y - target_xy[1]))
            status = 'ok' if gap <= GAP_WARN_THRESHOLD else 'gap'
            issues.append(EndpointIssue(road.id, end_name, x, y, status,
                f'Road {road.id} {end_name} -> road {tid} {contact}: gap {gap:.3f} m',
                gap_m=gap))
    return issues


# -- Lane-level diagnostics ---------------------------------------------------

def _lane_ids_at_boundary(road, boundary: str) -> Set[int]:
    if not road.lane_sections:
        return set()
    sections = sorted(road.lane_sections, key=lambda ls: ls.s)
    ls = sections[0] if boundary == 'start' else sections[-1]
    return {lane.id for lane in ls.left + ls.right}


def _check_lane_links(roads, road_map, junction_ids, ep_cache) -> List[LaneIssue]:
    issues = []
    for road in roads:
        start_xy, end_xy = ep_cache[road.id]
        if not road.lane_sections:
            continue
        sections = sorted(road.lane_sections, key=lambda ls: ls.s)
        first_ls = sections[0]
        last_ls  = sections[-1]

        pred_link = road.link.predecessor if road.link else None
        succ_link = road.link.successor   if road.link else None

        for end_name, ls, road_link_ep, xy in (
            ('start', first_ls, pred_link, start_xy),
            ('end',   last_ls,  succ_link, end_xy),
        ):
            if road_link_ep is None:
                continue

            x, y = xy

            target_lane_ids: Optional[Set[int]] = None
            if road_link_ep.element_type == 'road':
                tid = road_link_ep.element_id
                if tid in road_map:
                    contact = road_link_ep.contact_point or 'start'
                    target_lane_ids = _lane_ids_at_boundary(road_map[tid], contact)

            for lane in ls.left + ls.right:
                if lane.id == 0:
                    continue
                if lane.type not in ROUTABLE_LANE_TYPES:
                    continue

                linked_id = (lane.link.predecessor if end_name == 'start' else lane.link.successor) \
                            if lane.link else None

                if linked_id is None:
                    issues.append(LaneIssue(
                        road_id=road.id, end=end_name,
                        lane_id=lane.id, x=x, y=y,
                        status='no_link',
                        detail=(f'Road {road.id} {end_name}: lane {lane.id} ({lane.type}) '
                                f'has no {"predecessor" if end_name == "start" else "successor"} link'),
                    ))
                elif target_lane_ids is not None and linked_id not in target_lane_ids:
                    issues.append(LaneIssue(
                        road_id=road.id, end=end_name,
                        lane_id=lane.id, x=x, y=y,
                        status='bad_ref',
                        detail=(f'Road {road.id} {end_name}: lane {lane.id} links to lane '
                                f'{linked_id} which does not exist on road {road_link_ep.element_id}'),
                    ))
    return issues


# -- Public API ---------------------------------------------------------------

def diagnose(roads, junction_ids) -> Tuple[List[EndpointIssue], List[LaneIssue]]:
    road_map    = {r.id: r for r in roads}
    ep_cache    = {r.id: _road_endpoints(r) for r in roads}
    ep_issues   = _check_road_endpoints(roads, road_map, junction_ids, ep_cache)
    lane_issues = _check_lane_links(roads, road_map, junction_ids, ep_cache)
    return ep_issues, lane_issues


# -- Renderer overlay ---------------------------------------------------------

def render_overlay(ax, ep_issues: List[EndpointIssue], lane_issues: List[LaneIssue]):
    from collections import defaultdict

    ep_groups = defaultdict(lambda: ([], []))
    for i in ep_issues:
        ep_groups[i.status][0].append(i.x)
        ep_groups[i.status][1].append(i.y)
    for status, (xs, ys) in ep_groups.items():
        ax.scatter(xs, ys, c=_EP_COLOR[status], s=22, marker='o',
                   zorder=10, linewidths=0.4, edgecolors='#000000aa')

    lane_groups = defaultdict(lambda: ([], []))
    for i in lane_issues:
        lane_groups[i.status][0].append(i.x)
        lane_groups[i.status][1].append(i.y)
    for status, (xs, ys) in lane_groups.items():
        ax.scatter(xs, ys, c=_LANE_COLOR[status], s=28, marker='^',
                   zorder=11, linewidths=0.4, edgecolors='#000000aa')

    existing = ax.get_legend()
    if existing:
        ax.add_artist(existing)

    handles = []
    for status in ('ok', 'orphan', 'missing_ref', 'gap'):
        if status in ep_groups:
            handles.append(mlines.Line2D([], [], marker='o', color='none',
                markerfacecolor=_EP_COLOR[status], markersize=7,
                markeredgecolor='#555', label=_EP_LABEL[status]))
    for status in ('no_link', 'bad_ref'):
        if status in lane_groups:
            handles.append(mlines.Line2D([], [], marker='^', color='none',
                markerfacecolor=_LANE_COLOR[status], markersize=7,
                markeredgecolor='#555', label=_LANE_LABEL[status]))

    if handles:
        leg = ax.legend(handles=handles, loc='upper left',
                        framealpha=0.4, facecolor='#1a1a1a', edgecolor='#444',
                        labelcolor='#cccccc', fontsize=7.5,
                        title='Connectivity', title_fontsize=8)
        leg.get_title().set_color('#aaaaaa')


# -- Text report --------------------------------------------------------------

def print_report(ep_issues: List[EndpointIssue], lane_issues: List[LaneIssue]):
    ep_problems   = [i for i in ep_issues if i.status != 'ok']
    ep_ok         = len(ep_issues) - len(ep_problems)

    print(f"\n-- Road-endpoint connectivity ({ep_ok}/{len(ep_issues)} OK) --")
    if not ep_problems:
        print("  No issues found.")
    else:
        by_status = {}
        for i in ep_problems:
            by_status.setdefault(i.status, []).append(i)
        labels = {
            'orphan':      'Orphaned (no link declared)',
            'missing_ref': 'Links to non-existent element',
            'gap':         f'Geometry gap > {GAP_WARN_THRESHOLD} m',
        }
        for status, label in labels.items():
            group = by_status.get(status, [])
            if not group:
                continue
            print(f"\n  {label} ({len(group)}):")
            for i in group:
                gap_str = f'  [{i.gap_m:.2f} m]' if i.gap_m else ''
                print(f"    {i.detail}{gap_str}")

    print(f"\n-- Lane-level connectivity ({len(lane_issues)} issue(s)) --")
    if not lane_issues:
        print("  No issues found.")
    else:
        by_status = {}
        for i in lane_issues:
            by_status.setdefault(i.status, []).append(i)
        labels = {
            'no_link': 'Lane has no link at road boundary',
            'bad_ref': 'Lane link references non-existent lane',
        }
        for status, label in labels.items():
            group = by_status.get(status, [])
            if not group:
                continue
            print(f"\n  {label} ({len(group)}):")
            for i in group:
                print(f"    {i.detail}")
    print()

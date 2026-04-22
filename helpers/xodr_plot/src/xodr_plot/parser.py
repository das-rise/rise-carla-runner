"""
parser.py — Reads an OpenDRIVE (.xodr) XML file and returns structured Road objects.
"""

import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from typing import List, Optional


# ── Data classes ──────────────────────────────────────────────────────────────

@dataclass
class LinkEndpoint:
    element_type: str        # 'road' or 'junction'
    element_id:   str
    contact_point: Optional[str]  # 'start' | 'end' | None (for junctions)


@dataclass
class RoadLink:
    predecessor: Optional[LinkEndpoint]
    successor:   Optional[LinkEndpoint]


@dataclass
class GeomPrimitive:
    s: float
    x: float
    y: float
    hdg: float
    length: float
    type: str    # 'line' | 'arc' | 'spiral' | 'poly3' | 'paramPoly3'
    params: dict


@dataclass
class LaneWidth:
    s_offset: float
    a: float
    b: float
    c: float
    d: float


@dataclass
class LaneOffset:
    s: float
    a: float
    b: float
    c: float
    d: float


@dataclass
class LaneLink:
    predecessor: Optional[int]  # lane id, or None if absent
    successor:   Optional[int]


@dataclass
class JunctionLaneLink:
    from_id: int
    to_id:   int


@dataclass
class JunctionConnection:
    id:              str
    incoming_road:   str
    connecting_road: str
    contact_point:   str   # 'start' | 'end'
    lane_links:      List[JunctionLaneLink]


@dataclass
class Junction:
    id:          str
    connections: List[JunctionConnection]


@dataclass
class Lane:
    id: int
    type: str
    widths: List[LaneWidth]
    link: Optional[LaneLink] = None


@dataclass
class LaneSection:
    s: float
    left: List[Lane]
    right: List[Lane]


@dataclass
class Road:
    id: str
    junction: str   # '-1' means not a junction connecting road
    length: float
    geometries: List[GeomPrimitive]
    lane_sections: List[LaneSection]
    lane_offsets: List[LaneOffset] = field(default_factory=list)
    link: Optional[RoadLink] = None

    @property
    def is_junction_road(self) -> bool:
        return self.junction != '-1'


# ── Helpers ───────────────────────────────────────────────────────────────────

def _f(elem, attr, default=0.0) -> float:
    v = elem.get(attr)
    return float(v) if v is not None else default


def _parse_link_endpoint(elem) -> Optional[LinkEndpoint]:
    if elem is None:
        return None
    return LinkEndpoint(
        element_type=elem.get('elementType', 'road'),
        element_id=elem.get('elementId', ''),
        contact_point=elem.get('contactPoint'),
    )


def _parse_road_link(road_elem) -> Optional[RoadLink]:
    link_elem = road_elem.find('link')
    if link_elem is None:
        return None
    return RoadLink(
        predecessor=_parse_link_endpoint(link_elem.find('predecessor')),
        successor=_parse_link_endpoint(link_elem.find('successor')),
    )


def _parse_junction(junc_elem) -> Junction:
    jid = junc_elem.get('id', '?')
    connections = []
    for conn in junc_elem.findall('connection'):
        lane_links = [
            JunctionLaneLink(int(ll.get('from', 0)), int(ll.get('to', 0)))
            for ll in conn.findall('laneLink')
        ]
        connections.append(JunctionConnection(
            id=conn.get('id', '?'),
            incoming_road=conn.get('incomingRoad', ''),
            connecting_road=conn.get('connectingRoad', ''),
            contact_point=conn.get('contactPoint', 'start'),
            lane_links=lane_links,
        ))
    return Junction(jid, connections)


def _parse_lane_link(lane_elem) -> Optional[LaneLink]:
    link = lane_elem.find('link')
    if link is None:
        return None
    def _lane_id(child_tag):
        child = link.find(child_tag)
        if child is None:
            return None
        v = child.get('id')
        return int(v) if v is not None else None
    return LaneLink(
        predecessor=_lane_id('predecessor'),
        successor=_lane_id('successor'),
    )


def _parse_lane(lane_elem) -> Lane:
    lid   = int(lane_elem.get('id', 0))
    ltype = lane_elem.get('type', 'driving')
    widths = []
    for w in lane_elem.findall('width'):
        widths.append(LaneWidth(
            _f(w, 'sOffset'), _f(w, 'a'), _f(w, 'b'), _f(w, 'c'), _f(w, 'd')
        ))
    return Lane(lid, ltype, widths, link=_parse_lane_link(lane_elem))


def _parse_geometry(geom_elem) -> GeomPrimitive:
    s   = _f(geom_elem, 's')
    x   = _f(geom_elem, 'x')
    y   = _f(geom_elem, 'y')
    hdg = _f(geom_elem, 'hdg')
    ln  = _f(geom_elem, 'length')

    arc   = geom_elem.find('arc')
    spir  = geom_elem.find('spiral')
    p3    = geom_elem.find('poly3')
    pp3   = geom_elem.find('paramPoly3')

    if arc is not None:
        return GeomPrimitive(s, x, y, hdg, ln, 'arc',
                             {'curvature': _f(arc, 'curvature')})
    if spir is not None:
        return GeomPrimitive(s, x, y, hdg, ln, 'spiral',
                             {'curvStart': _f(spir, 'curvStart'),
                              'curvEnd':   _f(spir, 'curvEnd')})
    if p3 is not None:
        return GeomPrimitive(s, x, y, hdg, ln, 'poly3',
                             {k: _f(p3, k) for k in 'abcd'})
    if pp3 is not None:
        params = {k: _f(pp3, k) for k in ('aU','bU','cU','dU','aV','bV','cV','dV')}
        params['pRange'] = pp3.get('pRange', 'normalized')
        return GeomPrimitive(s, x, y, hdg, ln, 'paramPoly3', params)

    return GeomPrimitive(s, x, y, hdg, ln, 'line', {})


# ── Public API ────────────────────────────────────────────────────────────────

def _strip_namespaces(filepath: str) -> ET.Element:
    """Parse XML and strip any namespace prefixes from all tags."""
    tree = ET.parse(filepath)
    root = tree.getroot()
    for elem in root.iter():
        # Strip e.g. '{http://www.opendrive.org}road' → 'road'
        if '}' in elem.tag:
            elem.tag = elem.tag.split('}', 1)[1]
    return root


def parse(filepath: str) -> List[Road]:
    """Parse an OpenDRIVE file and return a list of Road objects."""
    root = _strip_namespaces(filepath)
    roads = []

    for road_elem in root.findall('road'):
        road_id  = road_elem.get('id', '?')
        junction = road_elem.get('junction', '-1')
        length   = _f(road_elem, 'length')

        # Geometry primitives
        geometries = []
        pv = road_elem.find('planView')
        if pv is not None:
            for g in pv.findall('geometry'):
                geometries.append(_parse_geometry(g))

        # Lane sections
        lane_sections = []
        lane_offsets = []
        lanes_elem = road_elem.find('lanes')
        if lanes_elem is not None:
            for lo_elem in lanes_elem.findall('laneOffset'):
                lane_offsets.append(LaneOffset(
                    _f(lo_elem, 's'), _f(lo_elem, 'a'),
                    _f(lo_elem, 'b'), _f(lo_elem, 'c'), _f(lo_elem, 'd')
                ))
            for ls_elem in lanes_elem.findall('laneSection'):
                ls_s  = _f(ls_elem, 's')
                left  = []
                right = []
                for side_name, container in (('left', left), ('right', right)):
                    side = ls_elem.find(side_name)
                    if side is not None:
                        for lane_elem in side.findall('lane'):
                            container.append(_parse_lane(lane_elem))
                lane_sections.append(LaneSection(ls_s, left, right))

        roads.append(Road(road_id, junction, length, geometries, lane_sections,
                          lane_offsets=lane_offsets, link=_parse_road_link(road_elem)))

    junctions = [
        _parse_junction(elem)
        for elem in root.findall('junction')
        if elem.get('id') is not None
    ]

    return roads, junctions

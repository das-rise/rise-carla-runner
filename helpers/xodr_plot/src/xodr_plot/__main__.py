"""
__main__.py — CLI entry point for xodr-plot.

Usage:
    xodr-plot path/to/map.xodr
    xodr-plot path/to/map.xodr --diagnose-connectivity
"""

import sys
import os

from . import parser, geometry, renderer, connectivity, fixer


def main():
    args = sys.argv[1:]

    if not args or args[0] in ('-h', '--help'):
        print("Usage: xodr-plot <file.xodr> [--diagnose-connectivity] [--fix-geometry]")
        sys.exit(0)

    filepath             = args[0]
    diagnose_connectivity = '--diagnose-connectivity' in args
    fix_geometry          = '--fix-geometry' in args
    plot_route            = '--plot-route' in args
    route = args[args.index('--plot-route') + 1] if plot_route else None
    route = eval(route) if route else None

    if not os.path.isfile(filepath):
        print(f"Error: file not found — {filepath}", file=sys.stderr)
        sys.exit(1)

    if not filepath.lower().endswith('.xodr'):
        print("Warning: file does not have a .xodr extension — proceeding anyway.")

    # ── Parse ──────────────────────────────────────────────────────────────────
    print(f"Parsing  {filepath} …")
    try:
        roads, junctions = parser.parse(filepath)
    except Exception as e:
        print(f"Error: failed to parse file — {e}", file=sys.stderr)
        sys.exit(1)

    junction_ids = {j.id for j in junctions}

    print(f"  {len(roads)} road(s) found  "
          f"({sum(1 for r in roads if r.is_junction_road)} junction roads)")

    # ── Build geometry ─────────────────────────────────────────────────────────
    print("Building geometry …")
    all_polys      = []
    all_boundaries = []
    all_refs       = []

    for road in roads:
        polys, bounds, rx, ry = geometry.build_road_data(road)
        all_polys.extend(polys)
        all_boundaries.extend(bounds)
        if len(rx) > 0:
            all_refs.append({
                'xs': rx,
                'ys': ry,
                'is_junction': road.is_junction_road,
            })

    print(f"  {len(all_polys)} lane polygon(s)  |  "
          f"{len(all_refs)} reference line(s)")

    # ── Geometry fix ───────────────────────────────────────────────────────────
    if fix_geometry:
        fixer.fix_geometry(filepath, roads)

    # ── Connectivity diagnostic ────────────────────────────────────────────────
    ep_issues  = None
    lane_issues = None
    if diagnose_connectivity:
        print("Running connectivity diagnostics …")
        ep_issues, lane_issues = connectivity.diagnose(roads, junctions)
        connectivity.print_report(ep_issues, lane_issues)

    # ── Render ─────────────────────────────────────────────────────────────────
    print("Rendering …")
    title = os.path.basename(filepath)
    if diagnose_connectivity:
        title += '  [connectivity diagnostics]'

    renderer.render(all_polys, all_boundaries, all_refs,
                    title=title,
                    ep_issues=ep_issues if diagnose_connectivity else None,
                    route = route)


if __name__ == '__main__':
    main()

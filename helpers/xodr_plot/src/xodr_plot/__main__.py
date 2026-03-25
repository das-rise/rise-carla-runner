"""
__main__.py — CLI entry point for xodr-plot.

Usage:
    xodr-plot path/to/map.xodr
    xodr-plot path/to/map.xodr --diagnose-connectivity
"""

import sys
import os
import argparse
import ast

from . import parser, geometry, renderer, connectivity, fixer


def main():
    argparser = argparse.ArgumentParser(
        prog="xodr-plot",
        description="CLI tool to visualise OpenDRIVE (.xodr) files with Matplotlib"
    )
    argparser.add_argument("filepath", help="Path to the .xodr file")
    argparser.add_argument("--diagnose-connectivity", action="store_true", help="Run connectivity diagnostics")
    argparser.add_argument("--fix-geometry", action="store_true", help="Fix geometry issues")
    argparser.add_argument("--plot-route", type=str, help="Plot a route given as a string of coordinate tuples, e.g., '[(0,0), (10,10)]'")

    args = argparser.parse_args()

    filepath              = args.filepath
    diagnose_connectivity = args.diagnose_connectivity
    fix_geometry          = args.fix_geometry
    
    route = None
    if args.plot_route:
        try:
            route = ast.literal_eval(args.plot_route)
        except (ValueError, SyntaxError) as e:
            print(f"Error: failed to parse route string '{args.plot_route[:50]}...' — {e}", file=sys.stderr)
            sys.exit(1)

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

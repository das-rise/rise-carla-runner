"""
__main__.py — CLI entry point for xodr-plot.

Usage:
    xodr-plot path/to/map.xodr
    xodr-plot path/to/map.xodr --load route.xml --output route.xml
    xodr-plot path/to/map.xodr --test-route route.xml
    xodr-plot path/to/map.xodr --diagnose-connectivity
"""

import argparse
import ast
import os
import sys
from typing import Optional

from . import connectivity, fixer, geometry, parser, renderer


def build_argparser() -> argparse.ArgumentParser:
    argparser = argparse.ArgumentParser(
        prog="xodr-plot",
        description="CLI tool to visualise OpenDRIVE (.xodr) files, create and "
        "edit agent routes, and test them against a live CARLA server",
    )
    argparser.add_argument("filepath", help="Path to the .xodr file")
    argparser.add_argument(
        "--diagnose-connectivity",
        action="store_true",
        help="Run connectivity diagnostics",
    )
    argparser.add_argument(
        "--fix-geometry", action="store_true", help="Fix geometry issues"
    )
    argparser.add_argument(
        "--plot-route",
        type=str,
        help="Plot a route given as a string of coordinate tuples, e.g., '[(0,0), (10,10)]'",
    )
    argparser.add_argument(
        "--plot-trajectories-folder",
        type=str,
        help="Plot all trajectories (.csv) inside a folder",
    )
    argparser.add_argument(
        "--load",
        type=str,
        help="Load an existing agent-route XML file into the editor (GUI mode)",
    )
    argparser.add_argument(
        "--output",
        type=str,
        help="File to write the route XML to (GUI 'Save' button and on exit; the "
        "route is always also printed to stdout on exit)",
    )
    argparser.add_argument(
        "--test-route",
        type=str,
        metavar="ROUTE_XML",
        help="Headless mode: test the given route XML against the CARLA "
        "container (started with `docker start` if needed, left running "
        "afterwards), print a per-segment report, and exit "
        "(0 = all segments routable, 1 = failure)",
    )
    argparser.add_argument(
        "--carla-container",
        type=str,
        help="Name of the CARLA container to use for route testing "
        "(default: CARLA_CONTAINER env var, then CARLA-0.9.15 — the "
        "container created by docker/start-carla-docker.sh)",
    )
    return argparser


def build_map_data(filepath: str, args):
    """Parse the xodr file and build all renderable geometry. Shared by GUI
    and headless-test paths."""
    print(f"Parsing  {filepath} …")
    try:
        roads, junctions = parser.parse(filepath)
    except Exception as e:
        print(f"Error: failed to parse file — {e}", file=sys.stderr)
        sys.exit(1)

    print(
        f"  {len(roads)} road(s) found  "
        f"({sum(1 for r in roads if r.is_junction_road)} junction roads)"
    )

    print("Building geometry …")
    all_polys = []
    all_boundaries = []
    all_refs = []
    for road in roads:
        polys, bounds, rx, ry = geometry.build_road_data(road)
        all_polys.extend(polys)
        all_boundaries.extend(bounds)
        if len(rx) > 0:
            all_refs.append({"xs": rx, "ys": ry, "is_junction": road.is_junction_road})

    print(f"  {len(all_polys)} lane polygon(s)  |  {len(all_refs)} reference line(s)")

    if args.fix_geometry:
        fixer.fix_geometry(filepath, roads)

    ep_issues = None
    lane_issues = None
    if args.diagnose_connectivity:
        print("Running connectivity diagnostics …")
        ep_issues, lane_issues = connectivity.diagnose(roads, junctions)
        connectivity.print_report(ep_issues, lane_issues)

    title = os.path.basename(filepath)
    if args.diagnose_connectivity:
        title += "  [connectivity diagnostics]"

    return all_polys, all_boundaries, all_refs, ep_issues, lane_issues, title


def run_headless_test(
    filepath: str, route_xml: str, carla_container: Optional[str]
) -> None:
    """Test a route XML against the CARLA container; exit 0/1."""
    try:
        import carla

        from . import planner
        from .carla_server import CarlaServerError, CarlaTestServer, log
        from .route import Route
    except ImportError as exc:
        print(
            f"Error: the route-test feature needs the `carla` package "
            f"(and `networkx`) — {exc}\n"
            "These are normally installed together with xodr-plot. This "
            "usually means you are on Python 3.11+ (`carla` wheels only exist "
            "for Python 3.8–3.10) or the install is incomplete — try "
            "`uv sync` or `pip install -e .` again.",
            file=sys.stderr,
        )
        sys.exit(1)

    try:
        route = Route.from_file(route_xml)
    except Exception as e:
        print(f"Error: failed to parse route file {route_xml} — {e}", file=sys.stderr)
        sys.exit(1)
    if len(route.waypoints) < 2:
        print(f"Error: route {route_xml} has fewer than 2 waypoints.", file=sys.stderr)
        sys.exit(1)

    server = CarlaTestServer(filepath, container_name=carla_container)
    try:
        wmap = server.ensure_ready(progress_cb=log)
        locations = [carla.Location(x=wp.x, y=wp.y, z=wp.z) for wp in route.waypoints]
        result = planner.check_route(wmap, locations, progress_cb=log)
    except CarlaServerError as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)
    finally:
        server.teardown()

    for line in result.summary_lines():
        print(line)
    sys.exit(0 if result.ok else 1)


def main() -> None:
    args = build_argparser().parse_args()

    filepath = args.filepath
    if not os.path.isfile(filepath):
        print(f"Error: file not found — {filepath}", file=sys.stderr)
        sys.exit(1)
    if not filepath.lower().endswith(".xodr"):
        print("Warning: file does not have a .xodr extension — proceeding anyway.")

    if args.test_route:
        run_headless_test(filepath, args.test_route, args.carla_container)
        return

    route_string = None
    if args.plot_route:
        try:
            route_string = ast.literal_eval(args.plot_route)
        except (ValueError, SyntaxError) as e:
            print(
                f"Error: failed to parse route string '{args.plot_route[:50]}...' — {e}",
                file=sys.stderr,
            )
            sys.exit(1)

    trajectory_routes = None
    if args.plot_trajectories_folder:
        folder = args.plot_trajectories_folder
        if not os.path.isdir(folder):
            print(f"Error: folder not found — {folder}", file=sys.stderr)
            sys.exit(1)

        import csv

        trajectory_routes = []
        for filename in os.listdir(folder):
            if not filename.endswith(".csv"):
                continue
            filepath_csv = os.path.join(folder, filename)
            trajectory = []
            with open(filepath_csv, "r") as f:
                reader = csv.DictReader(f)
                for row in reader:
                    try:
                        x = float(row["x_loc [m]"])
                        y = float(row["y_loc [m]"])
                        trajectory.append((x, y))
                    except (KeyError, ValueError):
                        pass
            if trajectory:
                trajectory_routes.append(trajectory)

    all_polys, all_boundaries, all_refs, ep_issues, lane_issues, title = build_map_data(
        filepath, args
    )

    # ── Render + interactive route editor ────────────────────────────────────
    print("Rendering …")
    renderer.render(
        all_polys,
        all_boundaries,
        all_refs,
        title=title,
        ep_issues=ep_issues if args.diagnose_connectivity else None,
        lane_issues=lane_issues if args.diagnose_connectivity else None,
        route=route_string,
        routes=trajectory_routes,
    )

    from .route import Route
    from .route_editor import RouteEditor

    initial_route = Route()
    if args.load:
        try:
            initial_route = Route.from_file(args.load)
        except Exception as e:
            print(
                f"Error: failed to parse route file {args.load} — {e}", file=sys.stderr
            )
            sys.exit(1)
        print(f"Loaded {len(initial_route.waypoints)} waypoint(s) from {args.load}")

    import matplotlib.pyplot as plt

    fig = plt.gcf()
    RouteEditor(
        fig,
        fig.axes[0],
        initial_route,
        filepath,
        output_path=args.output,
        carla_container=args.carla_container,
    )
    plt.show()


if __name__ == "__main__":
    main()

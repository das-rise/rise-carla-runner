import carla
import numpy as np
import cv2
import time
import argparse
from pathlib import Path
import math
import xml.etree.ElementTree as ET
import numpy as np


def load_opendrive_map(client, opendrive_file_path):
    """Load an OpenDRIVE map from file"""
    print(f"Loading OpenDRIVE map from: {opendrive_file_path}")

    with open(opendrive_file_path, "r") as f:
        opendrive_content = f.read()

    # Generate a unique map name
    map_name = Path(opendrive_file_path).stem

    # Load the OpenDRIVE map
    vertex_distance = 2.0  # Distance between waypoints
    max_road_length = 500.0  # Maximum road length
    wall_height = 0.0  # Height of walls
    extra_width = 0.0  # Extra width for sidewalks

    client.generate_opendrive_world(
        opendrive_content,
        carla.OpendriveGenerationParameters(
            vertex_distance=vertex_distance,
            max_road_length=max_road_length,
            wall_height=wall_height,
            additional_width=extra_width,
            smooth_junctions=True,
            enable_mesh_visibility=True,
        ),
    )

    print("OpenDRIVE map loaded successfully")
    time.sleep(2)  # Wait for world to stabilize


def setup_camera(world, spawn_points, image_width=4000, image_height=4000, fov=90.0):
    """Set up a bird's-eye view camera above the map"""
    # Calculate the center and bounds of all spawn points
    if not spawn_points:
        print("No spawn points found!")
        return None

    locations = [sp.location for sp in spawn_points]
    x_coords = [loc.x for loc in locations]
    y_coords = [loc.y for loc in locations]
    z_coords = [loc.z for loc in locations]

    center_x = (max(x_coords) + min(x_coords)) / 2
    center_y = (max(y_coords) + min(y_coords)) / 2
    max_z = max(z_coords)

    # Calculate required height based on map size
    width = max(x_coords) - min(x_coords)
    height = max(y_coords) - min(y_coords)
    map_size = max(width, height)
    camera_height = max_z + map_size * 0.8  # Adjust multiplier as needed

    print(
        f"Map bounds: X[{min(x_coords):.1f}, {max(x_coords):.1f}], Y[{min(y_coords):.1f}, {max(y_coords):.1f}]"
    )
    print(f"Camera position: ({center_x:.1f}, {center_y:.1f}, {camera_height:.1f})")

    # Create camera blueprint
    blueprint_library = world.get_blueprint_library()
    camera_bp = blueprint_library.find("sensor.camera.rgb")
    camera_bp.set_attribute("image_size_x", str(image_width))
    camera_bp.set_attribute("image_size_y", str(image_height))
    camera_bp.set_attribute("fov", str(fov))

    # Set camera transform (looking down)
    camera_transform = carla.Transform(
        carla.Location(x=center_x, y=center_y, z=camera_height),
        carla.Rotation(pitch=-90, yaw=0, roll=0),
    )

    # Spawn camera
    camera = world.spawn_actor(camera_bp, camera_transform)
    print("Camera spawned successfully")

    return camera


def draw_spawn_points_in_world(world, spawn_points, lifetime=0, arrow_scale=5.0):
    """Draw spawn points directly in the world using debug helpers"""
    debug = world.debug

    for idx, spawn_point in enumerate(spawn_points):
        loc = spawn_point.location

        # Draw a point at the spawn location
        # Red color for visibility
        debug.draw_point(
            loc,
            size=0.2,
            color=carla.Color(r=255, g=0, b=0),
            life_time=lifetime
        )


# --- OpenDRIVE parser (samples centerlines, inverts Y to match CARLA) ---
import math
import xml.etree.ElementTree as ET
import numpy as np
import matplotlib.pyplot as plt

def parse_opendrive_centerlines(opendrive_file, ds=1.0):
    """
    Parse an OpenDRIVE (.xodr) file and return a list of centerline polylines (Nx2 arrays).
    Y is inverted to match CARLA coordinate system.
    Supports geometry types: line, arc, spiral (basic).
    """
    tree = ET.parse(opendrive_file)
    root = tree.getroot()

    def strip_ns(tag):
        return tag.split("}")[-1]

    polylines = []

    for road in root.findall(".//road"):
        # find planView child
        plan = None
        for c in road:
            if strip_ns(c.tag) == "planView":
                plan = c
                break
        if plan is None:
            continue

        for geom in plan:
            if strip_ns(geom.tag) != "geometry":
                continue

            x0 = float(geom.attrib.get("x", 0.0))
            y0 = float(geom.attrib.get("y", 0.0))
            hdg = float(geom.attrib.get("hdg", 0.0))
            length = float(geom.attrib.get("length", 0.0))

            # detect geometry type (line/arc/spiral)
            child = None
            gtype = "line"
            for c in geom:
                t = strip_ns(c.tag).lower()
                if t in ("line", "arc", "spiral"):
                    gtype = t
                    child = c
                    break

            if length <= 0:
                continue

            n = max(2, int(math.ceil(length / ds)) + 1)
            s = np.linspace(0.0, length, n)

            if gtype == "line":
                xs = x0 + s * np.cos(hdg)
                ys = y0 + s * np.sin(hdg)

            elif gtype == "arc":
                # curvature attribute name may vary
                k = float(child.attrib.get("curvature", child.attrib.get("curv", 0.0)))
                if abs(k) < 1e-12:
                    xs = x0 + s * np.cos(hdg)
                    ys = y0 + s * np.sin(hdg)
                else:
                    theta = hdg + k * s
                    xs = x0 + (np.sin(theta) - np.sin(hdg)) / k
                    ys = y0 - (np.cos(theta) - np.cos(hdg)) / k

            elif gtype == "spiral":
                # linear curvature from curvStart -> curvEnd (best effort)
                k0 = float(child.attrib.get("curvStart", child.attrib.get("curvatureStart", 0.0)))
                k1 = float(child.attrib.get("curvEnd", child.attrib.get("curvatureEnd", k0)))
                theta = hdg + k0*s + 0.5*(k1-k0)*(s**2)/length
                cos_t = np.cos(theta)
                sin_t = np.sin(theta)
                ds_arr = np.diff(s)
                x_inc = np.concatenate(([0.0], 0.5*(cos_t[:-1] + cos_t[1:]) * ds_arr))
                y_inc = np.concatenate(([0.0], 0.5*(sin_t[:-1] + sin_t[1:]) * ds_arr))
                xs = x0 + np.cumsum(x_inc)
                ys = y0 + np.cumsum(y_inc)
            else:
                # fallback to straight
                xs = x0 + s * np.cos(hdg)
                ys = y0 + s * np.sin(hdg)

            # invert Y to match CARLA
            ys = -ys

            polylines.append(np.stack([xs, ys], axis=1))

    return polylines


# --- New plotting function: labels in a top strip + leader lines to spawn points ---
def plot_spawn_points_2d(spawn_points, opendrive_file=None, output_file="spawn_points_2d.png", ds=1.0):
    """
    Plot spawn points and OpenDRIVE centerlines.
    Places one label per spawn point in a horizontal strip above the map (no overlap),
    and draws a thin leader line from each label down to its spawn point.
    Spawn points are red dots.
    """
    # collect spawn coordinates
    xs = np.array([sp.location.x for sp in spawn_points])
    ys = np.array([sp.location.y for sp in spawn_points])
    n = len(xs)
    if n == 0:
        print("No spawn points to plot.")
        return

    # Figure and axes
    fig, ax = plt.subplots(figsize=(10, 10), dpi=100)

    # Plot OpenDRIVE centerlines underneath, if provided
    if opendrive_file:
        try:
            centerlines = parse_opendrive_centerlines(opendrive_file, ds=ds)
            for poly in centerlines:
                ax.plot(poly[:, 0], poly[:, 1], linewidth=0.9, alpha=0.8, color='gray', zorder=0)
        except Exception as e:
            print(f"Warning: failed to parse OpenDRIVE '{opendrive_file}': {e}")

    # Plot spawn points as red dots
    ax.scatter(xs, ys, s=40, c='red', zorder=4)

    # Determine bounds and label strip location
    min_x, max_x = float(xs.min()), float(xs.max())
    min_y, max_y = float(ys.min()), float(ys.max())
    x_span = max_x - min_x if max_x > min_x else 1.0
    y_span = max_y - min_y if max_y > min_y else 1.0

    # label strip Y in data coords: place above max_y by a fraction of y_span (and minimum absolute padding)
    label_gap = max(0.08 * y_span, 2.0)  # data units
    label_y = max_y + label_gap

    # Option: place labels across the X span evenly to avoid overlap. To reduce line crossings,
    # sort spawn points by X and assign label X positions in the same order.
    order = np.argsort(xs)
    # Reserve a horizontal margin so labels don't touch edges
    x_margin = max(0.04 * x_span, 1.0)
    label_min_x = min_x - x_margin
    label_max_x = max_x + x_margin

    # Compute positions equally spaced across [label_min_x, label_max_x]
    label_xs_sorted = np.linspace(label_min_x, label_max_x, n)

    # Map back to original order
    label_xs = np.empty(n, dtype=float)
    label_xs[order] = label_xs_sorted

    # Build labels (strings)
    labels = [str(i) for i in range(n)]

    # Draw leader lines: from spawn point to (label_x, label_y)
    for i in range(n):
        lx = label_xs[i]
        ly = label_y
        px = xs[i]
        py = ys[i]

        # Leader line (thin)
        ax.plot([lx, px], [ly, py], linewidth=0.7, color='black', alpha=0.6, zorder=2)

        # Draw label box (centered horizontally at lx, slightly above ly to leave a small gap)
        bbox_props = dict(boxstyle='round,pad=0.3', facecolor='white', edgecolor='none', alpha=0.95)
        ax.text(lx, ly, labels[i],
                fontsize=8, ha='center', va='center',
                bbox=bbox_props, zorder=5)

    # Optionally, draw small marker at label anchor (not necessary)
    # ax.scatter(label_xs, [label_y]*n, s=6, c='black', zorder=5)

    # Expand axes limits to include labels
    # We need to make sure label_y is visible (add padding)
    pad_x = 0.05 * x_span if x_span > 0 else 1.0
    pad_y = 0.12 * y_span if y_span > 0 else 2.5
    ax.set_xlim(min_x - pad_x, max_x + pad_x)
    ax.set_ylim(min_y - pad_y/2.0, label_y + pad_y)

    ax.set_aspect('equal', adjustable='box')
    ax.set_xlabel("X Coordinate")
    ax.set_ylabel("Y Coordinate")
    ax.set_title("Spawn Points (red) with Top-strip Labels and Leader Lines")
    ax.grid(True, alpha=0.25)
    plt.tight_layout()
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    plt.close(fig)
    print(f"✓ 2D plot saved to: {output_file}")


def main(
    opendrive_file,
    output_image="spawn_points_map.png",
    host="localhost",
    port=2000,
    image_width=1920,
    image_height=1080,
    fov=90.0,
    timeout=10.0,
    verbose=False,
):
    """Main function to visualize spawn points"""
    camera = None
    image_data = {"received": False, "image": None}

    try:
        # Connect to CARLA server
        print(f"Connecting to CARLA server at {host}:{port}...")
        client = carla.Client(host, port)
        client.set_timeout(timeout)

        print(f"CARLA version: {client.get_server_version()}")

        # Load OpenDRIVE map
        load_opendrive_map(client, opendrive_file)

        # Get world and spawn points
        world = client.get_world()
        carla_map = world.get_map()
        spawn_points = carla_map.get_spawn_points()

        print(f"Found {len(spawn_points)} spawn points")

        if not spawn_points:
            print("No spawn points available on this map!")
            return

        # Print spawn point locations if verbose
        if verbose:
            print("\nSpawn Points:")
            for idx, sp in enumerate(spawn_points):
                loc = sp.location
                rot = sp.rotation
                print(
                    f"  {idx}: Location({loc.x:.2f}, {loc.y:.2f}, {loc.z:.2f}) "
                    f"Rotation(pitch={rot.pitch:.1f}, yaw={rot.yaw:.1f}, roll={rot.roll:.1f})"
                )

        # Wait a moment for debug drawings to be visible
        time.sleep(0.5)

        # Set up bird's-eye view camera
        camera = setup_camera(world, spawn_points, image_width, image_height, fov)

        if camera is None:
            print("Failed to setup camera")
            return

        # Wait for camera to be ready
        time.sleep(1)

        # Capture image
        def on_image(image):
            image_data["image"] = image
            image_data["received"] = True

        camera.listen(on_image)

        print("Capturing image...")

        # Draw spawn points in the world using debug helpers
        print("Drawing spawn points in world...")
        draw_spawn_points_in_world(world, spawn_points, lifetime=0)

        # Wait for points to spawn and image to be received
        time.sleep(1)
        start_time = time.time()
        while not image_data["received"] and (time.time() - start_time) < timeout:
            time.sleep(0.1)

        if not image_data["received"]:
            print("Timeout while waiting for image.")
            return

        camera.stop()

        # Save image without additional processing
        print("Saving captured image...")
        img_array = np.frombuffer(image_data["image"].raw_data, dtype=np.uint8)
        img_array = img_array.reshape(
            (image_data["image"].height, image_data["image"].width, 4)
        )
        img_array = img_array[:, :, :3]  # Remove alpha channel
        img_bgr = cv2.cvtColor(img_array, cv2.COLOR_RGB2BGR)

        cv2.imwrite(output_image, img_bgr)
        print(f"✓ Image saved to: {output_image}")
        print(f"✓ Total spawn points marked: {len(spawn_points)}")
        print(f"✓ Image resolution: {image_width}x{image_height}")

        # Additionally, create a 2D plot of spawn points
        file2d = "spawn_points_2d.png"
        plot_spawn_points_2d(spawn_points, opendrive_file=opendrive_file, output_file=file2d, ds=0.33)
        print(f"✓ Additional 2D spawn point plot saved to: {file2d}")


    except Exception as e:
        print(f"Error: {e}")
        import traceback

        traceback.print_exc()

    finally:
        # Clean up
        if camera is not None:
            camera.destroy()
            print("Camera destroyed")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="CARLA OpenDRIVE Spawn Point Visualizer - Load an OpenDRIVE map and generate a bird's-eye view image with numbered spawn points",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s map.xodr
  %(prog)s map.xodr -o spawn_visualization.png
  %(prog)s map.xodr --host 192.168.1.100 --port 2000
  %(prog)s my_custom_map.xodr -o output.png --host localhost --port 2000 --width 2560 --height 1440

Notes:
  - Make sure the CARLA server is running before executing this script
  - The script will automatically calculate optimal camera positioning
  - Spawn points are numbered starting from 0
  - Output image shows red circles with white numbers at each spawn point
        """,
    )

    # Required arguments
    parser.add_argument(
        "opendrive_file",
        type=str,
        help="Path to the OpenDRIVE (.xodr) map file to load",
    )

    # Optional arguments
    parser.add_argument(
        "-o",
        "--output",
        type=str,
        default="spawn_points_map.png",
        metavar="FILE",
        help="Output image file path (default: spawn_points_map.png)",
    )

    parser.add_argument(
        "--host",
        type=str,
        default="localhost",
        metavar="HOST",
        help="CARLA server host address (default: localhost)",
    )

    parser.add_argument(
        "-p",
        "--port",
        type=int,
        default=2000,
        metavar="PORT",
        help="CARLA server port number (default: 2000)",
    )

    parser.add_argument(
        "--width",
        type=int,
        default=4000,
        metavar="PIXELS",
        help="Output image width in pixels (default: 4000)",
    )

    parser.add_argument(
        "--height",
        type=int,
        default=4000,
        metavar="PIXELS",
        help="Output image height in pixels (default: 4000)",
    )

    parser.add_argument(
        "--fov",
        type=float,
        default=90.0,
        metavar="DEGREES",
        help="Camera field of view in degrees (default: 90.0)",
    )

    parser.add_argument(
        "--timeout",
        type=float,
        default=10.0,
        metavar="SECONDS",
        help="Connection timeout in seconds (default: 10.0)",
    )

    parser.add_argument(
        "-v",
        "--verbose",
        action="store_true",
        help="Print detailed spawn point coordinates",
    )

    args = parser.parse_args()

    # Validate file exists
    if not Path(args.opendrive_file).exists():
        parser.error(f"OpenDRIVE file not found: {args.opendrive_file}")

    # Validate image dimensions
    if args.width < 100 or args.height < 100:
        parser.error("Image dimensions must be at least 100x100 pixels")

    if args.width != args.height:
        parser.error(f"Image width and height must be equal! Got {args.width}x{args.height}")

    # Validate port
    if args.port < 1 or args.port > 65535:
        parser.error("Port must be between 1 and 65535")

    # Validate FOV
    if args.fov < 10.0 or args.fov > 170.0:
        parser.error("Field of view must be between 10 and 170 degrees")

    main(
        opendrive_file=args.opendrive_file,
        output_image=args.output,
        host=args.host,
        port=args.port,
        image_width=args.width,
        image_height=args.height,
        fov=args.fov,
        timeout=args.timeout,
        verbose=args.verbose,
    )

"""
route_editor.py — Interactive route editing on top of the rendered map.

Features:
  * all waypoints are visible at all times (numbered marker + heading arrow)
  * left-click empty map        → add waypoint
  * left-click a marker         → select waypoint (angle panel activates)
  * drag a marker               → move waypoint
  * right-click a marker        → delete waypoint
  * left-click the route line   → insert waypoint between the two endpoints
  * inset panel (bottom left)   → heading slider + delete button for the
                                   selected waypoint
  * button bar (bottom)         → Test route / Clear / Save + status line

"Test route" runs in a background thread: it uses the user's CARLA
container (see carla_server.py), traces the route with the same
GlobalRoutePlanner the PCLA agents use at runtime, and reports the result
in-plot (dense route overlay, red highlight for failing segments).
"""

from __future__ import annotations

import queue
import threading
from typing import TYPE_CHECKING, List, Optional, Tuple

if TYPE_CHECKING:
    from . import planner

import matplotlib.pyplot as plt
import matplotlib.widgets as widgets
import numpy as np

from .carla_server import CarlaServerError, CarlaTestServer
from .route import Route, Waypoint

# ── Colors / geometry ────────────────────────────────────────────────────────

_WAYPOINT_COLOR = "#FF5555"
_WAYPOINT_SELECTED_COLOR = "#FFD700"
_ROUTE_LINE_COLOR = "cyan"
_ARROW_COLOR = "#FF5555"
_TEST_OK_COLOR = "#44dd44"
_TEST_FAIL_COLOR = "#ee3333"
_PANEL_BG = "#161616"
_PANEL_FG = "#cccccc"
_PANEL_DIM = "#666666"

_HIT_RADIUS_PX = 12.0
_SEGMENT_HIT_PX = 8.0
_ARROW_LENGTH_M = 8.0


def _panel_bg(ax) -> None:
    ax.set_facecolor(_PANEL_BG)
    for spine in ax.spines.values():
        spine.set_edgecolor("#333333")
    ax.set_xticks([])
    ax.set_yticks([])


class RouteEditor:
    """Owns the interactive route layer of the main figure."""

    def __init__(
        self,
        fig,
        ax,
        route: Route,
        xodr_filepath: str,
        output_path: Optional[str] = None,
        carla_container: Optional[str] = None,
    ):
        self.fig = fig
        self.ax = ax
        self.route = route
        self.xodr_filepath = xodr_filepath
        self.output_path = output_path

        self.server = CarlaTestServer(xodr_filepath, container_name=carla_container)
        self.selected_index: Optional[int] = None
        self._route_artists: List = []
        self._test_artists: List = []
        self._drag_index: Optional[int] = None
        self._saved_nav_mode = None
        self._closed = False

        self._queue: "queue.Queue" = queue.Queue()
        self._test_thread: Optional[threading.Thread] = None
        self._test_running = False

        self._build_ui()
        self._connect_events()
        self._update_status("Ready. Left-click to add waypoints.")
        self._redraw_route()
        self._draw_timer = fig.canvas.new_timer(interval=0.1)
        self._draw_timer.add_callback(self._drain_queue)
        self._draw_timer.start()

    # ── UI construction ──────────────────────────────────────────────────────

    @staticmethod
    def _abs(outer, inner) -> List[float]:
        """Convert an [x, y, w, h] rect relative to `outer` into absolute
        figure coordinates."""
        ox, oy, ow, oh = outer
        ix, iy, iw, ih = inner
        return [ox + ix * ow, oy + iy * oh, iw * ow, ih * oh]

    def _build_ui(self) -> None:
        fig = self.fig

        # Button bar
        bar_rect = [0.0, 0.0, 1.0, 0.055]
        bar_ax = fig.add_axes(bar_rect)
        _panel_bg(bar_ax)
        self.btn_test = widgets.Button(
            fig.add_axes(self._abs(bar_rect, [0.01, 0.2, 0.13, 0.6])),
            "Test route",
            color="#233b23",
        )
        self.btn_test.label.set_color(_PANEL_FG)
        self.btn_test.label.set_fontsize(9)
        self.btn_clear = widgets.Button(
            fig.add_axes(self._abs(bar_rect, [0.15, 0.2, 0.13, 0.6])),
            "Clear",
            color="#3b2323",
        )
        self.btn_clear.label.set_color(_PANEL_FG)
        self.btn_clear.label.set_fontsize(9)
        self.btn_save = widgets.Button(
            fig.add_axes(self._abs(bar_rect, [0.29, 0.2, 0.13, 0.6])),
            "Save",
            color="#232b3b",
        )
        self.btn_save.label.set_color(_PANEL_FG)
        self.btn_save.label.set_fontsize(9)
        self._status_text = bar_ax.text(
            0.99,
            0.5,
            "",
            ha="right",
            va="center",
            color=_PANEL_FG,
            fontsize=8.5,
            family="monospace",
        )
        self.btn_test.on_clicked(self._on_test_clicked)
        self.btn_clear.on_clicked(self._on_clear_clicked)
        self.btn_save.on_clicked(self._on_save_clicked)

        # Inset waypoint panel
        panel_rect = [0.015, 0.075, 0.22, 0.22]
        panel_ax = fig.add_axes(panel_rect)
        _panel_bg(panel_ax)
        self._panel_ax = panel_ax
        self._panel_title = panel_ax.text(
            0.5,
            0.95,
            "No waypoint selected",
            transform=panel_ax.transAxes,
            ha="center",
            va="top",
            color=_PANEL_DIM,
            fontsize=9,
            family="monospace",
        )
        slider_ax = fig.add_axes(self._abs(panel_rect, [0.1, 0.35, 0.8, 0.22]))
        self._angle_slider = widgets.Slider(
            slider_ax, "Yaw", -180.0, 180.0, valinit=0.0, valstep=1.0
        )
        self._angle_slider.valtext.set_color(_PANEL_FG)
        self._angle_slider.label_on = False
        self._panel_angle_text = panel_ax.text(
            0.5,
            0.62,
            "yaw: 0.0°",
            transform=panel_ax.transAxes,
            ha="center",
            va="center",
            color=_PANEL_FG,
            fontsize=9,
            family="monospace",
        )
        self._panel_delete_btn = widgets.Button(
            fig.add_axes(self._abs(panel_rect, [0.25, 0.03, 0.5, 0.22])),
            "Delete waypoint",
            color="#3b2323",
        )
        self._panel_delete_btn.label.set_color(_PANEL_FG)
        self._panel_delete_btn.label.set_fontsize(8)
        self._angle_slider.on_changed(self._on_slider_changed)
        self._panel_delete_btn.on_clicked(self._on_panel_delete)
        self._set_panel_enabled(False)

    def _set_panel_enabled(self, enabled: bool) -> None:
        self._angle_slider.set_active(enabled)
        self._panel_delete_btn.set_active(enabled)
        self._angle_slider.valtext.set_color(_PANEL_FG if enabled else _PANEL_DIM)

    # ── Event wiring ─────────────────────────────────────────────────────────

    def _connect_events(self) -> None:
        canvas = self.fig.canvas
        canvas.mpl_connect("button_press_event", self._on_button_press)
        canvas.mpl_connect("motion_notify_event", self._on_motion)
        canvas.mpl_connect("button_release_event", self._on_button_release)
        canvas.mpl_connect("close_event", self._on_close)

    # ── Hit testing ──────────────────────────────────────────────────────────

    def _display_point(self, px: float, py: float) -> np.ndarray:
        return self.ax.transData.transform((px, py))

    def _hit_test(self, event) -> Optional[int]:
        """Index of the waypoint marker under the mouse, or None."""
        if event.inaxes is not self.ax or event.xdata is None or event.ydata is None:
            return None
        mouse_disp = self._display_point(event.xdata, event.ydata)
        best_dist, best_idx = None, None
        for i, wp in enumerate(self.route.waypoints):
            wx, wy = Route.waypoint_to_plot(wp)
            dist = np.hypot(*(self._display_point(wx, wy) - mouse_disp))
            if dist <= _HIT_RADIUS_PX and (best_dist is None or dist < best_dist):
                best_dist, best_idx = dist, i
        return best_idx

    def _segment_hit(self, px: float, py: float) -> Optional[Tuple[int, int]]:
        """(i, i+1) if the mouse is on the route line between waypoints i and i+1."""
        if len(self.route.waypoints) < 2:
            return None
        mouse_disp = self._display_point(px, py)
        pts = np.array(
            [
                self._display_point(*Route.waypoint_to_plot(wp))
                for wp in self.route.waypoints
            ]
        )
        for i in range(len(pts) - 1):
            a, b = pts[i], pts[i + 1]
            ab = b - a
            denom = float(ab @ ab)
            t = (
                0.0
                if denom < 1e-9
                else float(np.clip(((mouse_disp - a) @ ab) / denom, 0.0, 1.0))
            )
            if np.hypot(*(mouse_disp - (a + t * ab))) <= _SEGMENT_HIT_PX:
                return i, i + 1
        return None

    # ── Mouse events ─────────────────────────────────────────────────────────

    def _on_button_press(self, event) -> None:
        if self._closed or event.inaxes is not self.ax:
            return
        if event.button == 1:
            idx = self._hit_test(event)
            if idx is not None:
                self._select(idx)
                self._begin_drag(idx)
                return
            if event.xdata is None or event.ydata is None:
                return
            segment = self._segment_hit(event.xdata, event.ydata)
            if segment is not None:
                self._insert_between(*segment)
            else:
                self._add_waypoint(event.xdata, event.ydata)
        elif event.button == 3:
            idx = self._hit_test(event)
            if idx is not None:
                self._remove_waypoint(idx)

    def _begin_drag(self, index: int) -> None:
        self._drag_index = index
        toolbar = getattr(self.fig.canvas.manager, "nav_toolbar", None)
        if toolbar is not None:
            self._saved_nav_mode = getattr(toolbar, "_active", None)
            try:
                toolbar._set_navigation_mode(None)
            except Exception:
                self._saved_nav_mode = None

    def _on_motion(self, event) -> None:
        if self._drag_index is None or event.inaxes is not self.ax:
            return
        if event.xdata is None or event.ydata is None:
            return
        x, y = Route.plot_to_carla(event.xdata, event.ydata)
        wp = self.route.waypoints[self._drag_index]
        wp.x, wp.y = x, y
        self._redraw_route()

    def _on_button_release(self, event) -> None:
        if self._drag_index is None:
            return
        self._drag_index = None
        toolbar = getattr(self.fig.canvas.manager, "nav_toolbar", None)
        if toolbar is not None and self._saved_nav_mode is not None:
            try:
                toolbar._set_navigation_mode(self._saved_nav_mode)
            except Exception:
                pass
        self._saved_nav_mode = None
        self._redraw_route()

    def _on_close(self, event) -> None:
        if self._closed:
            return
        self._closed = True
        if self._test_thread is not None and self._test_thread.is_alive():
            # Wait for the in-flight test so it can't be orphaned mid-way
            # (leaving the test container running).
            self._test_thread.join(timeout=900)
        self._print_route_on_exit()
        if self.output_path and self.route.waypoints:
            with open(self.output_path, "w") as fh:
                fh.write(self.route.to_xml() + "\n")
        self.server.teardown()

    # ── Route operations ─────────────────────────────────────────────────────

    def _add_waypoint(self, px: float, py: float) -> None:
        x, y = Route.plot_to_carla(px, py)
        index = self.route.add(Waypoint(x=x, y=y, yaw=0.0))
        self._select(index)
        self._update_status(
            f"Added waypoint {index + 1} — set its heading in the panel."
        )

    def _remove_waypoint(self, index: int) -> None:
        self.route.remove(index)
        self._select(None)
        self._clear_test_overlay()
        self._redraw_route()
        self._update_status(
            f"Deleted waypoint {index + 1} ({len(self.route.waypoints)} left)."
        )

    def _insert_between(self, i: int, j: int) -> None:
        wp = self.route.interpolated_waypoint(i, j)
        self.route.waypoints.insert(j, wp)
        self._select(j)
        self._clear_test_overlay()
        self._redraw_route()
        self._update_status(
            f"Inserted waypoint {j + 1} between waypoints {i + 1} and {j + 2}."
        )

    def _select(self, index: Optional[int]) -> None:
        self.selected_index = index
        if index is None:
            self._panel_title.set_text("No waypoint selected")
            self._set_panel_enabled(False)
        else:
            wp = self.route.waypoints[index]
            self._panel_title.set_text(
                f"Waypoint {index + 1} of {len(self.route.waypoints)}"
            )
            self._set_panel_enabled(True)
            self._angle_slider.val = wp.yaw
            self._panel_angle_text.set_text(f"yaw: {wp.yaw:.1f}°")
        self._redraw_route()
        self.fig.canvas.draw_idle()

    def _on_slider_changed(self, value) -> None:
        if self.selected_index is None:
            return
        self.route.waypoints[self.selected_index].yaw = value
        self._panel_angle_text.set_text(f"yaw: {value:.1f}°")
        self._redraw_route()

    def _on_panel_delete(self, event) -> None:
        if self.selected_index is not None:
            self._remove_waypoint(self.selected_index)

    # ── Button bar ───────────────────────────────────────────────────────────

    def _on_test_clicked(self, event) -> None:
        if self._test_running:
            return
        if len(self.route.waypoints) < 2:
            self._update_status("Need at least 2 waypoints to test a route.")
            return
        self._test_running = True
        self.btn_test.set_active(False)
        self._clear_test_overlay()
        self._update_status("Testing route… (starting container on first test)")
        self._test_thread = threading.Thread(target=self._test_worker, daemon=True)
        self._test_thread.start()

    def _on_clear_clicked(self, event) -> None:
        self.route.clear()
        self._select(None)
        self._clear_test_overlay()
        self._redraw_route()
        self._update_status("Route cleared.")

    def _on_save_clicked(self, event) -> None:
        if not self.route.waypoints:
            self._update_status("Nothing to save — the route is empty.")
            return
        if self.output_path:
            with open(self.output_path, "w") as fh:
                fh.write(self.route.to_xml() + "\n")
            self._update_status(f"Saved route to {self.output_path}")
        else:
            self._print_route_on_exit()
            self._update_status("No --output given — route printed to stdout.")

    def _print_route_on_exit(self) -> None:
        if len(self.route.waypoints) > 0:
            print(self.route.to_xml())

    def _update_status(self, message: str) -> None:
        self._status_text.set_text(message)
        self.fig.canvas.draw_idle()

    # ── Route rendering ──────────────────────────────────────────────────────

    def _redraw_route(self) -> None:
        for artist in self._route_artists:
            artist.remove()
        self._route_artists = []
        ax, route = self.ax, self.route
        if not route.waypoints:
            self.fig.canvas.draw_idle()
            return

        # Connecting polyline
        pts = np.array([Route.waypoint_to_plot(wp) for wp in route.waypoints])
        (line,) = ax.plot(
            pts[:, 0],
            pts[:, 1],
            color=_ROUTE_LINE_COLOR,
            linewidth=1.6,
            alpha=0.8,
            zorder=5,
        )
        self._route_artists.append(line)

        # Markers + heading arrows + numbers
        for i, wp in enumerate(route.waypoints):
            px, py = Route.waypoint_to_plot(wp)
            selected = i == self.selected_index
            color = _WAYPOINT_SELECTED_COLOR if selected else _WAYPOINT_COLOR
            (marker,) = ax.plot(
                [px],
                [py],
                marker="o",
                markersize=7,
                color=color,
                markeredgecolor="white",
                markeredgewidth=0.8,
                zorder=7,
            )
            self._route_artists.append(marker)

            plot_angle = np.deg2rad(Route.yaw_to_plot_angle(wp.yaw))
            arrow = ax.annotate(
                "",
                xy=(
                    px + _ARROW_LENGTH_M * np.cos(plot_angle),
                    py + _ARROW_LENGTH_M * np.sin(plot_angle),
                ),
                xytext=(px, py),
                arrowprops=dict(
                    arrowstyle="-|>",
                    color=_ARROW_COLOR,
                    lw=1.8,
                    mutation_scale=14,
                ),
                zorder=6,
            )
            self._route_artists.append(arrow)

            label = ax.text(
                px + 1.2,
                py + 1.2,
                str(i + 1),
                color=color,
                fontsize=8,
                fontweight="bold",
                zorder=8,
            )
            self._route_artists.append(label)

        self.fig.canvas.draw_idle()

    # ── Route testing ────────────────────────────────────────────────────────

    def _test_worker(self) -> None:
        import carla

        from . import planner

        waypoints = list(self.route.waypoints)
        try:
            wmap = self.server.ensure_ready(
                progress_cb=lambda msg: self._queue.put(("progress", msg))
            )
            locations = [carla.Location(x=wp.x, y=wp.y, z=wp.z) for wp in waypoints]
            result = planner.check_route(
                wmap,
                locations,
                progress_cb=lambda msg: self._queue.put(("progress", msg)),
            )
            self._queue.put(("result", result))
        except CarlaServerError as exc:
            self._queue.put(("error", str(exc)))
        except Exception as exc:  # defensive: never kill the UI
            self._queue.put(("error", f"unexpected error while testing: {exc}"))

    def _drain_queue(self) -> None:
        try:
            while True:
                kind, payload = self._queue.get_nowait()
                if kind == "progress":
                    self._update_status(payload)
                elif kind == "error":
                    self._finish_test(error=payload)
                elif kind == "result":
                    self._finish_test(result=payload)
        except queue.Empty:
            pass

    def _finish_test(self, result=None, error=None) -> None:
        self._test_running = False
        self.btn_test.set_active(True)
        if error is not None:
            self._update_status("Test failed.")
            self._show_result_dialog(f"Route test failed:\n\n{error}")
            return

        if result.ok:
            self._update_status(
                f"Route OK — {len(result.segments)} segment(s), "
                f"{result.total_length_m:.0f} m."
            )
        else:
            failed = [s for s in result.segments if not s.ok]
            self._update_status(
                f"Route FAILED — {len(failed)}/{len(result.segments)} segment(s) unroutable."
            )

        self._draw_test_overlay(result)
        self._show_result_dialog("\n".join(result.summary_lines()))

    def _draw_test_overlay(self, result: "planner.RouteTestResult") -> None:
        self._clear_test_overlay()
        ax = self.ax
        for seg in result.segments:
            if seg.points:
                # seg.points are CARLA coordinates (x, y); the map is drawn
                # in plot coordinates (y_carla, x_carla).
                pts = np.array([Route.carla_to_plot(x, y) for x, y in seg.points])
                (line,) = ax.plot(
                    pts[:, 0],
                    pts[:, 1],
                    color=_TEST_OK_COLOR,
                    linewidth=2.5,
                    alpha=0.85,
                    zorder=4,
                )
                self._test_artists.append(line)
            else:
                i = seg.start_index
                if i + 1 >= len(self.route.waypoints):
                    continue  # route was edited while the test ran
                a = Route.waypoint_to_plot(self.route.waypoints[i])
                b = Route.waypoint_to_plot(self.route.waypoints[i + 1])
                (line,) = ax.plot(
                    [a[0], b[0]],
                    [a[1], b[1]],
                    color=_TEST_FAIL_COLOR,
                    linewidth=2.5,
                    linestyle=(0, (4, 3)),
                    alpha=0.9,
                    zorder=4,
                )
                self._test_artists.append(line)
        self.fig.canvas.draw_idle()

    def _clear_test_overlay(self) -> None:
        for artist in self._test_artists:
            artist.remove()
        self._test_artists = []

    # ── Result dialog ────────────────────────────────────────────────────────

    def _show_result_dialog(self, message: str) -> None:
        dialog = plt.figure(figsize=(7, 3.5))
        dialog.patch.set_facecolor(_PANEL_BG)
        ax = dialog.add_axes([0.03, 0.35, 0.94, 0.6])
        ax.set_facecolor(_PANEL_BG)
        ax.set_xticks([])
        ax.set_yticks([])
        for spine in ax.spines.values():
            spine.set_visible(False)
        ax.text(
            0.0,
            1.0,
            message,
            transform=ax.transAxes,
            color=_PANEL_FG,
            fontsize=9,
            family="monospace",
            va="top",
            wrap=True,
        )
        btn_ax = dialog.add_axes([0.35, 0.05, 0.3, 0.2])
        button = widgets.Button(btn_ax, "OK", color="#232b3b")
        button.label.set_color(_PANEL_FG)
        button.on_clicked(lambda _e: plt.close(dialog))
        dialog.canvas.mpl_connect("close_event", lambda _e: None)
        dialog.show()

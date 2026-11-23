"""Scripted GUI tests for xodr_plot.route_editor.

Drives the matplotlib event system with synthetic events on the Agg
backend (no display needed): add / select / set-yaw / drag / delete /
insert, XML output, the test-button worker wiring, and window close.
"""

import time

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt  # noqa: E402
import pytest  # noqa: E402
from matplotlib.backend_bases import MouseEvent  # noqa: E402
from xodr_plot.route import Route  # noqa: E402
from xodr_plot.route_editor import RouteEditor  # noqa: E402

# ── Fixtures / helpers ────────────────────────────────────────────────────────


@pytest.fixture()
def editor():
    fig, ax = plt.subplots(figsize=(10, 8))
    ax.set_xlim(-100, 100)
    ax.set_ylim(-100, 100)
    ed = RouteEditor(fig, ax, Route(), xodr_filepath="dummy.xodr")
    # A real draw is needed for canvas.inaxes() to see the editor's
    # extra axes (draw_idle is a no-op on Agg without an event loop).
    fig.canvas.draw()
    yield ed
    ed._draw_timer.stop()
    plt.close("all")


def disp(ed, px, py):
    """Data (plot) coordinates → display (pixel) coordinates."""
    return ed.ax.transData.transform((px, py))


def _post(ed, name, x, y, button=None):
    """Post a synthetic mouse event via the (non-deprecated) callback API."""
    event = MouseEvent(name, ed.fig.canvas, x, y, button)
    ed.fig.canvas.callbacks.process(name, event)


def click(ed, px, py, button=1):
    x, y = disp(ed, px, py)
    _post(ed, "button_press_event", x, y, button)


def move(ed, px, py):
    x, y = disp(ed, px, py)
    _post(ed, "motion_notify_event", x, y, None)


def release(ed):
    _post(ed, "button_release_event", 0, 0, 1)


def wait_for(ed, condition, timeout=10.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        ed._drain_queue()
        if condition():
            return True
        time.sleep(0.02)
    ed._drain_queue()
    return condition()


# ── Add / select / yaw ────────────────────────────────────────────────────────


def test_click_empty_map_adds_waypoint(editor):
    click(editor, 10.0, 20.0)
    assert len(editor.route.waypoints) == 1
    wp = editor.route.waypoints[0]
    # plot (10, 20) → CARLA (x=plot_y, y=plot_x)
    assert wp.x == pytest.approx(20.0)
    assert wp.y == pytest.approx(10.0)
    assert wp.yaw == 0.0
    assert editor.selected_index == 0
    assert "Waypoint 1" in editor._panel_title.get_text()


def test_click_selects_existing_waypoint(editor):
    click(editor, 10.0, 20.0)
    click(editor, -30.0, -40.0)
    assert len(editor.route.waypoints) == 2
    # clicking back on the first marker re-selects it
    click(editor, 10.0, 20.0)
    assert editor.selected_index == 0


def test_slider_updates_selected_yaw(editor):
    click(editor, 10.0, 20.0)
    editor._angle_slider.set_val(90.0)
    assert editor.route.waypoints[0].yaw == 90.0
    assert "90.0" in editor._panel_angle_text.get_text()
    # deselecting disables the panel
    click(editor, 50.0, 50.0)  # adds waypoint 2 and selects it
    click(editor, 10.0, 20.0)  # re-select waypoint 1
    editor._select(None)
    assert not editor._angle_slider.active


def test_slider_ignored_when_nothing_selected(editor):
    editor._angle_slider.set_val(45.0)
    assert editor.route.waypoints == []


# ── Drag ──────────────────────────────────────────────────────────────────────


def test_drag_moves_waypoint(editor):
    click(editor, 10.0, 20.0)  # adds wp1 at plot (10, 20)
    click(editor, 10.0, 20.0)  # second press on the marker starts a drag
    move(editor, 15.0, 25.0)
    release(editor)
    wp = editor.route.waypoints[0]
    # plot (15, 25) → CARLA (x=25, y=15)
    assert wp.x == pytest.approx(25.0)
    assert wp.y == pytest.approx(15.0)


# ── Delete ────────────────────────────────────────────────────────────────────


def test_right_click_deletes_waypoint(editor):
    click(editor, 10.0, 20.0)
    click(editor, -30.0, -40.0)
    assert len(editor.route.waypoints) == 2
    click(editor, 10.0, 20.0, button=3)
    assert len(editor.route.waypoints) == 1
    assert editor.route.waypoints[0].x == pytest.approx(-40.0)  # second remains
    assert editor.selected_index is None


def test_panel_delete_button(editor):
    click(editor, 10.0, 20.0)
    editor._on_panel_delete(None)
    assert editor.route.waypoints == []


def test_clear_button(editor):
    click(editor, 10.0, 20.0)
    click(editor, -30.0, -40.0)
    editor._on_clear_clicked(None)
    assert editor.route.waypoints == []


# ── Insert ────────────────────────────────────────────────────────────────────


def test_click_on_route_line_inserts_waypoint(editor):
    click(editor, 0.0, 0.0)
    click(editor, 100.0, 0.0)
    # click on the line between them (the line runs along plot y=0)
    click(editor, 50.0, 0.0)
    assert len(editor.route.waypoints) == 3
    mid = editor.route.waypoints[1]
    # midpoint in CARLA coords
    assert mid.x == pytest.approx(0.0)
    assert mid.y == pytest.approx(50.0)
    assert editor.selected_index == 1


# ── XML output ────────────────────────────────────────────────────────────────


def test_exit_prints_route_xml(editor, capsys):
    click(editor, 10.0, 20.0)
    editor._angle_slider.set_val(90.0)
    click(editor, 50.0, 50.0)
    editor._on_close(None)
    out = capsys.readouterr().out
    assert '<?xml version="1.0" ?>' in out
    assert '<route id="_" town="_">' in out
    assert 'x="20.000" y="10.000" yaw="90.000"' in out
    assert 'x="50.000" y="50.000" yaw="0.000"' in out
    assert out.count("<waypoint") == 2


def test_exit_without_waypoints_prints_nothing(editor, capsys):
    editor._on_close(None)
    assert "<route" not in capsys.readouterr().out


def test_save_button_with_output_path(editor, tmp_path):
    out_file = tmp_path / "route.xml"
    editor.output_path = str(out_file)
    click(editor, 10.0, 20.0)
    editor._on_save_clicked(None)
    assert out_file.exists()
    assert "waypoint" in out_file.read_text()


def test_save_button_without_output_path_prints_to_stdout(editor, capsys):
    click(editor, 10.0, 20.0)
    editor._on_save_clicked(None)
    assert "<waypoint" in capsys.readouterr().out


# ── Test-route wiring ─────────────────────────────────────────────────────────


def test_test_button_rejects_short_route(editor):
    click(editor, 10.0, 20.0)
    editor._on_test_clicked(None)
    assert "at least 2 waypoints" in editor._status_text.get_text()
    assert not editor._test_running


def test_test_button_runs_worker_and_shows_result(editor, monkeypatch):
    planner = pytest.importorskip("xodr_plot.planner")

    fake_result = planner.RouteTestResult(
        segments=[
            planner.SegmentResult(
                start_index=0,
                ok=True,
                points=[(0.0, 0.0), (1.0, 1.0), (2.0, 0.0)],
                length_m=3.0,
            )
        ],
        warnings=[],
    )
    monkeypatch.setattr(planner, "check_route", lambda *a, **k: fake_result)

    class _FakeServer:
        def __init__(self):
            self.teardowns = 0

        def ensure_ready(self, progress_cb=None):
            return "FAKE_MAP"

        def teardown(self):
            self.teardowns += 1

    editor.server = _FakeServer()
    click(editor, 0.0, 0.0)
    click(editor, 100.0, 0.0)
    editor._on_test_clicked(None)
    assert editor._test_running
    assert wait_for(editor, lambda: not editor._test_running)
    assert "Route OK" in editor._status_text.get_text()
    assert len(editor._test_artists) == 1  # one dense-route overlay line
    # a result dialog figure was created
    assert len(plt.get_fignums()) >= 2


def test_test_button_shows_error_on_server_failure(editor, monkeypatch):
    from xodr_plot.carla_server import CarlaServerError

    class _FailingServer:
        def ensure_ready(self, progress_cb=None):
            raise CarlaServerError("no CARLA docker image found")

        def teardown(self):
            pass

    editor.server = _FailingServer()
    click(editor, 0.0, 0.0)
    click(editor, 100.0, 0.0)
    editor._on_test_clicked(None)
    assert wait_for(editor, lambda: not editor._test_running)
    assert "Test failed" in editor._status_text.get_text()


def test_close_triggers_teardown(editor, monkeypatch):
    class _SpyServer:
        def __init__(self):
            self.teardowns = 0

        def teardown(self):
            self.teardowns += 1

    editor.server = _SpyServer()
    click(editor, 0.0, 0.0)
    editor._on_close(None)
    assert editor.server.teardowns == 1
    # double-close is a no-op
    editor._on_close(None)
    assert editor.server.teardowns == 1

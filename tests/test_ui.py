import os
from unittest.mock import patch
from onshape2xacro.ui import (
    LinkMeshStats,
    ExportStats,
    NullExportUI,
    RichExportUI,
    suppress_c_stdout,
)


def test_stats_dataclasses():
    ls = LinkMeshStats(name="test_link", visual_formats=["obj"], visual_size_mb=1.5)
    assert ls.name == "test_link"
    assert ls.visual_size_mb == 1.5

    stats = ExportStats(robot_name="test_robot", num_links=10)
    stats.link_stats.append(ls)
    assert stats.robot_name == "test_robot"
    assert len(stats.link_stats) == 1


def test_null_export_ui():
    ui = NullExportUI()
    # These should all do nothing and not raise errors
    ui.phase_start("load")
    ui.phase_done("load")
    ui.mesh_progress_start("label", 10)
    ui.mesh_progress_advance("label")
    ui.mesh_progress_done("label")
    ui.finish_progress()
    ui.print_summary(ExportStats())
    ui.log("test")


@patch("rich.console.Console")
def test_rich_export_ui_phases(mock_console_class):
    mock_console = mock_console_class.return_value
    ui = RichExportUI()

    ui.phase_start("load", detail="working")
    assert mock_console.print.called

    ui.phase_done("load", detail="finished")
    assert mock_console.print.call_count >= 2


@patch("rich.console.Console")
@patch("rich.progress.Progress")
def test_rich_export_ui_progress(mock_progress_class, mock_console_class):
    mock_progress = mock_progress_class.return_value
    mock_progress.add_task.return_value = "task_1"

    ui = RichExportUI()
    ui.mesh_progress_start("meshes", 10)
    assert mock_progress.start.called
    assert mock_progress.add_task.called

    ui.mesh_progress_advance("meshes", description="part_1")
    assert mock_progress.update.called

    ui.mesh_progress_done("meshes", detail="done")
    ui.finish_progress()
    assert mock_progress.stop.called


@patch("rich.console.Console")
def test_rich_export_ui_summary(mock_console_class):
    mock_console = mock_console_class.return_value
    ui = RichExportUI()

    stats = ExportStats(
        robot_name="test_robot",
        output_path="/tmp/out",
        num_links=1,
        num_joints=1,
        total_mass_kg=10.0,
        compressed_count=1,
        total_collision_stls=5,
        coacd_fallback_count=1,
        missing_mesh_links=1,
        missing_mesh_parts=2,
        missing_meshes_path="MISSING.md",
        inertia_debug_path="debug.md",
        warnings=["test warning"],
    )
    stats.link_stats = [
        LinkMeshStats(
            name="link1",
            visual_size_mb=1.0,
            compressed=True,
            collision_hulls=3,
            collision_fallback=True,
            has_inertia=True,
            mass_kg=1.0,
        )
    ]

    with patch.object(ui, "_render_table_str", return_value="mock table"):
        ui.print_summary(stats)

    assert mock_console.print.called


def test_suppress_c_stdout_devnull():
    # Difficult to verify actually silenced at C level, but can verify context manager runs
    with suppress_c_stdout():
        print("This should still show in Python stdout but be redirected at C level")


def test_suppress_c_stdout_file(tmp_path):
    log_file = tmp_path / "c_output.log"
    with suppress_c_stdout(log_path=log_file):
        # We can't easily write to C-level stdout from Python's 'print'
        # because it might be buffered or handled differently,
        # but we can try os.write(1, ...)
        os.write(1, b"hello from c-level\n")

    assert log_file.exists()
    assert b"hello from c-level" in log_file.read_bytes()


@patch("rich.console.Console")
def test_rich_export_ui_log(mock_console_class):
    mock_console = mock_console_class.return_value
    ui = RichExportUI()
    ui.log("info message")
    ui.log("warning message", level="warning")
    ui.log("error message", level="error")
    assert mock_console.print.call_count == 3


def test_rich_export_ui_render_table_str():
    from rich.table import Table

    ui = RichExportUI()
    table = Table()
    table.add_column("Col1")
    table.add_row("Val1")

    res = ui._render_table_str(table)
    assert "Col1" in res
    assert "Val1" in res

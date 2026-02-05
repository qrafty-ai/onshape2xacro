"""Export progress UI abstractions.

Provides a protocol-based UI layer so the export pipeline can report
progress without coupling to a specific rendering backend.

Two implementations:
  - ``RichExportUI``: interactive Rich console with progress bars.
  - ``NullExportUI``: silent no-op for tests and non-interactive use.
"""

from __future__ import annotations

import os
import time
from contextlib import contextmanager
from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING, Generator, List, Optional, Protocol, runtime_checkable

if TYPE_CHECKING:
    pass


# ---------------------------------------------------------------------------
# Data containers for the export summary
# ---------------------------------------------------------------------------


@dataclass
class LinkMeshStats:
    """Per-link statistics accumulated during export."""

    name: str
    visual_formats: list[str] = field(default_factory=list)
    visual_size_mb: float = 0.0
    compressed: bool = False
    original_size_mb: float = 0.0
    collision_hulls: int = 0
    collision_fallback: bool = False
    has_inertia: bool = False
    mass_kg: float = 0.0


@dataclass
class ExportStats:
    """Aggregate statistics for the entire export."""

    robot_name: str = ""
    output_path: str = ""
    num_links: int = 0
    num_joints: int = 0
    total_mass_kg: float = 0.0
    link_stats: List[LinkMeshStats] = field(default_factory=list)
    # Compression summary
    compressed_count: int = 0
    avg_original_mb: float = 0.0
    avg_compressed_mb: float = 0.0
    # Collision summary
    total_collision_stls: int = 0
    coacd_fallback_count: int = 0
    # Missing meshes
    missing_mesh_links: int = 0
    missing_mesh_parts: int = 0
    missing_meshes_path: str = ""
    # Inertia
    inertia_debug_path: str = ""
    # Warnings
    warnings: List[str] = field(default_factory=list)


# ---------------------------------------------------------------------------
# Protocol – the contract every UI backend must satisfy
# ---------------------------------------------------------------------------

PHASE_LABELS = {
    "load": "Loading CAD data",
    "graph": "Building kinematic graph",
    "model": "Creating robot model",
    "config": "Loading configuration",
    "mesh": "Exporting meshes",
    "serialize": "Serializing to xacro",
}

TOTAL_PHASES = len(PHASE_LABELS)


@runtime_checkable
class ExportUI(Protocol):
    """Protocol that pipeline components call to report progress."""

    # -- phase lifecycle -----------------------------------------------------

    def phase_start(self, key: str, detail: str = "") -> None:
        """Begin a numbered pipeline phase (e.g. ``'load'``, ``'mesh'``)."""
        ...

    def phase_done(self, key: str, detail: str = "") -> None:
        """Mark a phase as completed with an optional summary string."""
        ...

    # -- sub-progress inside the "mesh" phase --------------------------------

    def mesh_progress_start(self, label: str, total: int) -> None:
        """Create a progress bar for a mesh sub-task."""
        ...

    def mesh_progress_advance(self, label: str, description: str = "") -> None:
        """Advance the progress bar by one step."""
        ...

    def mesh_progress_done(self, label: str, detail: str = "") -> None:
        """Finish a mesh sub-task progress bar."""
        ...

    def finish_progress(self) -> None:
        """Stop progress bars after all sub-tasks are done."""
        ...

    # -- summary -------------------------------------------------------------

    def print_summary(self, stats: ExportStats) -> None:
        """Render a final export summary table."""
        ...

    # -- logging integration -------------------------------------------------

    def log(self, message: str, level: str = "info") -> None:
        """Emit a log-level message through the UI (visible or suppressed)."""
        ...


# ---------------------------------------------------------------------------
# fd-level stdout/stderr suppression for native C++ libraries (CoACD)
# ---------------------------------------------------------------------------


@contextmanager
def suppress_c_stdout(
    log_path: Optional[Path] = None,
) -> Generator[None, None, None]:
    """Redirect C-level fd 1 (stdout) to devnull or a file.

    This silences native C++ output from libraries like CoACD that write
    directly to fd 1 and cannot be caught by Python-level redirects.

    Always restores the original fd in a ``finally`` block.

    Args:
        log_path: If given, redirect stdout to this file instead of devnull.
    """
    saved_fd = os.dup(1)
    try:
        if log_path is not None:
            log_path.parent.mkdir(parents=True, exist_ok=True)
            target = os.open(str(log_path), os.O_WRONLY | os.O_CREAT | os.O_TRUNC)
        else:
            target = os.open(os.devnull, os.O_WRONLY)
        os.dup2(target, 1)
        os.close(target)
        yield
    finally:
        os.dup2(saved_fd, 1)
        os.close(saved_fd)


# ---------------------------------------------------------------------------
# NullExportUI – silent implementation for tests
# ---------------------------------------------------------------------------


class NullExportUI:
    """No-op UI that satisfies the ``ExportUI`` protocol."""

    def phase_start(self, key: str, detail: str = "") -> None:
        pass

    def phase_done(self, key: str, detail: str = "") -> None:
        pass

    def mesh_progress_start(self, label: str, total: int) -> None:
        pass

    def mesh_progress_advance(self, label: str, description: str = "") -> None:
        pass

    def mesh_progress_done(self, label: str, detail: str = "") -> None:
        pass

    def finish_progress(self) -> None:
        pass

    def print_summary(self, stats: ExportStats) -> None:
        pass

    def log(self, message: str, level: str = "info") -> None:
        pass


# ---------------------------------------------------------------------------
# RichExportUI – interactive console implementation
# ---------------------------------------------------------------------------


class RichExportUI:
    """Rich-based interactive progress UI for the export pipeline.

    Renders numbered phase labels with checkmarks, nested progress bars
    for mesh export, and a styled summary table at the end.
    """

    def __init__(self) -> None:
        from rich.console import Console

        self.console = Console(stderr=True)
        self._phase_index = 0
        self._phase_timers: dict[str, float] = {}
        self._progress: Optional[object] = None
        self._task_ids: dict[str, object] = {}

    # -- phase lifecycle -----------------------------------------------------

    def phase_start(self, key: str, detail: str = "") -> None:
        self._phase_index += 1
        label = PHASE_LABELS.get(key, key)
        suffix = f"  {detail}" if detail else ""
        self.console.print(
            f"[bold][{self._phase_index}/{TOTAL_PHASES}][/bold] {label}...{suffix}",
            highlight=False,
        )
        self._phase_timers[key] = time.monotonic()

    def phase_done(self, key: str, detail: str = "") -> None:
        elapsed = time.monotonic() - self._phase_timers.get(key, time.monotonic())
        label = PHASE_LABELS.get(key, key)
        suffix = f"  {detail}" if detail else ""
        self.console.print(
            f"  [green]✓[/green] {label}{suffix}  [dim]({elapsed:.1f}s)[/dim]",
            highlight=False,
        )

    # -- sub-progress --------------------------------------------------------

    def mesh_progress_start(self, label: str, total: int) -> None:
        if self._progress is None:
            from rich.progress import (
                BarColumn,
                MofNCompleteColumn,
                Progress,
                SpinnerColumn,
                TextColumn,
                TimeElapsedColumn,
            )

            self._progress = Progress(
                SpinnerColumn(),
                TextColumn("[bold]{task.fields[tree_prefix]}[/bold]"),
                TextColumn("{task.description}"),
                BarColumn(bar_width=30),
                MofNCompleteColumn(),
                TimeElapsedColumn(),
                console=self.console,
            )
            self._progress.start()  # type: ignore[union-attr]

        tree_prefix = "├─"
        task_id = self._progress.add_task(  # type: ignore[union-attr]
            label,
            total=total,
            tree_prefix=tree_prefix,
        )
        self._task_ids[label] = task_id

    def mesh_progress_advance(self, label: str, description: str = "") -> None:
        if self._progress is not None and label in self._task_ids:
            task_id = self._task_ids[label]
            update_kwargs: dict = {"advance": 1}
            if description:
                update_kwargs["description"] = f"{label} ({description})"
            self._progress.update(task_id, **update_kwargs)  # type: ignore[union-attr]

    def mesh_progress_done(self, label: str, detail: str = "") -> None:
        if self._progress is not None and label in self._task_ids:
            task_id = self._task_ids[label]
            if detail:
                self._progress.update(task_id, description=f"{label}  {detail}")  # type: ignore[union-attr]

    def finish_progress(self) -> None:
        """Stop the Rich Progress instance if running."""
        if self._progress is not None:
            if self._task_ids:
                labels = list(self._task_ids.keys())
                last_label = labels[-1]
                self._progress.update(self._task_ids[last_label], tree_prefix="└─")  # type: ignore[union-attr]
            self._progress.stop()  # type: ignore[union-attr]
            self._progress = None
            self._task_ids.clear()

    # -- summary -------------------------------------------------------------

    def print_summary(self, stats: ExportStats) -> None:
        from rich.panel import Panel
        from rich.table import Table

        parts: list[str] = []

        # Header
        header_lines = [
            f"[bold]{stats.robot_name}[/bold]  "
            f"{stats.num_links} links, {stats.num_joints} joints",
        ]
        if stats.total_mass_kg > 0:
            header_lines[0] += f", {stats.total_mass_kg:.2f} kg"
        header_lines.append(f"Output: [dim]{stats.output_path}[/dim]")
        parts.append("\n".join(header_lines))

        # Mesh table (only if there are link stats)
        if stats.link_stats:
            table = Table(
                show_header=True, header_style="bold", box=None, padding=(0, 1)
            )
            table.add_column("Link", style="cyan")
            table.add_column("Visual", justify="right")
            table.add_column("Collision", justify="right")
            table.add_column("Mass", justify="right")

            for ls in stats.link_stats:
                vis_text = f"{ls.visual_size_mb:.1f} MB"
                if ls.compressed:
                    vis_text += " [yellow]↓[/yellow]"
                col_text = f"{ls.collision_hulls} hulls" if ls.collision_hulls else "-"
                if ls.collision_fallback:
                    col_text += " [yellow]!fb[/yellow]"
                mass_text = f"{ls.mass_kg:.3f} kg" if ls.has_inertia else "[dim]-[/dim]"
                table.add_row(ls.name, vis_text, col_text, mass_text)

            parts.append(self._render_table_str(table))

        # Compression note
        if stats.compressed_count > 0:
            parts.append(
                f"[yellow]{stats.compressed_count} meshes compressed[/yellow] "
                f"(avg {stats.avg_original_mb:.1f} -> {stats.avg_compressed_mb:.1f} MB)"
            )

        # Collision note
        if stats.total_collision_stls > 0:
            note = f"{stats.total_collision_stls} collision STL files written"
            if stats.coacd_fallback_count > 0:
                note += f"  [yellow]({stats.coacd_fallback_count} links fell back to convex hull)[/yellow]"
            parts.append(note)

        # Warnings / next actions
        if stats.missing_mesh_links > 0:
            parts.append(
                f"[yellow]Warning:[/yellow] {stats.missing_mesh_parts} parts missing across "
                f"{stats.missing_mesh_links} links.  See {stats.missing_meshes_path}"
            )
        if stats.inertia_debug_path:
            parts.append(f"Inertia debug table: [dim]{stats.inertia_debug_path}[/dim]")
        for w in stats.warnings:
            parts.append(f"[yellow]Warning:[/yellow] {w}")

        panel = Panel(
            "\n".join(parts),
            title="[bold green]Export Complete[/bold green]",
            expand=False,
        )
        self.console.print(panel)

    def _render_table_str(self, table: object) -> str:
        """Render a Rich Table to a plain string for embedding in a Panel."""
        from io import StringIO

        from rich.console import Console as _Console

        buf = StringIO()
        c = _Console(file=buf, force_terminal=True, width=100)
        c.print(table)
        return buf.getvalue().rstrip()

    # -- logging integration -------------------------------------------------

    def log(self, message: str, level: str = "info") -> None:
        style = {"warning": "yellow", "error": "red bold"}.get(level, "dim")
        self.console.print(f"  [{style}]{message}[/{style}]", highlight=False)

"""Tests for inertia report."""

from onshape2xacro.inertia.report import InertiaReport


def test_report_add_warning():
    report = InertiaReport()
    report.add_warning("link1", "part_a", "No mass or material assigned")

    assert len(report.warnings) == 1
    assert report.warnings[0].link_name == "link1"
    assert report.warnings[0].part_name == "part_a"


def test_report_summary_with_warnings():
    report = InertiaReport()
    report.add_warning("link1", "part_a", "No mass or material")
    report.add_warning("link1", "part_b", "No mass or material")
    report.add_warning("link2", "part_c", "Unknown material 'custom'")

    summary = report.get_summary()
    assert "3" in summary  # 3 warnings
    assert "link1" in summary
    assert "link2" in summary


def test_report_no_warnings():
    report = InertiaReport()
    summary = report.get_summary()
    assert "no warnings" in summary.lower() or "all parts" in summary.lower()

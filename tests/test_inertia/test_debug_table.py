"""Tests for inertia debug table generation."""

from onshape2xacro.inertia.report import InertiaReport, PartDebugInfo
from onshape2xacro.inertia.types import InertialProperties


def test_part_debug_info_creation():
    info = PartDebugInfo(
        part_id="solid_0",
        bom_match="link_base",
        match_type="exact",
        mass_source="BOM Mass",
        material="aluminum",
        volume_cm3=312.5,
        mass_kg=0.852,
        warnings=[],
    )
    assert info.part_id == "solid_0"
    assert info.mass_source == "BOM Mass"


def test_generate_debug_table_empty():
    report = InertiaReport()
    table = report.generate_debug_table()
    assert "No part debug information" in table


def test_generate_debug_table_with_parts():
    report = InertiaReport()

    report.link_properties["link_base"] = InertialProperties(
        mass=0.852, com=(0.1, 0.0, 0.0), ixx=0.001, iyy=0.001, izz=0.001
    )

    parts = [
        PartDebugInfo(
            part_id="solid_0",
            bom_match="link_base",
            match_type="exact",
            mass_source="BOM Mass",
            material=None,
            volume_cm3=312.5,
            mass_kg=0.852,
            warnings=[],
        )
    ]
    report.add_link_parts("link_base", parts)

    table = report.generate_debug_table()

    assert "## Link: link_base" in table
    assert "solid_0" in table
    assert "BOM Mass" in table
    assert "0.852" in table or "0.8520" in table


def test_debug_table_with_warnings():
    report = InertiaReport()

    parts = [
        PartDebugInfo(
            part_id="solid_0",
            bom_match=None,
            match_type="none",
            mass_source="Default Density",
            material=None,
            volume_cm3=100.0,
            mass_kg=0.1,
            warnings=["Not found in BOM"],
        )
    ]
    report.add_link_parts("link_test", parts)

    table = report.generate_debug_table()

    assert "⚠️" in table
    assert "Not found in BOM" in table

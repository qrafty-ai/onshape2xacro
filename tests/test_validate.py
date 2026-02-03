import sys
import pytest
from pathlib import Path
from unittest.mock import patch
from onshape2xacro.validate import validate_xacro_file, main


@pytest.fixture
def valid_xacro(tmp_path):
    p = tmp_path / "test.xacro"
    content = """<?xml version="1.0"?>
<robot name="test_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
    <link name="base_link"/>
    <xacro:macro name="test_macro" params="p1 p2"/>
    <xacro:include filename="other.xacro"/>
</robot>
"""
    p.write_text(content)
    return p


@pytest.fixture
def invalid_xml(tmp_path):
    p = tmp_path / "bad.xml"
    p.write_text("<robot><link></robot>")
    return p


def test_validate_valid_file(valid_xacro, capsys):
    assert validate_xacro_file(valid_xacro) is True
    out, _ = capsys.readouterr()
    assert "✓ File is well-formed XML" in out
    assert "✓ Root element: robot" in out
    assert "Links: 1" in out


def test_validate_invalid_file(invalid_xml, capsys):
    assert validate_xacro_file(invalid_xml) is False
    out, _ = capsys.readouterr()
    assert "✗ XML Parse Error" in out


def test_validate_missing_file(capsys):
    assert validate_xacro_file(Path("missing.xacro")) is False
    out, _ = capsys.readouterr()
    assert "✗ File not found" in out


def test_main_success(valid_xacro):
    with patch.object(sys, "argv", ["validate-xacro", str(valid_xacro)]):
        with pytest.raises(SystemExit) as e:
            main()
        assert e.value.code == 0


def test_main_failure(invalid_xml):
    with patch.object(sys, "argv", ["validate-xacro", str(invalid_xml)]):
        with pytest.raises(SystemExit) as e:
            main()
        assert e.value.code == 1


def test_main_no_args(capsys):
    with patch.object(sys, "argv", ["validate-xacro"]):
        with pytest.raises(SystemExit) as e:
            main()
        assert e.value.code == 1
        out, _ = capsys.readouterr()
        assert "Usage:" in out

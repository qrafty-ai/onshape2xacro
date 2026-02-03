import pytest
from onshape2xacro.naming import sanitize_name


@pytest.mark.parametrize(
    "input_name, expected",
    [
        ("Link Name", "link_name"),
        ("Link-Name", "link_name"),
        ("Link@Name!", "linkname"),
        ("123Link", "_123link"),
        ("   ", "_"),
        ("", "_"),
        ("___link___", "_link_"),
        ("Link   Name", "link_name"),
        ("LINK_NAME", "link_name"),
    ],
)
def test_sanitize_name(input_name, expected):
    assert sanitize_name(input_name) == expected

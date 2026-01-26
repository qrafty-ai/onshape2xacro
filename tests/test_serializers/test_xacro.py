import pytest
from onshape2xacro.serializers import sanitize_name, XacroSerializer


def test_sanitize_name():
    assert sanitize_name("My Part (v2.1)") == "my_part_v21"
    assert sanitize_name("Link 1") == "link_1"
    assert sanitize_name("Joint-A") == "joint_a"
    assert sanitize_name("123name") == "_123name"  # Should start with letter/underscore


def test_xacro_serializer_basic():
    # We'll need a way to mock Robot/Link/Joint without full library
    # For now, let's focus on the sanitization and structure tests
    pass


@pytest.mark.parametrize(
    "input_name,expected",
    [
        ("Space Part", "space_part"),
        ("Special!@#Char", "specialchar"),
        ("_already_safe", "_already_safe"),
        ("Capitals", "capitals"),
    ],
)
def test_name_sanitization_variants(input_name, expected):
    assert sanitize_name(input_name) == expected

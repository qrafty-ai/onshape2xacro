import re


def sanitize_name(name: str) -> str:
    """Sanitize Onshape names to valid ROS identifiers."""
    s = name.lower()
    s = s.replace(" ", "_").replace("-", "_")
    s = re.sub(r"[^a-z0-9_]", "", s)
    s = re.sub(r"_+", "_", s)
    if s and s[0].isdigit():
        s = "_" + s
    if not s:
        s = "_"
    return s

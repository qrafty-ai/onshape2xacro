import re


def sanitize_name(name: str) -> str:
    """Sanitize Onshape names to valid ROS identifiers."""
    s = name.lower()
    s = s.replace(" ", "_").replace("-", "_")
    # Use \w to allow Unicode characters (letters, numbers, underscore)
    # This prevents names like "机器人" from being stripped to empty/underscore
    s = re.sub(r"[^\w]", "", s)
    s = re.sub(r"_+", "_", s)
    if s and s[0].isdigit():
        s = "_" + s
    if not s:
        s = "_"
    return s

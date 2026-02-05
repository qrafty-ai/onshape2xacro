import pytest
import shutil
import atexit
from pathlib import Path


def _clean():
    artifacts = [
        "MagicMock",
        "output",
        "ORT.yaml",
        "ORT.log",
        "export.log",
        "file_output",
        "yaml_output",
        "override_output",
        "output_test",
        "openarm_asm",
        "waist",
        "teaarm_xacro",
    ]

    root = Path(__file__).parent.parent
    for artifact in artifacts:
        path = root / artifact
        if path.exists():
            if path.is_dir():
                shutil.rmtree(path)
            else:
                path.unlink()


@pytest.fixture(autouse=True)
def cleanup_artifacts():
    yield
    _clean()


@pytest.fixture(autouse=True, scope="session")
def session_cleanup():
    yield
    _clean()


atexit.register(_clean)

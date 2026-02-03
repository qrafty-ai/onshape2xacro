from pathlib import Path
from unittest.mock import MagicMock
from onshape2xacro.mate_values import (
    fetch_mate_values,
    save_mate_values,
    load_mate_values,
)


def test_fetch_mate_values_success():
    client = MagicMock()
    response = MagicMock()
    response.status_code = 200
    response.json.return_value = {
        "mateValues": [
            {"featureId": "mate1", "mateName": "Joint 1", "rotationX": 0.5},
            {"featureId": "mate2", "mateName": "Joint 2", "translationX": 0.1},
        ]
    }
    client.request.return_value = response

    values = fetch_mate_values(client, "did", "w", "wvmid", "eid")

    assert len(values) == 2
    assert values["mate1"]["featureId"] == "mate1"
    assert values["mate2"]["mateName"] == "Joint 2"


def test_fetch_mate_values_failure():
    client = MagicMock()
    response = MagicMock()
    response.status_code = 404
    response.text = "Not Found"
    client.request.return_value = response

    values = fetch_mate_values(client, "did", "w", "wvmid", "eid")
    assert values == {}


def test_fetch_mate_values_exception():
    client = MagicMock()
    client.request.side_effect = Exception("Network error")

    values = fetch_mate_values(client, "did", "w", "wvmid", "eid")
    assert values == {}


def test_save_and_load_mate_values(tmp_path):
    file_path = tmp_path / "mates.json"
    data = {"mate1": {"value": 0.5}, "mate2": {"value": 0.1}}

    save_mate_values(file_path, data)
    assert file_path.exists()

    loaded = load_mate_values(file_path)
    assert loaded == data


def test_load_mate_values_missing():
    assert load_mate_values(Path("non_existent.json")) == {}

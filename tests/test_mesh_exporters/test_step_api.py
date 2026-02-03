import pytest
from unittest.mock import MagicMock, patch
from pathlib import Path
import io
import zipfile
from onshape2xacro.mesh_exporters.step import StepMeshExporter


@pytest.fixture
def mock_client():
    return MagicMock()


@pytest.fixture
def mock_cad():
    cad = MagicMock()
    cad.document_id = "did"
    cad.wtype = "w"
    cad.workspace_id = "wid"
    cad.element_id = "eid"
    return cad


def test_export_step_success(mock_client, mock_cad, tmp_path):
    exporter = StepMeshExporter(mock_client, mock_cad)
    output_path = tmp_path / "output.step"

    # Mock Translation POST
    mock_post_resp = MagicMock()
    mock_post_resp.json.return_value = {"id": "trans_id"}

    # Mock Status GET (Poling)
    mock_status_running = MagicMock()
    mock_status_running.json.return_value = {"requestState": "ACTIVE"}
    mock_status_done = MagicMock()
    mock_status_done.json.return_value = {
        "requestState": "DONE",
        "resultExternalDataIds": ["file_id"],
    }

    # Mock Download GET
    mock_download = MagicMock()
    mock_download.content = (
        b"ISO-10303-21; HEADER; ENDSEC; DATA; ENDSEC; END-ISO-10303-21;"
    )

    mock_client.request.side_effect = [
        mock_post_resp,  # POST translation
        mock_status_running,  # GET status 1
        mock_status_done,  # GET status 2
        mock_download,  # GET file
    ]

    with patch("time.sleep"):  # Skip sleep
        result = exporter.export_step(output_path)

    assert result == output_path
    assert output_path.read_bytes() == mock_download.content
    assert mock_client.request.call_count == 4


def test_export_step_zip_handling(mock_client, mock_cad, tmp_path):
    exporter = StepMeshExporter(mock_client, mock_cad)
    output_path = tmp_path / "output.step"

    # Create valid zip content
    zip_buffer = io.BytesIO()
    with zipfile.ZipFile(zip_buffer, "w") as zf:
        zf.writestr("model.step", b"ISO-10303-21; HEADER; ENDSEC;")
    zip_content = zip_buffer.getvalue()

    # Mock responses
    mock_post = MagicMock()
    mock_post.json.return_value = {"id": "trans_id"}

    mock_status = MagicMock()
    mock_status.json.return_value = {
        "requestState": "DONE",
        "resultExternalDataIds": ["file_id"],
    }

    mock_download = MagicMock()
    mock_download.content = zip_content

    mock_client.request.side_effect = [mock_post, mock_status, mock_download]

    with patch("time.sleep"):
        exporter.export_step(output_path)

    assert output_path.read_bytes() == b"ISO-10303-21; HEADER; ENDSEC;"


def test_export_step_fallback_download(mock_client, mock_cad, tmp_path):
    exporter = StepMeshExporter(mock_client, mock_cad)
    output_path = tmp_path / "output.step"

    mock_post = MagicMock()
    mock_post.json.return_value = {"id": "trans_id"}

    mock_status = MagicMock()
    mock_status.json.return_value = {
        "requestState": "DONE",
        "resultExternalDataIds": ["file_id_xml"],
    }

    # First download returns XML (e.g. Parasolid manifest)
    mock_download_xml = MagicMock()
    mock_download_xml.content = b"<?xml version='1.0'?>"

    # Fallback download returns STEP
    mock_download_fallback = MagicMock()
    mock_download_fallback.content = b"ISO-10303-21;"

    mock_client.request.side_effect = [
        mock_post,
        mock_status,
        mock_download_xml,  # Primary download failed to match STEP
        mock_download_fallback,  # Fallback URL
    ]

    with patch("time.sleep"):
        exporter.export_step(output_path)

    assert output_path.read_bytes() == b"ISO-10303-21;"


def test_export_step_no_client(mock_cad):
    exporter = StepMeshExporter(None, mock_cad)
    with pytest.raises(RuntimeError, match="Cannot export STEP without Onshape client"):
        exporter.export_step(Path("out.step"))


def test_export_step_missing_ids(mock_client):
    cad = MagicMock()
    cad.wtype = None
    cad.wvm = None
    exporter = StepMeshExporter(mock_client, cad)
    with pytest.raises(AttributeError, match="missing workspace identifiers"):
        exporter.export_step(Path("out.step"))


def test_export_step_failed_translation(mock_client, mock_cad, tmp_path):
    exporter = StepMeshExporter(mock_client, mock_cad)

    mock_client.request.side_effect = [
        MagicMock(json=lambda: {"id": "tid"}),
        MagicMock(json=lambda: {"requestState": "FAILED"}),
    ]

    with (
        patch("time.sleep"),
        pytest.raises(RuntimeError, match="STEP translation failed"),
    ):
        exporter.export_step(tmp_path / "out.step")


def test_export_step_no_external_ids(mock_client, mock_cad, tmp_path):
    exporter = StepMeshExporter(mock_client, mock_cad)

    mock_client.request.side_effect = [
        MagicMock(json=lambda: {"id": "tid"}),
        MagicMock(json=lambda: {"requestState": "DONE", "resultExternalDataIds": []}),
    ]

    with patch("time.sleep"), pytest.raises(RuntimeError, match="No external data ids"):
        exporter.export_step(tmp_path / "out.step")

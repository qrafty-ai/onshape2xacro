import json
from unittest.mock import MagicMock


def test_fetch_cad_command(monkeypatch, tmp_path):
    from onshape2xacro.cli import main
    import onshape2xacro.pipeline as pipeline

    output_file = tmp_path / "cad.json"
    url = "https://cad.onshape.com/documents/123/w/456/e/789"

    # Mock credentials
    monkeypatch.setattr(pipeline, "get_credentials", lambda: ("access", "secret"))

    # Create a mock CAD object
    mock_cad = MagicMock()
    mock_cad.document_id = "123"
    mock_cad.element_id = "789"
    mock_cad.wtype = "w"
    mock_cad.workspace_id = "456"
    mock_cad.document_microversion = "mv1"
    mock_cad.name = "test_robot"
    mock_cad.max_depth = 5
    mock_cad.keys_by_id = {}
    mock_cad.keys_by_name = {}
    mock_cad.instances = {}
    mock_cad.occurrences = {}
    mock_cad.subassemblies = {}
    mock_cad.mates = {}
    mock_cad.patterns = {}
    mock_cad.parts = {}

    # Mock CAD.from_url
    # pipeline.py imports CAD from onshape_robotics_toolkit
    monkeypatch.setattr(
        "onshape2xacro.pipeline.CAD.from_url", lambda *args, **kwargs: mock_cad
    )

    # Set CLI arguments
    test_args = ["onshape2xacro", "fetch-cad", url, "--output", str(output_file)]
    monkeypatch.setattr("sys.argv", test_args)

    # Run main
    main()

    # Verify output
    assert output_file.exists()
    with open(output_file) as f:
        data = json.load(f)

    assert data["document_id"] == "123"
    assert data["element_id"] == "789"
    assert data["name"] == "test_robot"
    assert data["workspace_id"] == "456"

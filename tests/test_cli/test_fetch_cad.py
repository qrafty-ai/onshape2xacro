import pickle


# Define MockCAD at module level so it can be pickled
class MockCAD:
    def __init__(self):
        self.document_id = "123"
        self.element_id = "789"
        self.wtype = "w"
        self.workspace_id = "456"
        self.document_microversion = "mv1"
        self.name = "test_robot"
        self.max_depth = 5
        self.keys_by_id = {}
        self.keys_by_name = {}
        self.instances = {}
        self.occurrences = {}
        self.subassemblies = {}
        self.mates = {}
        self.patterns = {}
        self.parts = {}


def test_fetch_cad_command(monkeypatch, tmp_path):
    from onshape2xacro.cli import main
    import onshape2xacro.pipeline as pipeline

    output_file = tmp_path / "cad.pkl"
    url = "https://cad.onshape.com/documents/123/w/456/e/789"

    # Mock credentials
    monkeypatch.setattr(pipeline, "get_credentials", lambda: ("access", "secret"))

    mock_cad = MockCAD()

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
    with open(output_file, "rb") as f:
        data = pickle.load(f)

    assert data.document_id == "123"
    assert data.element_id == "789"
    assert data.name == "test_robot"
    assert data.workspace_id == "456"

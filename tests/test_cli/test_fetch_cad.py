from unittest.mock import MagicMock
import pickle


# Define MockCAD at module level so it can be pickled
class MockCAD:
    def __init__(self):
        self.document_id = "000000000000000000000123"
        self.element_id = "000000000000000000000789"
        self.wtype = "w"
        self.workspace_id = "000000000000000000000456"
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
    url = "https://cad.onshape.com/documents/000000000000000000000123/w/000000000000000000000456/e/000000000000000000000789"

    # Mock credentials
    monkeypatch.setattr(pipeline, "get_credentials", lambda: ("access", "secret"))

    mock_cad = MockCAD()

    # Mock CAD.from_url
    # pipeline.py imports OptimizedCAD from onshape2xacro.optimized_cad
    monkeypatch.setattr(
        "onshape2xacro.optimized_cad.OptimizedCAD.from_url",
        lambda *args, **kwargs: mock_cad,
    )

    # Mock StepMeshExporter to avoid API calls
    class MockExporter:
        def __init__(self, client, cad):
            pass

        def export_step(self, path):
            # Create dummy file
            with open(path, "w") as f:
                f.write("STEP DATA")

    monkeypatch.setattr(
        "onshape2xacro.mesh_exporters.step.StepMeshExporter", MockExporter
    )

    # Mock KinematicGraph and CondensedRobot
    mock_graph = MagicMock()
    monkeypatch.setattr(
        "onshape2xacro.pipeline.KinematicGraph.from_cad", lambda *args: mock_graph
    )
    mock_robot = MagicMock()
    mock_robot.nodes = ["link1", "link2"]
    monkeypatch.setattr(
        "onshape2xacro.pipeline.CondensedRobot.from_graph",
        lambda *args, **kwargs: mock_robot,
    )

    # Set CLI arguments
    test_args = ["onshape2xacro", "fetch-cad", url, "--output", str(output_file)]
    monkeypatch.setattr("sys.argv", test_args)

    # Run main
    main()

    # Verify output
    # output_file is a directory in this test context due to run_fetch_cad behavior
    assert output_file.exists()
    assert output_file.is_dir()

    # Check cad.pickle inside
    with open(output_file / "cad.pickle", "rb") as f:
        data = pickle.load(f)

    assert data.document_id == "000000000000000000000123"
    assert data.element_id == "000000000000000000000789"
    assert data.name == "test_robot"
    assert data.workspace_id == "000000000000000000000456"

    # Verify configuration.yaml saved
    assert (output_file / "configuration.yaml").exists()
    import yaml

    with open(output_file / "configuration.yaml", "r") as f:
        config_data = yaml.safe_load(f)

    assert "mate_values" in config_data
    assert "link_names" in config_data
    assert config_data["link_names"] == {"link1": "link1", "link2": "link2"}
    assert config_data["export"]["name"] == "test_robot"

    # Verify mate_values.json is NOT saved
    assert not (output_file / "mate_values.json").exists()

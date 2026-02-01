from onshape2xacro.cli import ExportConfig
from onshape2xacro import pipeline


def test_run_export_uses_condensed_robot(monkeypatch, tmp_path):
    class DummyCad:
        name = "dummy_cad"
        document_id = "doc123"
        wtype = "w"
        workspace_id = "ws456"
        element_id = "elem789"

    class DummyClient:
        pass

    dummy_graph = object()
    called = {}

    def fake_get_client_and_cad(url, max_depth):
        called["client"] = DummyClient()
        called["cad"] = DummyCad()
        return called["client"], called["cad"]

    def fake_from_cad(cad):
        called["graph_cad"] = cad
        return dummy_graph

    def fake_from_graph(graph, cad, name, **kwargs):
        called["graph"] = graph
        called["cad_arg"] = cad
        called["name"] = name

        class DummyRobot:
            pass

        return DummyRobot()

    class DummySerializer:
        def save(self, robot, output, download_assets, **kwargs):
            called["saved_robot"] = robot

    monkeypatch.setattr(pipeline, "_get_client_and_cad", fake_get_client_and_cad)
    monkeypatch.setattr(
        pipeline.KinematicGraph, "from_cad", staticmethod(fake_from_cad)
    )
    monkeypatch.setattr(
        pipeline.CondensedRobot, "from_graph", staticmethod(fake_from_graph)
    )
    monkeypatch.setattr(pipeline.ConfigOverride, "load", staticmethod(lambda _: None))
    monkeypatch.setattr(pipeline, "XacroSerializer", DummySerializer)

    config = ExportConfig(
        url="https://cad.onshape.com/documents/dummy/w/123/e/456",
        output=tmp_path,
        name="robot",
        max_depth=1,
    )
    pipeline.run_export(config)

    assert called["graph"] is dummy_graph
    assert called["cad_arg"] is called["cad"]
    assert called["name"] == "robot"


def test_run_export_prefetched_uses_client_if_credentials_available(
    monkeypatch, tmp_path
):
    class DummyCad:
        name = "dummy_cad"

    class DummyClient:
        def __init__(self, **kwargs):
            self.kwargs = kwargs

    class DummySerializer:
        def save(self, robot, output, download_assets, **kwargs):
            assert robot.client is not None

    def fake_get_credentials():
        return "key", "secret"

    def fake_from_cad(cad):
        return object()

    def fake_from_graph(graph, cad, name, **kwargs):
        class DummyRobot:
            pass

        return DummyRobot()

    import pickle
    from types import SimpleNamespace

    cad_path = tmp_path / "cad.pickle"
    with open(cad_path, "wb") as f:
        pickle.dump(SimpleNamespace(name="dummy_cad"), f)

    monkeypatch.setattr(pipeline, "get_credentials", fake_get_credentials)
    monkeypatch.setattr(pipeline, "Client", DummyClient)
    monkeypatch.setattr(
        pipeline.KinematicGraph, "from_cad", staticmethod(fake_from_cad)
    )
    monkeypatch.setattr(
        pipeline.CondensedRobot, "from_graph", staticmethod(fake_from_graph)
    )
    monkeypatch.setattr(pipeline.ConfigOverride, "load", staticmethod(lambda _: None))
    monkeypatch.setattr(pipeline, "XacroSerializer", DummySerializer)

    config = ExportConfig(
        url=str(tmp_path),
        output=tmp_path / "out",
        name="robot",
        max_depth=1,
    )
    pipeline.run_export(config)

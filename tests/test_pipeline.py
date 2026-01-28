from onshape2xacro.cli import ExportConfig
from onshape2xacro import pipeline


def test_run_export_uses_condensed_robot(monkeypatch, tmp_path):
    class DummyCad:
        name = "dummy_cad"

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

    def fake_from_graph(graph, cad, name):
        called["graph"] = graph
        called["cad_arg"] = cad
        called["name"] = name

        class DummyRobot:
            pass

        return DummyRobot()

    class DummySerializer:
        def save(self, robot, output, download_assets, config):
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

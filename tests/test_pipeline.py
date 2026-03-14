import pickle
import yaml
from types import SimpleNamespace
from onshape2xacro.cli import ExportConfig
from onshape2xacro import pipeline


class DummyCad:
    name = "dummy_cad"
    document_id = "doc123"
    wtype = "w"
    workspace_id = "ws456"
    element_id = "elem789"


def test_run_export_uses_condensed_robot(monkeypatch, tmp_path):
    class DummyClient:
        pass

    dummy_graph = object()
    called = {}

    cad_path = tmp_path / "cad.pickle"
    with open(cad_path, "wb") as f:
        pickle.dump(DummyCad(), f)

    config_path = tmp_path / "configuration.yaml"
    with open(config_path, "w") as f:
        yaml.dump(
            {
                "export": {
                    "name": "robot",
                    "output": str(tmp_path),
                    "visual_mesh_format": "stl",
                },
                "mate_values": {},
                "link_names": {},
            },
            f,
        )

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
        called["cad"] = cad  # Ensure cad is in called

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
        path=tmp_path,
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

    cad_path = tmp_path / "cad.pickle"
    with open(cad_path, "wb") as f:
        pickle.dump(SimpleNamespace(name="dummy_cad"), f)

    config_path = tmp_path / "configuration.yaml"
    with open(config_path, "w") as f:
        yaml.dump(
            {
                "export": {
                    "name": "robot",
                    "output": str(tmp_path),
                    "visual_mesh_format": "stl",
                },
                "mate_values": {},
                "link_names": {},
            },
            f,
        )

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
        path=tmp_path,
        output=tmp_path / "out",
        name="robot",
        max_depth=1,
    )
    pipeline.run_export(config)


def test_pipeline_credentials_error(monkeypatch):
    import os
    import pytest
    from onshape2xacro.pipeline import _get_client_and_cad

    monkeypatch.setattr("onshape2xacro.pipeline.get_credentials", lambda: (None, None))
    monkeypatch.setattr(os, "environ", {})

    with pytest.raises(ValueError, match="Onshape credentials not found"):
        _get_client_and_cad("http://url", 5)


def test_pipeline_configuration_name_resolves(monkeypatch):
    from onshape2xacro.pipeline import _get_client_and_cad

    monkeypatch.setattr(
        "onshape2xacro.pipeline.get_credentials", lambda: ("access", "secret")
    )

    class DummyResponse:
        status_code = 200

        def json(self):
            return {
                "configurationParameters": [
                    {
                        "message": {
                            "parameterId": "List_EqbreqzJQ2Vu6y",
                            "parameterName": "Configuration",
                            "options": [
                                {
                                    "message": {
                                        "option": "Default",
                                        "optionName": "Default",
                                    }
                                },
                                {
                                    "message": {
                                        "option": "original_gripper",
                                        "optionName": "original gripper",
                                    }
                                },
                            ],
                        }
                    }
                ]
            }

    class DummyClient:
        def __init__(self, **kwargs):
            self.kwargs = kwargs

        def request(self, method, path, **kwargs):
            return DummyResponse()

    captured = {}

    def _mock_from_url(url, **kwargs):
        captured["from_url_kwargs"] = kwargs
        return SimpleNamespace(name="dummy")

    monkeypatch.setattr("onshape2xacro.pipeline.OptimizedClient", DummyClient)
    monkeypatch.setattr("onshape2xacro.pipeline.OptimizedCAD.from_url", _mock_from_url)

    _, _, resolved = _get_client_and_cad(
        "https://cad.onshape.com/documents/a0ef8c2702b6aff0fdcf6530/w/ad2b0ef097cbce69232e9a71/e/ee09e09777738cd7d63862b2",
        5,
        configuration="original gripper",
    )

    assert resolved == "List_EqbreqzJQ2Vu6y=original_gripper"
    assert (
        captured["from_url_kwargs"]["configuration"]
        == "List_EqbreqzJQ2Vu6y=original_gripper"
    )


def test_pipeline_passes_empty_configuration_through(monkeypatch):
    from onshape2xacro.pipeline import _get_client_and_cad

    monkeypatch.setattr(
        "onshape2xacro.pipeline.get_credentials", lambda: ("access", "secret")
    )
    monkeypatch.setattr(
        "onshape2xacro.pipeline._resolve_configuration_arg",
        lambda client, url, configuration: "",
    )

    class DummyClient:
        def __init__(self, **kwargs):
            self.kwargs = kwargs

    captured = {}

    def _mock_from_url(url, **kwargs):
        captured["from_url_kwargs"] = kwargs
        return SimpleNamespace(name="dummy")

    monkeypatch.setattr("onshape2xacro.pipeline.OptimizedClient", DummyClient)
    monkeypatch.setattr("onshape2xacro.pipeline.OptimizedCAD.from_url", _mock_from_url)

    _, _, resolved = _get_client_and_cad(
        "https://cad.onshape.com/documents/d/w/w/e/e", 5
    )

    assert resolved == ""
    assert "configuration" in captured["from_url_kwargs"]
    assert captured["from_url_kwargs"]["configuration"] == ""


def test_pipeline_configuration_unknown_value_error(monkeypatch):
    import pytest
    from onshape2xacro.pipeline import _get_client_and_cad

    monkeypatch.setattr(
        "onshape2xacro.pipeline.get_credentials", lambda: ("access", "secret")
    )

    class DummyResponse:
        status_code = 200

        def json(self):
            return {
                "configurationParameters": [
                    {
                        "message": {
                            "parameterId": "List_EqbreqzJQ2Vu6y",
                            "parameterName": "Configuration",
                            "options": [
                                {
                                    "message": {
                                        "option": "Default",
                                        "optionName": "Default",
                                    }
                                },
                                {
                                    "message": {
                                        "option": "original_gripper",
                                        "optionName": "original gripper",
                                    }
                                },
                            ],
                        }
                    }
                ]
            }

    class DummyClient:
        def __init__(self, **kwargs):
            self.kwargs = kwargs

        def request(self, method, path, **kwargs):
            return DummyResponse()

    monkeypatch.setattr("onshape2xacro.pipeline.OptimizedClient", DummyClient)

    with pytest.raises(ValueError, match="Unknown configuration value"):
        _get_client_and_cad(
            "https://cad.onshape.com/documents/a0ef8c2702b6aff0fdcf6530/w/ad2b0ef097cbce69232e9a71/e/ee09e09777738cd7d63862b2",
            5,
            configuration="not a real option",
        )

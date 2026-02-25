import pytest
import asyncio
from typing import cast, Any
from unittest.mock import MagicMock, patch
from pydantic import ValidationError

from onshape_robotics_toolkit import Client
from onshape_robotics_toolkit.models.assembly import (
    RootAssembly,
    SubAssembly,
    MateFeatureData,
    Part,
)
from onshape_robotics_toolkit.parse import PathKey

from onshape2xacro.optimized_cad import OptimizedCAD, OptimizedClient

# --- Fixtures ---


@pytest.fixture
def mock_client():
    client = MagicMock(spec=OptimizedClient)
    # Methods called via asyncio.to_thread should be synchronous mocks
    client.get_root_assembly = MagicMock()
    client.get_mass_property = MagicMock()
    client.get_assembly_mass_properties = MagicMock()
    # Synchronous methods
    client.get_assembly = MagicMock()
    client.get_features = MagicMock()
    client.get_document_metadata = MagicMock()
    return client


@pytest.fixture
def mock_cad(mock_client):
    # Bypass __init__ to avoid real API calls or complex setup
    cad = MagicMock(spec=OptimizedCAD)
    cad.client = mock_client
    cad.document_id = "d1"
    cad.workspace_id = "w1"
    cad.element_id = "e1"
    cad.wtype = "w"
    cad.document_microversion = "m1"

    # Initialize empty containers
    cad.subassemblies = {}
    cad.instances = {}
    cad.keys_by_id = {}
    cad.occurrences = {}
    cad.mates = {}
    cad.parts = {}

    # Bind methods to the real implementation we want to test
    # We use __get__ to bind the method to the instance
    cad.fetch_occurrences_for_subassemblies = (
        OptimizedCAD.fetch_occurrences_for_subassemblies.__get__(cad, OptimizedCAD)
    )
    cad.fetch_mate_limits = OptimizedCAD.fetch_mate_limits.__get__(cad, OptimizedCAD)
    cad.fetch_mass_properties_for_parts = (
        OptimizedCAD.fetch_mass_properties_for_parts.__get__(cad, OptimizedCAD)
    )

    return cad


# --- Tests ---


def test_from_url_success(mock_client):
    url = "https://cad.onshape.com/documents/000000000000000000000001/w/000000000000000000000002/e/000000000000000000000003"

    # Mock return of client.get_assembly
    mock_assembly = MagicMock()
    mock_client.get_assembly.return_value = mock_assembly

    # Mock OptimizedCAD.from_assembly to return a dummy object
    with patch(
        "onshape2xacro.optimized_cad.OptimizedCAD.from_assembly"
    ) as mock_from_asm:
        result = OptimizedCAD.from_url(url, client=mock_client)

        # Verify call to client
        mock_client.get_assembly.assert_called_once()
        args, kwargs = mock_client.get_assembly.call_args
        # did, wtype, wid, eid are positional args
        assert args[0] == "000000000000000000000001"  # did
        assert args[1] == "w"  # wtype
        assert args[2] == "000000000000000000000002"  # wid
        assert args[3] == "000000000000000000000003"  # eid

        # Verify instantiation
        mock_from_asm.assert_called_once()
        assert result == mock_from_asm.return_value


def test_from_url_missing_client():
    url = "https://cad.onshape.com/documents/000000000000000000000001/w/000000000000000000000002/e/000000000000000000000003"
    with pytest.raises(ValueError, match="client must be provided"):
        OptimizedCAD.from_url(url, client=cast(Client, cast(object, None)))


def test_get_assembly_applies_mate_connector_cs_fallback(monkeypatch):
    client = object.__new__(OptimizedClient)
    client._url = "https://cad.onshape.com"

    payload: Any = {
        "rootAssembly": {
            "instances": [],
            "patterns": [],
            "occurrences": [],
            "features": [
                {
                    "id": "feature-1",
                    "suppressed": False,
                    "featureType": "mateConnector",
                    "featureData": {
                        "occurrence": ["occ-1"],
                        "name": "frame_test",
                    },
                }
            ],
            "fullConfiguration": "default",
            "configuration": "default",
            "documentId": "000000000000000000000001",
            "elementId": "000000000000000000000002",
            "documentMicroversion": "000000000000000000000003",
        },
        "subAssemblies": [],
        "parts": [],
        "partStudioFeatures": [],
    }

    response = MagicMock(status_code=200)
    response.json.return_value = payload
    client.request = MagicMock(return_value=response)

    monkeypatch.setattr(
        "onshape_robotics_toolkit.utilities.helpers.clean_json_numerics",
        lambda data, threshold, decimals: data,
    )
    monkeypatch.setattr(
        "onshape_robotics_toolkit.config.record_document_config", lambda **kwargs: None
    )
    monkeypatch.setattr(
        "onshape_robotics_toolkit.config.record_assembly_config", lambda **kwargs: None
    )

    target_error = ValidationError.from_exception_data(
        "Assembly",
        [
            {
                "type": "missing",
                "loc": ("rootAssembly", "features", 0, "mateConnectorCS"),
                "input": {"occurrence": ["occ-1"], "name": "frame_test"},
            }
        ],
    )

    validated_assembly = MagicMock()
    calls = {"count": 0}

    def _validate_side_effect(data):
        calls["count"] += 1
        if calls["count"] == 1:
            raise target_error
        return validated_assembly

    with patch(
        "onshape2xacro.optimized_cad.Assembly.model_validate",
        side_effect=_validate_side_effect,
    ):
        result = client.get_assembly(
            did="000000000000000000000001",
            wtype="w",
            wid="000000000000000000000004",
            eid="000000000000000000000002",
            with_meta_data=False,
        )

    assert result is validated_assembly
    assert calls["count"] == 2
    assert payload["rootAssembly"]["features"][0]["featureData"]["mateConnectorCS"][
        "origin"
    ] == [0.0, 0.0, 0.0]


def test_get_assembly_does_not_apply_fallback_for_non_target_validation_error(
    monkeypatch,
):
    client = object.__new__(OptimizedClient)
    client._url = "https://cad.onshape.com"

    payload: Any = {
        "rootAssembly": {
            "instances": [],
            "patterns": [],
            "occurrences": [],
            "features": [
                {
                    "id": "feature-1",
                    "suppressed": False,
                    "featureType": "mateConnector",
                    "featureData": {
                        "occurrence": ["occ-1"],
                    },
                }
            ],
            "fullConfiguration": "default",
            "configuration": "default",
            "documentId": "000000000000000000000001",
            "elementId": "000000000000000000000002",
            "documentMicroversion": "000000000000000000000003",
        },
        "subAssemblies": [],
        "parts": [],
        "partStudioFeatures": [],
    }

    response = MagicMock(status_code=200)
    response.json.return_value = payload
    client.request = MagicMock(return_value=response)

    monkeypatch.setattr(
        "onshape_robotics_toolkit.utilities.helpers.clean_json_numerics",
        lambda data, threshold, decimals: data,
    )
    monkeypatch.setattr(
        "onshape_robotics_toolkit.config.record_document_config", lambda **kwargs: None
    )
    monkeypatch.setattr(
        "onshape_robotics_toolkit.config.record_assembly_config", lambda **kwargs: None
    )

    non_target_error = ValidationError.from_exception_data(
        "Assembly",
        [
            {
                "type": "missing",
                "loc": ("rootAssembly", "features", 0, "name"),
                "input": {"occurrence": ["occ-1"]},
            }
        ],
    )

    with patch(
        "onshape2xacro.optimized_cad.Assembly.model_validate",
        side_effect=non_target_error,
    ):
        with pytest.raises(ValidationError):
            client.get_assembly(
                did="000000000000000000000001",
                wtype="w",
                wid="000000000000000000000004",
                eid="000000000000000000000002",
                with_meta_data=False,
            )

    assert (
        "mateConnectorCS" not in payload["rootAssembly"]["features"][0]["featureData"]
    )


def test_get_assembly_raises_clear_error_for_onshape_error_payload():
    client = object.__new__(OptimizedClient)
    client._url = "https://cad.onshape.com"

    response = MagicMock(status_code=400)
    response.json.return_value = {
        "message": "An illegal argument was provided",
        "status": 400,
        "errorCode": 0,
        "moreInfoUrl": "",
    }
    client.request = MagicMock(return_value=response)

    with pytest.raises(RuntimeError, match="An illegal argument was provided"):
        client.get_assembly(
            did="000000000000000000000001",
            wtype="w",
            wid="000000000000000000000004",
            eid="000000000000000000000002",
            configuration="original gripper",
            with_meta_data=False,
        )


def test_fetch_occurrences_deduplication(mock_cad, mock_client):
    # Setup: 2 instances of the SAME subassembly definition
    sub1_key = PathKey(path=("sub1",), name_path=("sub1",))
    sub2_key = PathKey(path=("sub2",), name_path=("sub2",))

    # Mock SubAssembly objects
    sub_asm = MagicMock(spec=SubAssembly)
    sub_asm.documentId = "d2"
    sub_asm.documentMicroversion = "mv2"
    sub_asm.elementId = "e2"
    sub_asm.isRigid = True
    sub_asm.RootOccurrences = None  # Initially None

    mock_cad.subassemblies = {sub1_key: sub_asm, sub2_key: sub_asm}

    # Mock Instances (not suppressed)
    inst1 = MagicMock()
    inst1.suppressed = False
    inst2 = MagicMock()
    inst2.suppressed = False
    mock_cad.instances = {sub1_key: inst1, sub2_key: inst2}

    # Mock RootAssembly return from client
    mock_root = MagicMock(spec=RootAssembly)
    mock_root.occurrences = []  # Return empty occurrences for simplicity
    mock_client.get_root_assembly.return_value = mock_root

    # Run
    asyncio.run(mock_cad.fetch_occurrences_for_subassemblies(mock_client))

    # Verify get_root_assembly called ONLY ONCE
    # Since we use asyncio.run, AsyncMock should track the await
    mock_client.get_root_assembly.assert_called_once()
    kwargs = mock_client.get_root_assembly.call_args.kwargs
    assert kwargs["did"] == "d2"
    assert kwargs["eid"] == "e2"


def test_fetch_occurrences_suppressed(mock_cad, mock_client):
    # Setup: 1 suppressed instance
    sub_key = PathKey(path=("sub1",), name_path=("sub1",))

    sub_asm = MagicMock(spec=SubAssembly)
    sub_asm.isRigid = True
    sub_asm.RootOccurrences = None

    mock_cad.subassemblies = {sub_key: sub_asm}

    inst = MagicMock()
    inst.suppressed = True  # SUPPRESSED
    mock_cad.instances = {sub_key: inst}

    # Run
    asyncio.run(mock_cad.fetch_occurrences_for_subassemblies(mock_client))

    # Verify get_root_assembly NOT called
    mock_client.get_root_assembly.assert_not_called()


def test_fetch_mate_limits_parsing(mock_cad, mock_client):
    # Setup
    feature_id = "feat1"
    sub_key = PathKey(path=("sub1",), name_path=("sub1",))

    # Add a non-rigid subassembly to trigger feature fetching
    sub_asm = MagicMock(spec=SubAssembly)
    sub_asm.isRigid = False
    sub_asm.documentId = "d2"
    sub_asm.documentMicroversion = "mv2"
    sub_asm.elementId = "e2"

    mock_cad.subassemblies = {sub_key: sub_asm}
    mock_cad.instances = {sub_key: MagicMock(suppressed=False)}

    # Add a mate mapped to this subassembly and feature
    mate_data = MagicMock(spec=MateFeatureData)
    mate_data.id = feature_id
    mate_data.name = "Joint 1"
    mate_data.mateType = "REVOLUTE"

    # Key in self.mates is (asm_key, feature_id, ...) ?
    # Logic: for (asm_key, _, _), mate in self.mates.items():
    # So key is 3-tuple
    mock_cad.mates = {(sub_key, feature_id, "foo"): mate_data}

    # Mock API response for features
    mock_feature = MagicMock()
    mock_feature.message.featureType = "mate"
    mock_feature.message.featureId = feature_id

    # Valid limits parameter dict
    mock_feature.message.parameter_dict.return_value = {
        "limitsEnabled": {"message": {"value": True}},
        "limitAxialZMin": {
            "typeName": "BTMParameterNullableQuantity",
            "message": {"isNull": False, "expression": "-1.57"},
        },
        "limitAxialZMax": {
            "typeName": "BTMParameterNullableQuantity",
            "message": {"isNull": False, "expression": "1.57"},
        },
    }

    mock_features_resp = MagicMock()
    mock_features_resp.features = [mock_feature]
    mock_client.get_features.return_value = mock_features_resp

    # Patch helper to return floats
    with (
        patch("onshape2xacro.optimized_cad.parse_onshape_expression") as mock_parse,
        patch("onshape2xacro.optimized_cad.update_mate_limits"),
    ):
        mock_parse.side_effect = lambda x: float(x)

        # Run
        mock_cad.fetch_mate_limits(mock_client)

        # Verify limits updated
        assert mate_data.limits == {"min": -1.57, "max": 1.57}
        mock_client.get_features.assert_called()


def test_fetch_mass_properties_mixed(mock_cad, mock_client):
    # Setup
    # Part 1: Normal part
    p1_key = PathKey(path=("p1",), name_path=("p1",))
    p1 = MagicMock(spec=Part)
    p1.bodyType = "solid"
    p1.isRigidAssembly = False
    p1.documentId = "d2"
    p1.documentMicroversion = "mv2"
    p1.elementId = "e2"
    p1.partId = "pid1"
    p1.MassProperty = None
    p1.rigidAssemblyToPartTF = None

    # Part 2: Rigid Assembly
    p2_key = PathKey(path=("p2",), name_path=("p2",))
    p2 = MagicMock(spec=Part)
    p2.bodyType = "solid"
    p2.isRigidAssembly = True
    p2.documentId = "d3"
    p2.rigidAssemblyWorkspaceId = "w3"
    p2.elementId = "e3"
    p2.MassProperty = None
    p2.rigidAssemblyToPartTF = None

    mock_cad.parts = {p1_key: p1, p2_key: p2}
    mock_cad.instances = {
        p1_key: MagicMock(suppressed=False),
        p2_key: MagicMock(suppressed=False),
    }

    # Mock API returns
    mass_p1 = MagicMock()
    mass_p2 = MagicMock()

    mock_client.get_mass_property.return_value = mass_p1
    mock_client.get_assembly_mass_properties.return_value = mass_p2

    # Run
    asyncio.run(mock_cad.fetch_mass_properties_for_parts(mock_client))

    # Verify assignments
    assert p1.MassProperty == mass_p1
    assert p2.MassProperty == mass_p2

    # Verify API calls
    mock_client.get_mass_property.assert_called()
    mock_client.get_assembly_mass_properties.assert_called()
